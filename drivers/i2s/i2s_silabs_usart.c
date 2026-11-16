/*
 * Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * EFR32 USART I2S driver (stereo/mono TX, stereo RX).
 *
 * BCLK and LRCLK (WS) toggle only while TX data is shifting out on this USART
 * master. RX alone does not generate clocks; full-duplex needs a TX stream
 * (or an external bit clock). configure() requires I2S_OPT_BIT_CLK_GATED;
 * I2S_OPT_BIT_CLK_CONT is rejected with -EINVAL. I2S_DIR_BOTH is -ENOSYS.
 *
 * The hardware always emits two channel slots per LRCLK frame on the wire.
 *
 * USART DMA request naming (RM / SoC headers):
 *   TXBL      = Transmit Buffer Level (NOT "left"). Fires when the TX FIFO has
 *               space for another sample. With I2SCTRL.DMASPLIT=0 it requests
 *               one LDMA move for the next word(s) on the bus (stereo: L+R).
 *   TXBLRIGHT = Separate DMA request when DMASPLIT=1; targets the right I2S
 *               slot only. The "RIGHT" suffix is explicit; TXBL itself is not
 *               an abbreviation for "TX buffer left".
 *   RXDATAV   = Receive data available (DMASPLIT=0 stereo RX from RXDOUBLE).
 *
 * Stereo TX (configure channels == 2, I2SCTRL.DMASPLIT = 0):
 *   One LDMA channel on USARTnTXBL writes interleaved L/R samples to
 *   USARTn_TXDOUBLE. Buffer: [L0, R0, L1, R1, ...].
 *
 * Stereo RX (channels == 2, DMASPLIT = 0):
 *   One LDMA channel on USARTnRXDATAV reads RXDOUBLE into the same layout.
 *   Gapless chaining uses silabs_ldma_append_block() like TX.
 *
 * Mono TX (configure channels == 1, I2SCTRL.DMASPLIT = 1):
 *   Two LDMA channels (devicetree "txbl" + "txblright"):
 *     - USARTnTXBL:      coincidentally the LRCLK-low / left wire slot
 *     - USARTnTXBLRIGHT: LRCLK-high / right wire slot
 *   Slot assignment comes from silabs,mono-tx-slot (default right).
 *   Application buffer: [M0, M1, ...] (one sample per LRCLK frame).
 *   I2SCTRL.MONO stays 0; LRCLK still toggles L/R on the bus.
 *
 * With DATABITS=16, USARTn_TXDATA loads only 8 FIFO bits per write; a full
 * I2S slot needs USARTn_TXDOUBLE (see RM §22.3.2.6 / §22.3.2.19). Stereo
 * and mono TX both target TXDOUBLE; mono moves one slot per DMA trigger.
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_silabs_ldma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>

#include <sl_hal_usart.h>
#include <sl_hal_ldma.h>

#define DT_DRV_COMPAT silabs_usart_i2s

#define I2S_SILABS_SUPPORTED_OPTIONS\
	(I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_GATED)

#define I2S_SILABS_BLOCK_Q_MAX_DEPTH\
	MAX(CONFIG_I2S_SILABS_USART_TX_BLOCK_COUNT, CONFIG_I2S_SILABS_USART_RX_BLOCK_COUNT)

/* Stereo: every LRCLK frame carries L + R = 2 channel slots. */
#define I2S_STEREO_SLOTS_PER_FRAME 2U

/* USART I2S data word is 16 bits (SL_HAL_USART_I2S_FORMAT_W16D16). */
#define I2S_SILABS_WORD_SIZE_BITS  16U
#define I2S_SILABS_WORD_SIZE_BYTES 2U

/*
 * One LDMA descriptor holds at most SL_HAL_LDMA_DESCRIPTOR_MAX_XFER_SIZE
 * transfers of I2S_SILABS_WORD_SIZE_BYTES. dma_silabs_ldma splits larger
 * dma_block_config.block_size values into linked descriptors and fires
 * complete_callback per descriptor; this driver frees the block on each
 * callback. silabs_ldma_append_block() also rejects oversized blocks.
 */
#define I2S_SILABS_LDMA_MAX_BLOCK_BYTES\
	(SL_HAL_LDMA_DESCRIPTOR_MAX_XFER_SIZE * I2S_SILABS_WORD_SIZE_BYTES)

/*
 * Minimum sample rate accepted by configure().  Below 8 kHz the producer-side
 * timing tolerance shrinks and the LDMA TXBL trigger latency dominates audio
 * quality.  Most I2S codecs (e.g. TAS2505) also spec >= 8 kHz.
 */
#define I2S_MIN_FRAME_CLK_HZ 8000U

/*
 * LDMA channel priority (0 = highest, 3 = lowest in the Silabs LDMA driver).
 * I2S TX must keep up with the codec but other DMA users (UART console,
 * crypto, etc.) typically have shorter deadlines, so I2S sits at the lowest
 * priority slot.
 */
#define I2S_DMA_CHANNEL_PRIORITY 3U

/*
 * Boolean for dma_config.complete_callback_en (1 = invoke dma_callback on
 * each block completion -- required for our gapless append/chain logic).
 */
#define I2S_DMA_COMPLETE_CB_ENABLED 1U

enum i2s_silabs_usart_mono_tx_slot {
	I2S_SILABS_USART_MONO_TX_SLOT_LEFT = 0,
	I2S_SILABS_USART_MONO_TX_SLOT_RIGHT = 1,
};

/* One queued I2S memory block (pointer + byte length). Stored by value in a
 * Zephyr k_msgq for ISR/thread-safe block handoff.
 */
struct i2s_silabs_usart_block {
	void *blk;
	size_t len;
};

/*
 * Per-stream DMA runtime state only. Descriptor templates (dma_cfg / blk_cfg)
 * live in instance-static storage pointed from i2s_silabs_usart_cfg.
 */
struct i2s_silabs_usart_dma {
	int channel;
	bool busy;
};

struct i2s_silabs_usart_dma_desc {
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
};

struct i2s_silabs_usart_stream {
	int32_t state;
	bool cfg_valid;
	/*
	 * When state is STOPPING: true = I2S_TRIGGER_DRAIN (empty the TX queue),
	 * false = I2S_TRIGGER_STOP (finish the current/pending block only).
	 */
	bool drain;
	/*
	 * Gated BCLK stopped (TX underrun) while RX is STOPPING. The block in
	 * active still completes and is queued; the already-chained next block
	 * is dropped because it can never finish without a bit clock.
	 */
	bool stop_after_active;
	struct i2s_config cfg;
	struct k_msgq q;
	char q_buf[I2S_SILABS_BLOCK_Q_MAX_DEPTH * sizeof(struct i2s_silabs_usart_block)];
	/*
	 * One-slot undo for a failed silabs_ldma_append_block(): the block was
	 * already accepted by i2s_write() and must stay at the queue head.
	 * .blk == NULL means empty. Checked before k_msgq in TX get paths.
	 */
	struct i2s_silabs_usart_block q_unpop;
	void *active;
	void *pending;
	struct i2s_silabs_usart_dma dma;
	struct k_sem sem;
};

struct i2s_silabs_usart_data {
	struct i2s_silabs_usart_stream tx;
	struct i2s_silabs_usart_stream rx;
	struct i2s_silabs_usart_dma tx_silence;
	struct k_mutex cfg_lock;
	struct k_spinlock lock;
};

struct i2s_silabs_usart_cfg {
	USART_TypeDef *base;
	const struct device *clock_dev;
	struct silabs_clock_control_cmu_config clock_cfg;
	const struct pinctrl_dev_config *pcfg;
	const struct device *dma_dev;
	uint32_t dma_txbl_slot;      /* LDMA slot: USARTnTXBL (buffer level, not "left") */
	uint32_t dma_txblright_slot; /* LDMA slot: USARTnTXBLRIGHT (mono DMASPLIT only) */
	uint32_t dma_rxdatav_slot;   /* LDMA slot: USARTnRXDATAV */
	struct i2s_silabs_usart_dma_desc *tx_dma;
	/* NULL when mono TX (txblright) is not wired in DT. */
	struct i2s_silabs_usart_dma_desc *tx_silence_dma;
	/* NULL when RX (rxdatav) is not wired in DT. */
	struct i2s_silabs_usart_dma_desc *rx_dma;
	const struct device *mclk_dev;      /* optional CMU CLKOUT providing codec MCLK */
	clock_control_subsys_t mclk_subsys; /* output index for series-clock-output */
	bool has_tx_split;
	bool has_rx;
	uint8_t mono_tx_default;
};

/*
 * Shared mono pad word (BSS / RAM so LDMA can always read it). Always zero;
 * never written. Not per-device -- one word for all instances.
 */
static int16_t i2s_silabs_usart_silence_word;

/*
 * Gate codec MCLK with the standard clock_control API: enabled before I2S
 * BCLK/LRCLK are started, disabled after they are stopped on de-configure.
 * Wired via silabs,mclk-out. The phandle is a clock-output@N child under
 * silabs,series-clock-output; the device is the parent &clkout and subsys is
 * the child's reg (CLKOUT index).
 */
static void i2s_silabs_usart_mclk_set(const struct device *mclk, clock_control_subsys_t subsys,
				      bool enable)
{
	if (mclk == NULL) {
		return;
	}
	if (enable) {
		(void)clock_control_on(mclk, subsys);
	} else {
		(void)clock_control_off(mclk, subsys);
	}
}

/*
 * TX blocks must come from cfg.mem_slab (i2s_write contract). The driver takes
 * ownership and always returns the block to that slab when done.
 */
static void i2s_silabs_usart_tx_release_block(struct i2s_silabs_usart_stream *tx, void *blk)
{
	if (blk == NULL) {
		return;
	}

	k_mem_slab_free(tx->cfg.mem_slab, blk);
	k_sem_give(&tx->sem);
}

static inline bool i2s_silabs_usart_tx_cfg_is_mono(const struct i2s_config *cfg)
{
	return cfg->channels == 1U;
}

static inline bool i2s_silabs_usart_tx_path_is_mono(const struct device *dev)
{
	const struct i2s_silabs_usart_data *data = dev->data;

	return data->tx.cfg_valid && i2s_silabs_usart_tx_cfg_is_mono(&data->tx.cfg);
}

static inline int i2s_silabs_usart_block_q_put(struct k_msgq *q, void *blk, size_t len)
{
	struct i2s_silabs_usart_block item = {.blk = blk, .len = len};

	return k_msgq_put(q, &item, K_NO_WAIT);
}

static inline int i2s_silabs_usart_block_q_get(struct k_msgq *q, void **blk, size_t *len)
{
	struct i2s_silabs_usart_block item = {0};
	int ret = k_msgq_get(q, &item, K_NO_WAIT);

	if (ret == 0) {
		*blk = item.blk;
		*len = item.len;
	}
	return ret;
}

static inline bool i2s_silabs_usart_tx_q_empty(struct i2s_silabs_usart_stream *tx)
{
	return tx->q_unpop.blk == NULL && k_msgq_num_used_get(&tx->q) == 0U;
}

static inline int i2s_silabs_usart_tx_q_get(struct i2s_silabs_usart_stream *tx, void **blk,
					    size_t *len)
{
	if (tx->q_unpop.blk != NULL) {
		*blk = tx->q_unpop.blk;
		*len = tx->q_unpop.len;
		tx->q_unpop.blk = NULL;
		tx->q_unpop.len = 0U;
		return 0;
	}

	return i2s_silabs_usart_block_q_get(&tx->q, blk, len);
}

/* Return a just-popped TX block to the head of the queue (next one out). */
static inline int i2s_silabs_usart_tx_q_unpop(struct i2s_silabs_usart_stream *tx, void *blk,
					      size_t len)
{
	if (tx->q_unpop.blk != NULL) {
		return -ENOMEM;
	}

	tx->q_unpop.blk = blk;
	tx->q_unpop.len = len;
	return 0;
}

static void i2s_silabs_usart_tx_finish_stopping_if_quiescent(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *tx = &data->tx;

	if (tx->state != I2S_STATE_STOPPING) {
		return;
	}
	if (tx->active != NULL || tx->pending != NULL) {
		return;
	}
	/* DRAIN must empty the queue; STOP leaves remaining blocks for later START. */
	if (tx->drain && !i2s_silabs_usart_tx_q_empty(tx)) {
		return;
	}
	/*
	 * With gapless LDMA chaining, tx->active may already be NULL while the
	 * channel is still marked busy until the final block completes.  Force
	 * the channel idle so STOPPING cannot wedge.
	 */
	if (tx->dma.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)tx->dma.channel);
		tx->dma.busy = false;
	}
	if (i2s_silabs_usart_tx_path_is_mono(dev) && data->tx_silence.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
		data->tx_silence.busy = false;
	}
	tx->drain = false;
	tx->state = I2S_STATE_READY;
}

static inline void i2s_silabs_usart_apply_dma_xfer_size(struct dma_config *dma_cfg)
{
	dma_cfg->dest_data_size = I2S_SILABS_WORD_SIZE_BYTES;
	dma_cfg->source_data_size = I2S_SILABS_WORD_SIZE_BYTES;
	dma_cfg->source_burst_length = I2S_SILABS_WORD_SIZE_BYTES;
	dma_cfg->dest_burst_length = I2S_SILABS_WORD_SIZE_BYTES;
}

static void i2s_silabs_usart_tx_dma_apply_stereo(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_dma_desc *tx_dma = pcfg->tx_dma;

	i2s_silabs_usart_apply_dma_xfer_size(&tx_dma->dma_cfg);
	/*
	 * DMASPLIT=0: USARTnTXBL (Transmit Buffer Level) triggers one LDMA move for
	 * interleaved L+R — not the left channel alone.
	 */
	tx_dma->dma_cfg.dma_slot = pcfg->dma_txbl_slot;
	tx_dma->blk_cfg.dest_address = (uintptr_t)&pcfg->base->TXDOUBLE;
	tx_dma->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	tx_dma->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
}

static void i2s_silabs_usart_tx_dma_apply_mono(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_dma_desc *tx_dma = pcfg->tx_dma;
	struct i2s_silabs_usart_dma_desc *silence = pcfg->tx_silence_dma;
	const bool audio_on_txbl = (pcfg->mono_tx_default == I2S_SILABS_USART_MONO_TX_SLOT_LEFT);
	const uint32_t audio_slot = audio_on_txbl ? pcfg->dma_txbl_slot : pcfg->dma_txblright_slot;
	const uint32_t pad_slot = audio_on_txbl ? pcfg->dma_txblright_slot : pcfg->dma_txbl_slot;

	i2s_silabs_usart_apply_dma_xfer_size(&tx_dma->dma_cfg);
	i2s_silabs_usart_apply_dma_xfer_size(&silence->dma_cfg);

	tx_dma->dma_cfg.dma_slot = audio_slot;
	tx_dma->blk_cfg.dest_address = (uintptr_t)&pcfg->base->TXDOUBLE;
	tx_dma->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	tx_dma->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	silence->dma_cfg.dma_slot = pad_slot;
	silence->blk_cfg.dest_address = (uintptr_t)&pcfg->base->TXDOUBLE;
	silence->blk_cfg.source_address = (uintptr_t)&i2s_silabs_usart_silence_word;
	silence->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	silence->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	silence->blk_cfg.block_size = I2S_SILABS_WORD_SIZE_BYTES;
}

static void i2s_silabs_usart_hw_disable_data_irqs(USART_TypeDef *base)
{
	sl_hal_usart_disable_interrupts(base, USART_IF_TXBL | USART_IF_RXDATAV);
	sl_hal_usart_clear_interrupts(base, USART_IF_TXBL | USART_IF_RXDATAV);
}

static void i2s_silabs_usart_hw_disable_error_irqs(USART_TypeDef *base)
{
	sl_hal_usart_disable_interrupts(base, USART_IF_TXUF | USART_IF_RXOF);
	sl_hal_usart_clear_interrupts(base, USART_IF_TXUF | USART_IF_RXOF);
}

static void i2s_silabs_usart_hw_enable_rxof_irq(USART_TypeDef *base)
{
	sl_hal_usart_clear_interrupts(base, USART_IF_RXOF);
	sl_hal_usart_enable_interrupts(base, USART_IF_RXOF);
}

static bool i2s_silabs_usart_cfg_bus_match(const struct i2s_config *a, const struct i2s_config *b)
{
	return a->frame_clk_freq == b->frame_clk_freq && a->word_size == b->word_size &&
	       a->channels == b->channels && a->format == b->format;
}

/*
 * Compute USART CLKDIV for sync (I2S) mode with fractional precision.
 *
 * sl_hal_usart_sync_calculate_clock_div() only yields the integer divider
 * (clkdiv = (ref/(2*br)) << 8), which leaves the fractional sub-field at 0.
 *
 * RM: br = (128 * ref) / (256 + CLKDIV)  =>  CLKDIV = round(128*ref/br) - 256
 *
 * Returns 0 and stores the divider in *clkdiv_out on success, or -EINVAL when
 * ref_hz or baud_hz is zero, or when baud_hz exceeds what the USART divider
 * can reach (CLKDIV would clamp to the minimum).
 */
static int i2s_silabs_usart_sync_clkdiv(uint32_t ref_hz, uint32_t baud_hz, uint32_t *clkdiv_out)
{
	uint64_t clkdiv;

	if (ref_hz == 0U || baud_hz == 0U) {
		return -EINVAL;
	}

	clkdiv = ((uint64_t)128U * ref_hz + (uint64_t)baud_hz / 2U) / (uint64_t)baud_hz;

	if (clkdiv < 256U) {
		return -EINVAL;
	}
	clkdiv -= 256U;

#if defined(_USART_CLKDIV_DIVEXT_MASK)
	*clkdiv_out = (uint32_t)(clkdiv & (_USART_CLKDIV_DIV_MASK | _USART_CLKDIV_DIVEXT_MASK));
#else
	*clkdiv_out = (uint32_t)(clkdiv & _USART_CLKDIV_DIV_MASK);
#endif
	return 0;
}

static int i2s_silabs_usart_validate_frame_clk_hz(uint32_t ref_hz, uint32_t frame_clk_hz)
{
	uint64_t bit_hz;
	uint64_t max_bit_hz;

	if (frame_clk_hz < I2S_MIN_FRAME_CLK_HZ) {
		return -EINVAL;
	}

	bit_hz = (uint64_t)frame_clk_hz * I2S_STEREO_SLOTS_PER_FRAME *
		 (uint64_t)I2S_SILABS_WORD_SIZE_BITS;
	if (bit_hz == 0U) {
		return -EINVAL;
	}

	if (ref_hz != 0U) {
		/* RM: minimum CLKDIV is 0, so max SCLK is ref_hz / 2. */
		max_bit_hz = (uint64_t)ref_hz / 2U;
		if (bit_hz > max_bit_hz) {
			return -EINVAL;
		}
	}

	return 0;
}

static int i2s_silabs_usart_validate_i2s_cfg(const struct i2s_config *ic)
{
	if ((ic->options & I2S_SILABS_SUPPORTED_OPTIONS) != ic->options) {
		return -EINVAL;
	}
	if ((ic->options & I2S_OPT_BIT_CLK_TARGET) || (ic->options & I2S_OPT_FRAME_CLK_TARGET)) {
		return -EINVAL;
	}
	/*
	 * BCLK/LRCLK run only while TX data is on the wire; there is no free-running
	 * clock before START or with an empty queue. Require I2S_OPT_BIT_CLK_GATED.
	 */
	if ((ic->options & I2S_OPT_BIT_CLK_GATED) == 0U) {
		return -EINVAL;
	}
	/* USART DATABITS max is 16; word_size is the data word, not the slot. */
	if (ic->word_size != I2S_SILABS_WORD_SIZE_BITS) {
		return -EINVAL;
	}
	if (i2s_silabs_usart_validate_frame_clk_hz(0U, ic->frame_clk_freq) != 0) {
		return -EINVAL;
	}
	if (ic->channels != 1U && ic->channels != 2U) {
		return -EINVAL;
	}

	switch (ic->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		break;
	default:
		return -EINVAL;
	}

	/* USART I2S is MSB-first; WS polarity is not inverted. */
	if ((ic->format & I2S_FMT_DATA_ORDER_LSB) != 0U) {
		return -EINVAL;
	}
	if ((ic->format & I2S_FMT_FRAME_CLK_INV) != 0U) {
		return -EINVAL;
	}

	return 0;
}

static int i2s_silabs_usart_validate_config(const struct device *dev,
					    const struct i2s_config **out_ic, uint32_t *ref_hz)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	const struct i2s_silabs_usart_data *data = dev->data;
	const struct i2s_config *ic;
	int err;

	if (!data->tx.cfg_valid && !data->rx.cfg_valid) {
		return -EINVAL;
	}

	err = clock_control_get_rate(pcfg->clock_dev,
				     (clock_control_subsys_t)(uintptr_t)&pcfg->clock_cfg, ref_hz);
	if (err < 0) {
		return err;
	}
	if (*ref_hz == 0U) {
		return -EINVAL;
	}

	ic = data->tx.cfg_valid ? &data->tx.cfg : &data->rx.cfg;
	err = i2s_silabs_usart_validate_i2s_cfg(ic);
	if (err < 0) {
		return err;
	}
	err = i2s_silabs_usart_validate_frame_clk_hz(*ref_hz, ic->frame_clk_freq);
	if (err < 0) {
		return err;
	}

	*out_ic = ic;
	return 0;
}

static int i2s_silabs_usart_hw_apply(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	const struct i2s_config *ic;
	sl_hal_usart_i2s_init_t init = SL_HAL_USART_INIT_I2S_DEFAULT;
	uint32_t ref_hz = 0;
	uint32_t bit_hz;
	uint32_t clkdiv;
	int err;

	err = i2s_silabs_usart_validate_config(dev, &ic, &ref_hz);
	if (err < 0) {
		return err;
	}

	/*
	 * Required physical SCLK (bit clock):
	 *
	 *   bit_hz = frame_clk_freq * SLOTS_PER_FRAME * word_size
	 *
	 * SLOTS_PER_FRAME is hard-wired to 2 (stereo) -- every LRCLK period
	 * carries L + R, each `word_size` bits wide.
	 *
	 * Example: frame_clk=16 kHz, word_size=16  -> bit_hz = 16k*2*16 = 512 kHz.
	 *
	 * Converted to a CLKDIV via sl_hal_usart_sync_calculate_clock_div() so the
	 * USART produces SCLK == bit_hz on the wire.
	 */
	bit_hz = ic->frame_clk_freq * I2S_STEREO_SLOTS_PER_FRAME * (uint32_t)ic->word_size;

	if (data->tx.cfg_valid) {
		if (i2s_silabs_usart_tx_cfg_is_mono(&data->tx.cfg)) {
			i2s_silabs_usart_tx_dma_apply_mono(dev);
		} else {
			i2s_silabs_usart_tx_dma_apply_stereo(dev);
		}
	}

	i2s_silabs_usart_hw_disable_data_irqs(pcfg->base);

	init.sync.clock_div = sl_hal_usart_sync_calculate_clock_div(ref_hz, bit_hz);
	init.sync.data_bits = SL_HAL_USART_DATA_BITS_16;
	init.sync.master = true;
	init.sync.msb_first = true;
	init.sync.clock_mode =
		((ic->format & I2S_FMT_CLK_FORMAT_MASK) == I2S_FMT_CLK_NF_IB) ||
				((ic->format & I2S_FMT_CLK_FORMAT_MASK) == I2S_FMT_CLK_IF_IB)
			? SL_HAL_USART_CLOCK_MODE_1
			: SL_HAL_USART_CLOCK_MODE_0;
	init.format = SL_HAL_USART_I2S_FORMAT_W16D16;
	init.justify = SL_HAL_USART_JUSTIFY_LEFT;
	/*
	 * Wire format stays stereo (MONO=0): LRCLK toggles L/R. Mono TX uses
	 * DMASPLIT + dual LDMA (samples on one slot, silence on the other).
	 */
	init.mono = false;
	init.dma_split = data->tx.cfg_valid && i2s_silabs_usart_tx_cfg_is_mono(&data->tx.cfg);

	if ((ic->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_I2S) {
		init.delay = true;
	} else {
		init.delay = false;
	}

	sl_hal_usart_reset(pcfg->base);
	sl_hal_usart_init_i2s(pcfg->base, &init);

	/*
	 * Override the integer-only CLKDIV that sl_hal_usart_init_i2s() programs so
	 * BCLK matches bit_hz as closely as the USART fractional divider allows.
	 * Only apply the override when i2s_silabs_usart_sync_clkdiv() returns a valid
	 * value; otherwise keep the divider the init function programmed.
	 */
	if (i2s_silabs_usart_sync_clkdiv(ref_hz, bit_hz, &clkdiv) == 0) {
		pcfg->base->CLKDIV = clkdiv;
	}

	sl_hal_usart_enable(pcfg->base);
	if (data->tx.cfg_valid) {
		sl_hal_usart_enable_tx(pcfg->base);
	}
	if (data->rx.cfg_valid) {
		sl_hal_usart_enable_rx(pcfg->base);
		i2s_silabs_usart_hw_enable_rxof_irq(pcfg->base);
	}

	return 0;
}

static k_timeout_t i2s_silabs_usart_cfg_timeout(int32_t t)
{
	if (t == SYS_FOREVER_MS) {
		return K_FOREVER;
	}
	if (t == 0) {
		return K_NO_WAIT;
	}
	return SYS_TIMEOUT_MS(t);
}

static void i2s_silabs_usart_tx_drop(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	void *blk;
	size_t len;

	if (data->tx.dma.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)data->tx.dma.channel);
		data->tx.dma.busy = false;
	}
	if (i2s_silabs_usart_tx_path_is_mono(dev) && data->tx_silence.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
		data->tx_silence.busy = false;
	}
	if (data->tx.pending != NULL) {
		i2s_silabs_usart_tx_release_block(&data->tx, data->tx.pending);
		data->tx.pending = NULL;
	}
	if (data->tx.active != NULL) {
		i2s_silabs_usart_tx_release_block(&data->tx, data->tx.active);
		data->tx.active = NULL;
	}

	while (i2s_silabs_usart_tx_q_get(&data->tx, &blk, &len) == 0) {
		i2s_silabs_usart_tx_release_block(&data->tx, blk);
	}
	data->tx.drain = false;
}

static void i2s_silabs_usart_rx_release_block(struct i2s_silabs_usart_stream *rx, void *blk)
{
	if (blk == NULL) {
		return;
	}

	k_mem_slab_free(rx->cfg.mem_slab, blk);
}

static void i2s_silabs_usart_rx_wake_error(struct i2s_silabs_usart_stream *rx)
{
	rx->state = I2S_STATE_ERROR;
	k_sem_give(&rx->sem);
}

static void i2s_silabs_usart_rx_drop(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	void *blk;
	size_t len;

	if (rx->dma.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
		rx->dma.busy = false;
	}
	if (rx->pending != NULL) {
		i2s_silabs_usart_rx_release_block(rx, rx->pending);
		rx->pending = NULL;
	}
	if (rx->active != NULL) {
		i2s_silabs_usart_rx_release_block(rx, rx->active);
		rx->active = NULL;
	}

	while (i2s_silabs_usart_block_q_get(&rx->q, &blk, &len) == 0) {
		i2s_silabs_usart_rx_release_block(rx, blk);
	}
	rx->drain = false;
	rx->stop_after_active = false;
	k_sem_reset(&rx->sem);
}

static void i2s_silabs_usart_rx_finish_stopping_if_quiescent(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;

	if (rx->state != I2S_STATE_STOPPING) {
		return;
	}
	if (rx->active != NULL || rx->pending != NULL) {
		return;
	}
	if (rx->dma.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
		rx->dma.busy = false;
	}
	rx->drain = false;
	rx->state = I2S_STATE_READY;
}

static int i2s_silabs_usart_rx_dma_config_start(const struct device *dev, void *blk)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	int ret;

	ret = dma_config(cfg->dma_dev, (uint32_t)rx->dma.channel, &cfg->rx_dma->dma_cfg);
	if (ret < 0) {
		rx->dma.busy = false;
		rx->active = NULL;
		i2s_silabs_usart_rx_release_block(rx, blk);
		i2s_silabs_usart_rx_wake_error(rx);
		return ret;
	}

	ret = dma_start(cfg->dma_dev, (uint32_t)rx->dma.channel);
	if (ret < 0) {
		dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
		rx->dma.busy = false;
		rx->active = NULL;
		i2s_silabs_usart_rx_release_block(rx, blk);
		i2s_silabs_usart_rx_wake_error(rx);
		return ret;
	}

	return 0;
}

static void i2s_silabs_usart_rx_try_start(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	void *blk = NULL;
	int ret;
	k_spinlock_key_t key;

	if (rx->state == I2S_STATE_RUNNING) {
		/* continue */
	} else if (rx->state == I2S_STATE_STOPPING && rx->drain) {
		/* keep receiving until STOP settles */
	} else {
		return;
	}

	key = k_spin_lock(&data->lock);

	if (rx->active == NULL) {
		ret = k_mem_slab_alloc(rx->cfg.mem_slab, &blk, K_NO_WAIT);
		if (ret < 0) {
			k_spin_unlock(&data->lock, key);
			if (rx->state == I2S_STATE_RUNNING) {
				i2s_silabs_usart_rx_wake_error(rx);
			} else {
				i2s_silabs_usart_rx_finish_stopping_if_quiescent(dev);
			}
			return;
		}

		rx->active = blk;
		rx->dma.busy = true;
		cfg->rx_dma->blk_cfg.dest_address = (uintptr_t)blk;
		cfg->rx_dma->blk_cfg.block_size = rx->cfg.block_size;

		ret = i2s_silabs_usart_rx_dma_config_start(dev, blk);
		k_spin_unlock(&data->lock, key);
		if (ret < 0) {
			return;
		}

		/* Double-buffer immediately for a gapless first transition. */
		i2s_silabs_usart_rx_try_start(dev);
		return;
	}

	if (rx->pending != NULL) {
		k_spin_unlock(&data->lock, key);
		return;
	}

	ret = k_mem_slab_alloc(rx->cfg.mem_slab, &blk, K_NO_WAIT);
	if (ret < 0) {
		k_spin_unlock(&data->lock, key);
		return;
	}

	cfg->rx_dma->blk_cfg.dest_address = (uintptr_t)blk;
	cfg->rx_dma->blk_cfg.block_size = rx->cfg.block_size;

	ret = silabs_ldma_append_block(cfg->dma_dev, (uint32_t)rx->dma.channel,
				       &cfg->rx_dma->dma_cfg);
	if (ret == 0) {
		rx->pending = blk;
		k_spin_unlock(&data->lock, key);
	} else {
		k_spin_unlock(&data->lock, key);
		i2s_silabs_usart_rx_release_block(rx, blk);
	}
}

static void i2s_silabs_usart_rx_abort_stream(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;

	dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
	rx->dma.busy = false;
	if (rx->pending != NULL) {
		i2s_silabs_usart_rx_release_block(rx, rx->pending);
		rx->pending = NULL;
	}
	if (rx->active != NULL) {
		i2s_silabs_usart_rx_release_block(rx, rx->active);
		rx->active = NULL;
	}
	i2s_silabs_usart_rx_wake_error(rx);
}

/*
 * RX is STOPPING and the bit clock is already gone, so the block now in
 * `active` will never complete. Drop it and return to READY. Do not call this
 * while `pending` is still set: that block has not been retired by the DMA
 * callback yet and the caller still expects it on i2s_read().
 */
static void i2s_silabs_usart_rx_drop_stalled(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	void *stalled;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	if (!rx->cfg_valid || rx->state != I2S_STATE_STOPPING || rx->pending != NULL) {
		k_spin_unlock(&data->lock, key);
		return;
	}

	dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
	rx->dma.busy = false;
	stalled = rx->active;
	rx->active = NULL;
	rx->drain = false;
	rx->stop_after_active = false;
	rx->state = I2S_STATE_READY;
	k_spin_unlock(&data->lock, key);

	i2s_silabs_usart_rx_release_block(rx, stalled);
	sl_hal_usart_clear_rx(cfg->base);
	sl_hal_usart_clear_interrupts(cfg->base, USART_IF_RXOF | USART_IF_RXDATAV |
							  USART_IF_RXFULL);
}

static void i2s_silabs_usart_tx_underrun(const struct device *dev)
{
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	bool drop_stalled;
	k_spinlock_key_t key;

	data->tx.state = I2S_STATE_ERROR;

	/*
	 * Master BCLK stops with the last TX word. If RX STOP already armed a
	 * second block, that block cannot finish. Keep the block the DMA
	 * callback has not retired yet.
	 */
	key = k_spin_lock(&data->lock);
	drop_stalled = false;
	if (rx->state == I2S_STATE_STOPPING) {
		if (rx->pending != NULL) {
			rx->stop_after_active = true;
		} else {
			drop_stalled = true;
		}
	}
	k_spin_unlock(&data->lock, key);

	if (drop_stalled) {
		i2s_silabs_usart_rx_drop_stalled(dev);
	}
}

static void i2s_silabs_usart_dma_rx_cb(const struct device *dma_dev, void *user_data,
				       uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	void *done;
	k_spinlock_key_t key;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		i2s_silabs_usart_rx_abort_stream(dev);
		return;
	}

	key = k_spin_lock(&data->lock);
	done = rx->active;

	if (rx->pending != NULL) {
		/*
		 * silabs_ldma_append_block() chained the pending block: the
		 * LDMA has already auto-started it via LINKLOAD before this
		 * callback fires. Do NOT call dma_stop() -- the channel is
		 * still running.
		 */
		rx->active = rx->pending;
		rx->pending = NULL;
	} else {
		dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
		rx->active = NULL;
		rx->dma.busy = false;
	}

	/*
	 * TX underrun stopped BCLK. Queue the block that just finished, then
	 * drop the chained one that cannot complete.
	 */
	if (rx->stop_after_active && rx->state == I2S_STATE_STOPPING) {
		void *chained = rx->active;

		dma_stop(cfg->dma_dev, (uint32_t)rx->dma.channel);
		rx->dma.busy = false;
		rx->active = NULL;
		rx->pending = NULL;
		rx->drain = false;
		rx->stop_after_active = false;
		rx->state = I2S_STATE_READY;
		k_spin_unlock(&data->lock, key);
		i2s_silabs_usart_rx_release_block(rx, chained);
	} else {
		k_spin_unlock(&data->lock, key);
	}

	if (done != NULL) {
		if (i2s_silabs_usart_block_q_put(&rx->q, done, rx->cfg.block_size) == 0) {
			k_sem_give(&rx->sem);
		} else {
			i2s_silabs_usart_rx_release_block(rx, done);
			i2s_silabs_usart_rx_wake_error(rx);
			return;
		}
	}

	if (rx->state == I2S_STATE_RUNNING || (rx->state == I2S_STATE_STOPPING && rx->drain)) {
		i2s_silabs_usart_rx_try_start(dev);
	}
	i2s_silabs_usart_rx_finish_stopping_if_quiescent(dev);
}

static void i2s_silabs_usart_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	uint32_t flags = sl_hal_usart_get_enabled_pending_interrupts(cfg->base);

	if ((flags & USART_IF_RXOF) != 0U) {
		sl_hal_usart_clear_interrupts(cfg->base, USART_IF_RXOF);
		if (!data->rx.cfg_valid || data->rx.state == I2S_STATE_ERROR) {
			return;
		}
		if (data->rx.state == I2S_STATE_STOPPING) {
			if (data->rx.pending != NULL) {
				data->rx.stop_after_active = true;
			} else {
				i2s_silabs_usart_rx_drop_stalled(dev);
			}
		} else if (data->rx.state == I2S_STATE_RUNNING) {
			i2s_silabs_usart_rx_abort_stream(dev);
		}
	}
}

static bool i2s_silabs_usart_cfg_state_allowed(int32_t state);

static int i2s_silabs_usart_dir_power_down(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;

	if (dir == I2S_DIR_TX) {
		if (!i2s_silabs_usart_cfg_state_allowed(data->tx.state)) {
			return -EINVAL;
		}
		if (data->tx.cfg_valid) {
			i2s_silabs_usart_tx_drop(dev);
		}
		data->tx.cfg_valid = false;
		data->tx.state = I2S_STATE_NOT_READY;
	} else {
		if (!i2s_silabs_usart_cfg_state_allowed(data->rx.state)) {
			return -EINVAL;
		}
		if (data->rx.cfg_valid) {
			i2s_silabs_usart_rx_drop(dev);
		}
		data->rx.cfg_valid = false;
		data->rx.state = I2S_STATE_NOT_READY;
		sl_hal_usart_disable_rx(pcfg->base);
	}

	if (!data->tx.cfg_valid && !data->rx.cfg_valid) {
		i2s_silabs_usart_hw_disable_data_irqs(pcfg->base);
		i2s_silabs_usart_hw_disable_error_irqs(pcfg->base);
		sl_hal_usart_disable(pcfg->base);
		i2s_silabs_usart_mclk_set(pcfg->mclk_dev, pcfg->mclk_subsys, false);
	}

	return 0;
}

static int i2s_silabs_usart_validate_configure_block_size(const struct i2s_config *cfg)
{
	const uint32_t frame_bytes =
		((uint32_t)cfg->word_size / BITS_PER_BYTE) * (uint32_t)cfg->channels;

	if (cfg->block_size == 0U || frame_bytes == 0U || (cfg->block_size % frame_bytes) != 0U ||
	    (cfg->block_size % I2S_SILABS_WORD_SIZE_BYTES) != 0U) {
		return -EINVAL;
	}
	if (cfg->block_size > I2S_SILABS_LDMA_MAX_BLOCK_BYTES) {
		return -EINVAL;
	}
	return 0;
}

static bool i2s_silabs_usart_cfg_state_allowed(int32_t state)
{
	return (state == I2S_STATE_NOT_READY) || (state == I2S_STATE_READY);
}

static int i2s_silabs_usart_configure_apply_hw(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	int err;

	err = pinctrl_apply_state(pcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = clock_control_on(pcfg->clock_dev, (clock_control_subsys_t)&pcfg->clock_cfg);
	if (err < 0 && err != -EALREADY) {
		return err;
	}

	i2s_silabs_usart_mclk_set(pcfg->mclk_dev, pcfg->mclk_subsys, true);

	err = i2s_silabs_usart_hw_apply(dev);
	if (err < 0) {
		i2s_silabs_usart_mclk_set(pcfg->mclk_dev, pcfg->mclk_subsys, false);
		return err;
	}

	return 0;
}

static int i2s_silabs_usart_configure(const struct device *dev, enum i2s_dir dir,
				      const struct i2s_config *cfg)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	bool need_hw_apply;
	int err = 0;

	k_mutex_lock(&data->cfg_lock, K_FOREVER);

	if (dir == I2S_DIR_BOTH) {
		err = -ENOSYS;
		goto out;
	}
	if (dir != I2S_DIR_TX && dir != I2S_DIR_RX) {
		err = -EINVAL;
		goto out;
	}
	if (dir == I2S_DIR_RX && !pcfg->has_rx) {
		err = -ENOTSUP;
		goto out;
	}

	if (cfg == NULL || cfg->frame_clk_freq == 0U) {
		err = i2s_silabs_usart_dir_power_down(dev, dir);
		goto out;
	}

	err = i2s_silabs_usart_validate_i2s_cfg(cfg);
	if (err < 0) {
		goto out;
	}

	err = i2s_silabs_usart_validate_configure_block_size(cfg);
	if (err < 0) {
		goto out;
	}

	if (dir == I2S_DIR_TX) {
		if (!i2s_silabs_usart_cfg_state_allowed(data->tx.state) ||
		    !i2s_silabs_usart_cfg_state_allowed(data->rx.state)) {
			/* hw_apply() resets the shared USART. */
			err = -EINVAL;
			goto out;
		}
		if (cfg->channels != 1U && cfg->channels != 2U) {
			err = -EINVAL;
			goto out;
		}
		if (cfg->channels == 1U && !pcfg->has_tx_split) {
			err = -EINVAL;
			goto out;
		}
		if (data->rx.cfg_valid && !i2s_silabs_usart_cfg_bus_match(cfg, &data->rx.cfg)) {
			err = -EINVAL;
			goto out;
		}

		memcpy(&data->tx.cfg, cfg, sizeof(*cfg));
		data->tx.cfg_valid = true;
		need_hw_apply = true;
	} else {
		if (!i2s_silabs_usart_cfg_state_allowed(data->rx.state)) {
			err = -EINVAL;
			goto out;
		}
		if (cfg->channels != 2U) {
			err = -EINVAL;
			goto out;
		}
		if (data->tx.cfg_valid && !i2s_silabs_usart_cfg_bus_match(cfg, &data->tx.cfg)) {
			err = -EINVAL;
			goto out;
		}

		memcpy(&data->rx.cfg, cfg, sizeof(*cfg));
		data->rx.cfg_valid = true;
		/* Peer TX already owns the USART programming; only enable RX. */
		need_hw_apply = !data->tx.cfg_valid;
		if (need_hw_apply && !i2s_silabs_usart_cfg_state_allowed(data->tx.state)) {
			err = -EINVAL;
			goto out;
		}
	}

	if (need_hw_apply) {
		err = i2s_silabs_usart_configure_apply_hw(dev);
		if (err < 0) {
			goto fail_cfg;
		}
	} else {
		sl_hal_usart_enable_rx(pcfg->base);
		i2s_silabs_usart_hw_enable_rxof_irq(pcfg->base);
	}

	if (dir == I2S_DIR_TX) {
		data->tx.state = I2S_STATE_READY;
	} else {
		data->rx.state = I2S_STATE_READY;
	}
	goto out;

fail_cfg:
	if (dir == I2S_DIR_TX) {
		data->tx.cfg_valid = false;
		data->tx.state = I2S_STATE_NOT_READY;
	} else {
		data->rx.cfg_valid = false;
		data->rx.state = I2S_STATE_NOT_READY;
	}

out:
	k_mutex_unlock(&data->cfg_lock);
	return err;
}

static const struct i2s_config *i2s_silabs_usart_config_get(const struct device *dev,
							    enum i2s_dir dir)
{
	struct i2s_silabs_usart_data *data = dev->data;

	if (dir == I2S_DIR_TX && data->tx.cfg_valid) {
		return &data->tx.cfg;
	}
	if (dir == I2S_DIR_RX && data->rx.cfg_valid) {
		return &data->rx.cfg;
	}
	return NULL;
}

/*
 * Start the mono pad-slot (silence) channel as a self-linked, endlessly looping
 * descriptor. It only moves a word when the USART requests the pad slot, so it
 * stays in lockstep with the audio channel without needing per-block appends.
 * (Per-block appends on this channel raced the audio channel's completion IRQ:
 * when its previous link had not yet been consumed the append failed with
 * -EINVAL and the already-accepted audio block was dropped -- audible as
 * distortion on the first tone(s).)
 */
static int i2s_silabs_usart_mono_tx_silence_run(const struct device *dev, size_t len)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_dma *silence = &data->tx_silence;
	struct i2s_silabs_usart_dma_desc *desc = cfg->tx_silence_dma;
	int ret;

	if (silence->busy) {
		return 0;
	}

	desc->blk_cfg.block_size = len;
	desc->blk_cfg.source_address = (uintptr_t)&i2s_silabs_usart_silence_word;
	desc->blk_cfg.next_block = &desc->blk_cfg;

	ret = dma_config(cfg->dma_dev, (uint32_t)silence->channel, &desc->dma_cfg);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(cfg->dma_dev, (uint32_t)silence->channel);
	if (ret < 0) {
		dma_stop(cfg->dma_dev, (uint32_t)silence->channel);
		return ret;
	}

	silence->busy = true;
	return 0;
}

static int i2s_silabs_usart_dma_config_start(const struct device *dev, void *blk)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *tx = &data->tx;
	int ret;

	ret = dma_config(cfg->dma_dev, (uint32_t)tx->dma.channel, &cfg->tx_dma->dma_cfg);
	if (ret < 0) {
		if (i2s_silabs_usart_tx_path_is_mono(dev)) {
			dma_stop(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
			data->tx_silence.busy = false;
		}
		tx->dma.busy = false;
		tx->active = NULL;
		i2s_silabs_usart_tx_release_block(tx, blk);
		tx->state = I2S_STATE_ERROR;
		return ret;
	}

	ret = dma_start(cfg->dma_dev, (uint32_t)tx->dma.channel);
	if (ret < 0) {
		dma_stop(cfg->dma_dev, (uint32_t)tx->dma.channel);
		if (i2s_silabs_usart_tx_path_is_mono(dev)) {
			dma_stop(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
			data->tx_silence.busy = false;
		}
		tx->dma.busy = false;
		tx->active = NULL;
		i2s_silabs_usart_tx_release_block(tx, blk);
		tx->state = I2S_STATE_ERROR;
		return ret;
	}

	return 0;
}

static void i2s_silabs_usart_tx_try_start(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *tx = &data->tx;
	void *blk = NULL;
	size_t len = 0U;
	int ret;
	k_spinlock_key_t key;

	if (tx->state == I2S_STATE_RUNNING) {
		/* continue */
	} else if (tx->state == I2S_STATE_STOPPING && tx->drain) {
		/* I2S_TRIGGER_DRAIN: keep feeding until the queue is empty */
	} else {
		return;
	}

	key = k_spin_lock(&data->lock);

	if (tx->active == NULL) {
		/*
		 * DMA is idle -- dequeue a block and perform a full dma_config() +
		 * dma_start() cycle.  Cold-start path used by I2S_TRIGGER_START and as
		 * fallback when the queue was empty.
		 */
		ret = i2s_silabs_usart_tx_q_get(tx, &blk, &len);
		if (ret < 0) {
			k_spin_unlock(&data->lock, key);
			if (tx->state == I2S_STATE_RUNNING) {
				i2s_silabs_usart_tx_underrun(dev);
			} else {
				i2s_silabs_usart_tx_finish_stopping_if_quiescent(dev);
			}
			return;
		}

		tx->active = blk;
		tx->dma.busy = true;

		cfg->tx_dma->blk_cfg.source_address = (uintptr_t)blk;
		cfg->tx_dma->blk_cfg.block_size = len;

		if (i2s_silabs_usart_tx_path_is_mono(dev)) {
			ret = i2s_silabs_usart_mono_tx_silence_run(dev, len);
			if (ret < 0) {
				tx->dma.busy = false;
				tx->active = NULL;
				i2s_silabs_usart_tx_release_block(tx, blk);
				tx->state = I2S_STATE_ERROR;
				k_spin_unlock(&data->lock, key);
				return;
			}
		}

		ret = i2s_silabs_usart_dma_config_start(dev, blk);
		k_spin_unlock(&data->lock, key);
		if (ret < 0) {
			return;
		}

		/* Immediately try to double-buffer for gapless first transition */
		i2s_silabs_usart_tx_try_start(dev);
		return;
	}

	if (tx->pending != NULL) {
		k_spin_unlock(&data->lock, key);
		return;
	}

	/*
	 * DMA is running and no next block queued yet.  Pop the next block and
	 * use silabs_ldma_append_block() to chain it after the current transfer.
	 * The LDMA hardware will seamlessly transition via LINKLOAD, eliminating
	 * the gap that dma_stop/config/start would introduce.
	 */
	ret = i2s_silabs_usart_tx_q_get(tx, &blk, &len);
	if (ret < 0) {
		k_spin_unlock(&data->lock, key);
		return;
	}

	cfg->tx_dma->blk_cfg.source_address = (uintptr_t)blk;
	cfg->tx_dma->blk_cfg.block_size = len;

	ret = silabs_ldma_append_block(cfg->dma_dev, (uint32_t)tx->dma.channel,
				       &cfg->tx_dma->dma_cfg);
	if (ret == 0) {
		tx->pending = blk;
		k_spin_unlock(&data->lock, key);
	} else {
		/*
		 * The block was accepted by i2s_write(), so it must not be discarded.
		 * Put it back at the head of the queue; it is retried on the next DMA
		 * completion (via append, or a cold start if the channel went idle).
		 * After a successful pop under the spinlock the queue has room; if
		 * unpop still fails, release the block so the slot is not leaked.
		 */
		ret = i2s_silabs_usart_tx_q_unpop(tx, blk, len);
		k_spin_unlock(&data->lock, key);
		if (ret < 0) {
			i2s_silabs_usart_tx_release_block(tx, blk);
		}
	}
}

static void i2s_silabs_usart_tx_abort_stream(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *tx = &data->tx;

	tx->state = I2S_STATE_ERROR;
	dma_stop(cfg->dma_dev, (uint32_t)tx->dma.channel);
	tx->dma.busy = false;
	if (i2s_silabs_usart_tx_path_is_mono(dev) && data->tx_silence.busy) {
		dma_stop(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
		data->tx_silence.busy = false;
	}
	if (tx->pending != NULL) {
		i2s_silabs_usart_tx_release_block(tx, tx->pending);
		tx->pending = NULL;
	}
	if (tx->active != NULL) {
		i2s_silabs_usart_tx_release_block(tx, tx->active);
		tx->active = NULL;
	}
}

static void i2s_silabs_usart_dma_tx_cb(const struct device *dma_dev, void *user_data,
				       uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *tx = &data->tx;
	void *done;
	k_spinlock_key_t key;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		i2s_silabs_usart_tx_abort_stream(dev);
		return;
	}

	key = k_spin_lock(&data->lock);
	done = tx->active;

	if (tx->pending != NULL) {
		/*
		 * silabs_ldma_append_block() chained the pending block: the
		 * LDMA has already auto-started it via LINKLOAD before this
		 * callback fires.  Do NOT call dma_stop() -- the channel is
		 * still running.
		 */
		tx->active = tx->pending;
		tx->pending = NULL;
	} else {
		/*
		 * No block was chained -- DMA is now idle.  Call dma_stop()
		 * to clear the LDMA driver's internal "busy" atomic so the
		 * next dma_config() does not fail with -EBUSY.
		 *
		 * If the queue is empty while still RUNNING, treat as TX
		 * underrun (I2S_STATE_ERROR).
		 */
		dma_stop(cfg->dma_dev, (uint32_t)tx->dma.channel);
		tx->active = NULL;
		tx->dma.busy = false;
		if (i2s_silabs_usart_tx_path_is_mono(dev) && data->tx_silence.busy) {
			dma_stop(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
			data->tx_silence.busy = false;
		}
	}
	k_spin_unlock(&data->lock, key);

	if (done != NULL) {
		i2s_silabs_usart_tx_release_block(tx, done);
	}

	/*
	 * STOPPING without drain (I2S_TRIGGER_STOP): do not pull more blocks.
	 * DRAIN / RUNNING continue via i2s_silabs_usart_tx_try_start.
	 */
	if (tx->state == I2S_STATE_RUNNING || (tx->state == I2S_STATE_STOPPING && tx->drain)) {
		i2s_silabs_usart_tx_try_start(dev);
	}
	i2s_silabs_usart_tx_finish_stopping_if_quiescent(dev);
}

static int i2s_silabs_usart_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_silabs_usart_data *data = dev->data;
	struct i2s_silabs_usart_stream *rx = &data->rx;
	void *blk;
	size_t len;
	int ret;

	if (!rx->cfg_valid) {
		return -EIO;
	}

	if (rx->state == I2S_STATE_ERROR && k_msgq_num_used_get(&rx->q) == 0U) {
		return -EIO;
	}

	ret = k_sem_take(&rx->sem, (rx->state == I2S_STATE_ERROR)
					   ? K_NO_WAIT
					   : i2s_silabs_usart_cfg_timeout(rx->cfg.timeout));
	if (ret < 0) {
		return (rx->state == I2S_STATE_ERROR) ? -EIO : ret;
	}

	ret = i2s_silabs_usart_block_q_get(&rx->q, &blk, &len);
	if (ret < 0) {
		return -EIO;
	}

	*mem_block = blk;
	*size = len;
	return 0;
}

static int i2s_silabs_usart_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_silabs_usart_data *data = dev->data;
	uint32_t tx_frame_bytes;
	int ret;
	k_spinlock_key_t key;

	if (!data->tx.cfg_valid) {
		return -EIO;
	}
	if (data->tx.state != I2S_STATE_READY && data->tx.state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	tx_frame_bytes = ((uint32_t)data->tx.cfg.word_size / BITS_PER_BYTE) *
			 (uint32_t)data->tx.cfg.channels;

	if (size == 0U || size > data->tx.cfg.block_size || tx_frame_bytes == 0U ||
	    (size % tx_frame_bytes) != 0U || (size % I2S_SILABS_WORD_SIZE_BYTES) != 0U) {
		return -EINVAL;
	}

	ret = k_sem_take(&data->tx.sem, i2s_silabs_usart_cfg_timeout(data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	key = k_spin_lock(&data->lock);

	ret = i2s_silabs_usart_block_q_put(&data->tx.q, mem_block, size);
	k_spin_unlock(&data->lock, key);

	if (ret < 0) {
		k_sem_give(&data->tx.sem);
		return -ENOMEM;
	}

	if (data->tx.state == I2S_STATE_RUNNING) {
		i2s_silabs_usart_tx_try_start(dev);
	}

	return 0;
}

static int i2s_silabs_usart_trigger_dir_check(enum i2s_dir dir)
{
	if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	}
	if (dir != I2S_DIR_TX && dir != I2S_DIR_RX) {
		return -EINVAL;
	}
	return 0;
}

static int i2s_silabs_usart_trigger_start(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	int err;

	err = i2s_silabs_usart_trigger_dir_check(dir);
	if (err < 0) {
		return err;
	}

	if (dir == I2S_DIR_TX) {
		if (!data->tx.cfg_valid || data->tx.state != I2S_STATE_READY) {
			return -EIO;
		}
		if (i2s_silabs_usart_tx_q_empty(&data->tx)) {
			return -EIO;
		}

		data->tx.drain = false;
		data->tx.state = I2S_STATE_RUNNING;
		i2s_silabs_usart_tx_try_start(dev);

		if (data->tx.state == I2S_STATE_ERROR) {
			return -EIO;
		}
		if (data->tx.active == NULL && !data->tx.dma.busy) {
			data->tx.state = I2S_STATE_READY;
			return -EIO;
		}
		return 0;
	}

	if (!pcfg->has_rx || !data->rx.cfg_valid || data->rx.state != I2S_STATE_READY) {
		return -EIO;
	}

	/*
	 * Master BCLK runs only while TX shifts data. Arm RX DMA before TX
	 * START so the first frames are captured when clocks appear.
	 */
	sl_hal_usart_clear_rx(pcfg->base);
	sl_hal_usart_clear_interrupts(pcfg->base, USART_IF_RXOF | USART_IF_RXDATAV |
							  USART_IF_RXFULL);
	sl_hal_usart_enable_rx(pcfg->base);
	i2s_silabs_usart_hw_enable_rxof_irq(pcfg->base);

	data->rx.drain = false;
	data->rx.state = I2S_STATE_RUNNING;
	i2s_silabs_usart_rx_try_start(dev);

	if (data->rx.state == I2S_STATE_ERROR) {
		return -EIO;
	}
	if (data->rx.active == NULL && !data->rx.dma.busy) {
		data->rx.state = I2S_STATE_READY;
		return -EIO;
	}

	return 0;
}

static int i2s_silabs_usart_trigger_stop(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_silabs_usart_data *data = dev->data;
	int err;

	err = i2s_silabs_usart_trigger_dir_check(dir);
	if (err < 0) {
		return err;
	}

	if (dir == I2S_DIR_TX) {
		if (!data->tx.cfg_valid || data->tx.state != I2S_STATE_RUNNING) {
			return -EIO;
		}

		data->tx.drain = false;
		if (data->tx.active == NULL && data->tx.pending == NULL) {
			data->tx.state = I2S_STATE_READY;
		} else {
			data->tx.state = I2S_STATE_STOPPING;
			i2s_silabs_usart_tx_finish_stopping_if_quiescent(dev);
		}
		return 0;
	}

	if (!data->rx.cfg_valid || data->rx.state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	data->rx.drain = false;
	if (data->rx.active == NULL && data->rx.pending == NULL) {
		data->rx.state = I2S_STATE_READY;
	} else {
		data->rx.state = I2S_STATE_STOPPING;
		i2s_silabs_usart_rx_finish_stopping_if_quiescent(dev);
	}
	return 0;
}

static int i2s_silabs_usart_trigger_drain(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_silabs_usart_data *data = dev->data;
	int err;

	err = i2s_silabs_usart_trigger_dir_check(dir);
	if (err < 0) {
		return err;
	}

	if (dir == I2S_DIR_TX) {
		if (!data->tx.cfg_valid || data->tx.state != I2S_STATE_RUNNING) {
			return -EIO;
		}

		data->tx.drain = true;
		data->tx.state = I2S_STATE_STOPPING;
		i2s_silabs_usart_tx_try_start(dev);
		i2s_silabs_usart_tx_finish_stopping_if_quiescent(dev);
		return 0;
	}

	/* RX DRAIN: keep receiving until STOPPING settles (no TX queue). */
	if (!data->rx.cfg_valid || data->rx.state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	data->rx.drain = false;
	data->rx.state = I2S_STATE_STOPPING;
	i2s_silabs_usart_rx_finish_stopping_if_quiescent(dev);
	return 0;
}

static int i2s_silabs_usart_trigger_drop(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_silabs_usart_cfg *pcfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	int err;

	err = i2s_silabs_usart_trigger_dir_check(dir);
	if (err < 0) {
		return err;
	}

	if (dir == I2S_DIR_TX) {
		if (!data->tx.cfg_valid) {
			return -EIO;
		}
		i2s_silabs_usart_hw_disable_data_irqs(pcfg->base);
		i2s_silabs_usart_tx_drop(dev);
		data->tx.state = I2S_STATE_READY;
		return 0;
	}

	if (!data->rx.cfg_valid) {
		return -EIO;
	}

	i2s_silabs_usart_rx_drop(dev);
	data->rx.state = I2S_STATE_READY;
	return 0;
}

static int i2s_silabs_usart_trigger_prepare(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_silabs_usart_data *data = dev->data;
	int err;

	err = i2s_silabs_usart_trigger_dir_check(dir);
	if (err < 0) {
		return err;
	}

	if (dir == I2S_DIR_TX) {
		if (!data->tx.cfg_valid || data->tx.state != I2S_STATE_ERROR) {
			return -EIO;
		}
		i2s_silabs_usart_tx_drop(dev);
		data->tx.state = I2S_STATE_READY;
		return 0;
	}

	if (!data->rx.cfg_valid || data->rx.state != I2S_STATE_ERROR) {
		return -EIO;
	}

	i2s_silabs_usart_rx_drop(dev);
	data->rx.state = I2S_STATE_READY;
	return 0;
}

static int i2s_silabs_usart_trigger(const struct device *dev, enum i2s_dir dir,
				    enum i2s_trigger_cmd cmd)
{
	struct i2s_silabs_usart_data *data = dev->data;
	int err = 0;

	k_mutex_lock(&data->cfg_lock, K_FOREVER);

	switch (cmd) {
	case I2S_TRIGGER_START:
		err = i2s_silabs_usart_trigger_start(dev, dir);
		break;
	case I2S_TRIGGER_STOP:
		err = i2s_silabs_usart_trigger_stop(dev, dir);
		break;
	case I2S_TRIGGER_DRAIN:
		err = i2s_silabs_usart_trigger_drain(dev, dir);
		break;
	case I2S_TRIGGER_DROP:
		err = i2s_silabs_usart_trigger_drop(dev, dir);
		break;
	case I2S_TRIGGER_PREPARE:
		err = i2s_silabs_usart_trigger_prepare(dev, dir);
		break;
	default:
		err = -EINVAL;
		break;
	}

	k_mutex_unlock(&data->cfg_lock);
	return err;
}

static int i2s_silabs_usart_init(const struct device *dev)
{
	const struct i2s_silabs_usart_cfg *cfg = dev->config;
	struct i2s_silabs_usart_data *data = dev->data;
	USART_TypeDef *base = cfg->base;
	int err;

	k_mutex_init(&data->cfg_lock);
	k_msgq_init(&data->tx.q, data->tx.q_buf, sizeof(struct i2s_silabs_usart_block),
		    CONFIG_I2S_SILABS_USART_TX_BLOCK_COUNT);
	k_sem_init(&data->tx.sem, CONFIG_I2S_SILABS_USART_TX_BLOCK_COUNT,
		   CONFIG_I2S_SILABS_USART_TX_BLOCK_COUNT);
	k_msgq_init(&data->rx.q, data->rx.q_buf, sizeof(struct i2s_silabs_usart_block),
		    CONFIG_I2S_SILABS_USART_RX_BLOCK_COUNT);
	k_sem_init(&data->rx.sem, 0, CONFIG_I2S_SILABS_USART_RX_BLOCK_COUNT);

	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	/*
	 * USART register access faults if the peripheral clock is
	 * still gated (see uart_silabs_init).
	 */
	err = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)&cfg->clock_cfg);
	if (err < 0 && err != -EALREADY) {
		return err;
	}

	if (cfg->has_tx_split) {
		data->tx_silence.channel = dma_request_channel(cfg->dma_dev, NULL);
		if (data->tx_silence.channel < 0) {
			return -ENODEV;
		}

		data->tx.dma.channel = dma_request_channel(cfg->dma_dev, NULL);
		if (data->tx.dma.channel < 0) {
			dma_release_channel(cfg->dma_dev, (uint32_t)data->tx_silence.channel);
			return -ENODEV;
		}
	} else {
		data->tx_silence.channel = -1;
		data->tx.dma.channel = dma_request_channel(cfg->dma_dev, NULL);
		if (data->tx.dma.channel < 0) {
			return -ENODEV;
		}
	}

	/*
	 * Provisional DMA defaults for 16-bit words. i2s_silabs_usart_apply_dma_xfer_size()
	 * keeps these consistent when configure() runs. Descriptors are instance-static
	 * (see cfg->tx_dma / tx_silence_dma), not driver data.
	 */
	memset(&cfg->tx_dma->dma_cfg, 0, sizeof(cfg->tx_dma->dma_cfg));
	cfg->tx_dma->dma_cfg.dma_slot = cfg->dma_txbl_slot;
	cfg->tx_dma->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	cfg->tx_dma->dma_cfg.source_data_size = I2S_SILABS_WORD_SIZE_BYTES;
	cfg->tx_dma->dma_cfg.dest_data_size = I2S_SILABS_WORD_SIZE_BYTES;
	cfg->tx_dma->dma_cfg.source_burst_length = I2S_SILABS_WORD_SIZE_BYTES;
	cfg->tx_dma->dma_cfg.dest_burst_length = I2S_SILABS_WORD_SIZE_BYTES;
	cfg->tx_dma->dma_cfg.head_block = &cfg->tx_dma->blk_cfg;
	cfg->tx_dma->dma_cfg.user_data = (void *)dev;
	cfg->tx_dma->dma_cfg.dma_callback = i2s_silabs_usart_dma_tx_cb;
	cfg->tx_dma->dma_cfg.complete_callback_en = I2S_DMA_COMPLETE_CB_ENABLED;
	cfg->tx_dma->dma_cfg.channel_priority = I2S_DMA_CHANNEL_PRIORITY;
	memset(&cfg->tx_dma->blk_cfg, 0, sizeof(cfg->tx_dma->blk_cfg));
	/* TX DMA writes to TXDOUBLE: a 16-bit access enqueues one I2S slot. */
	cfg->tx_dma->blk_cfg.dest_address = (uintptr_t)&base->TXDOUBLE;
	cfg->tx_dma->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	cfg->tx_dma->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	if (cfg->has_tx_split) {
		memset(&cfg->tx_silence_dma->dma_cfg, 0, sizeof(cfg->tx_silence_dma->dma_cfg));
		cfg->tx_silence_dma->dma_cfg.dma_slot = cfg->dma_txbl_slot;
		cfg->tx_silence_dma->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		cfg->tx_silence_dma->dma_cfg.source_data_size = I2S_SILABS_WORD_SIZE_BYTES;
		cfg->tx_silence_dma->dma_cfg.dest_data_size = I2S_SILABS_WORD_SIZE_BYTES;
		cfg->tx_silence_dma->dma_cfg.source_burst_length = I2S_SILABS_WORD_SIZE_BYTES;
		cfg->tx_silence_dma->dma_cfg.dest_burst_length = I2S_SILABS_WORD_SIZE_BYTES;
		cfg->tx_silence_dma->dma_cfg.head_block = &cfg->tx_silence_dma->blk_cfg;
		cfg->tx_silence_dma->dma_cfg.channel_priority = I2S_DMA_CHANNEL_PRIORITY;
		cfg->tx_silence_dma->dma_cfg.complete_callback_en = 0U;
		memset(&cfg->tx_silence_dma->blk_cfg, 0, sizeof(cfg->tx_silence_dma->blk_cfg));
		cfg->tx_silence_dma->blk_cfg.dest_address = (uintptr_t)&base->TXDOUBLE;
		cfg->tx_silence_dma->blk_cfg.source_address =
			(uintptr_t)&i2s_silabs_usart_silence_word;
		cfg->tx_silence_dma->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		cfg->tx_silence_dma->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	data->rx.dma.channel = -1;
	if (cfg->has_rx) {
		data->rx.dma.channel = dma_request_channel(cfg->dma_dev, NULL);
		if (data->rx.dma.channel < 0) {
			return -ENODEV;
		}

		memset(&cfg->rx_dma->dma_cfg, 0, sizeof(cfg->rx_dma->dma_cfg));
		cfg->rx_dma->dma_cfg.dma_slot = cfg->dma_rxdatav_slot;
		cfg->rx_dma->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		i2s_silabs_usart_apply_dma_xfer_size(&cfg->rx_dma->dma_cfg);
		cfg->rx_dma->dma_cfg.head_block = &cfg->rx_dma->blk_cfg;
		cfg->rx_dma->dma_cfg.user_data = (void *)dev;
		cfg->rx_dma->dma_cfg.dma_callback = i2s_silabs_usart_dma_rx_cb;
		cfg->rx_dma->dma_cfg.complete_callback_en = I2S_DMA_COMPLETE_CB_ENABLED;
		cfg->rx_dma->dma_cfg.channel_priority = I2S_DMA_CHANNEL_PRIORITY;
		memset(&cfg->rx_dma->blk_cfg, 0, sizeof(cfg->rx_dma->blk_cfg));
		cfg->rx_dma->blk_cfg.source_address = (uintptr_t)&base->RXDOUBLE;
		cfg->rx_dma->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		cfg->rx_dma->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	i2s_silabs_usart_hw_disable_data_irqs(base);
	i2s_silabs_usart_hw_disable_error_irqs(base);

	return 0;
}

static DEVICE_API(i2s, i2s_silabs_usart_driver_api) = {
	.configure = i2s_silabs_usart_configure,
	.config_get = i2s_silabs_usart_config_get,
	.read = i2s_silabs_usart_read,
	.write = i2s_silabs_usart_write,
	.trigger = i2s_silabs_usart_trigger,
};

/*
 * Optional codec MCLK from silabs,mclk-out → clock-output@N under
 * silabs,series-clock-output (parent device + child reg as subsystem).
 */
#define I2S_SILABS_USART_MCLK_INIT(idx)                                                            \
	COND_CODE_1(                                                                               \
		DT_INST_NODE_HAS_PROP(idx, silabs_mclk_out),                                       \
		(.mclk_dev = DEVICE_DT_GET(                                                        \
			 DT_PARENT(DT_INST_PHANDLE(idx, silabs_mclk_out))),                        \
		 .mclk_subsys = (clock_control_subsys_t)(uintptr_t)DT_REG_ADDR(                    \
			 DT_INST_PHANDLE(idx, silabs_mclk_out)),),                                 \
		(.mclk_dev = NULL, .mclk_subsys = NULL,))

#define I2S_SILABS_USART_DMA_STORAGE(idx)                                                          \
	static struct i2s_silabs_usart_dma_desc i2s_silabs_usart_tx_dma_##idx;                     \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, txblright),                                         \
		    (static struct i2s_silabs_usart_dma_desc                                       \
			     i2s_silabs_usart_tx_silence_dma_##idx;),                              \
		    ())                                                                            \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, rxdatav),                                           \
		    (static struct i2s_silabs_usart_dma_desc i2s_silabs_usart_rx_dma_##idx;),      \
		    ())

#define I2S_SILABS_USART_DMA_INIT(idx)                                                             \
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, txbl)),                            \
	.dma_txbl_slot = SILABS_LDMA_REQSEL_TO_SLOT(DT_INST_DMAS_CELL_BY_NAME(idx, txbl, slot)),   \
	.dma_txblright_slot = COND_CODE_1(                                                         \
		DT_INST_DMAS_HAS_NAME(idx, txblright),                                             \
		(SILABS_LDMA_REQSEL_TO_SLOT(DT_INST_DMAS_CELL_BY_NAME(idx, txblright, slot))),     \
		(0U)),                                                                             \
	.dma_rxdatav_slot = COND_CODE_1(                                                           \
		DT_INST_DMAS_HAS_NAME(idx, rxdatav),                                               \
		(SILABS_LDMA_REQSEL_TO_SLOT(DT_INST_DMAS_CELL_BY_NAME(idx, rxdatav, slot))),       \
		(0U)),                                                                             \
	.tx_dma = &i2s_silabs_usart_tx_dma_##idx,                                                  \
	.tx_silence_dma = COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, txblright),                       \
				      (&i2s_silabs_usart_tx_silence_dma_##idx),                    \
				      (NULL)),                                                     \
	.rx_dma = COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, rxdatav),                                 \
			       (&i2s_silabs_usart_rx_dma_##idx), (NULL)),                          \
	.has_tx_split = DT_INST_DMAS_HAS_NAME(idx, txblright),                                     \
	.has_rx = DT_INST_DMAS_HAS_NAME(idx, rxdatav),

#define I2S_SILABS_USART_MONO_TX_INIT(idx)                                                         \
	.mono_tx_default =                                                                         \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, silabs_mono_tx_slot),                       \
				       (DT_INST_ENUM_IDX(idx, silabs_mono_tx_slot)),               \
				       (I2S_SILABS_USART_MONO_TX_SLOT_RIGHT)),

#define I2S_SILABS_USART_IRQ_CONNECT(idx)                                                          \
	static void i2s_silabs_usart_irq_connect_##idx(void)                                       \
	{                                                                                          \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, rxdatav),                                   \
			    (IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, rx, irq),                        \
					 DT_INST_IRQ_BY_NAME(idx, rx, priority),                   \
					 i2s_silabs_usart_isr, DEVICE_DT_INST_GET(idx), 0);        \
			     irq_enable(DT_INST_IRQ_BY_NAME(idx, rx, irq));),                      \
			    ())                                                                    \
	}

#define I2S_SILABS_USART_DEFINE(idx)                                                               \
	I2S_SILABS_USART_DMA_STORAGE(idx);                                                         \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	I2S_SILABS_USART_IRQ_CONNECT(idx);                                                         \
	static const struct i2s_silabs_usart_cfg i2s_silabs_usart_cfg_##idx = {                    \
		.base = (USART_TypeDef *)DT_INST_REG_ADDR(idx),                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),                              \
		.clock_cfg = SILABS_DT_INST_CLOCK_CFG(idx),                                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		I2S_SILABS_USART_DMA_INIT(idx) I2S_SILABS_USART_MCLK_INIT(idx)                     \
			I2S_SILABS_USART_MONO_TX_INIT(idx)};                                       \
	static struct i2s_silabs_usart_data i2s_silabs_usart_data_##idx;                           \
	static int i2s_silabs_usart_init_##idx(const struct device *dev)                           \
	{                                                                                          \
		int err;                                                                           \
                                                                                                   \
		err = i2s_silabs_usart_init(dev);                                                  \
		if (err == 0) {                                                                    \
			i2s_silabs_usart_irq_connect_##idx();                                      \
		}                                                                                  \
		return err;                                                                        \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(idx, i2s_silabs_usart_init_##idx, NULL,                              \
			      &i2s_silabs_usart_data_##idx, &i2s_silabs_usart_cfg_##idx,           \
			      POST_KERNEL, CONFIG_I2S_INIT_PRIORITY, &i2s_silabs_usart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_SILABS_USART_DEFINE)
