/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Verify I2S TX by looping PC0 (TX SD) to PC1 (RX SD). The bus slot width comes
 * from silabs,slot-width (16: W16D16, 32: W32D16; select with -DSLOT_WIDTH=32).
 * i2s_config.word_size is 16 in both cases and memory holds one 16-bit word per
 * slot (W32D16 RX returns the 16 MSBs of each slot, TX pads with zeros).
 * Cases: stereo TX, then mono TX (silabs,mono-tx-slot). RX is always stereo.
 * Left/right slot carries a per-block PCM staircase on the active TX path;
 * the other slot is zero. RX checks the bus pattern (200 blocks each case).
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/devicetree.h>

#ifndef FRAME_CLK_HZ
#define FRAME_CLK_HZ 16000
#endif
/* i2s_config.word_size accepted by the driver (the bus slot width is separate). */
#define WORD_SIZE 16

/* Bit clocks per slot on the bus, from the devicetree (default 16). */
#if DT_NODE_HAS_PROP(DT_ALIAS(i2s_tx), silabs_slot_width)
#define SLOT_WIDTH DT_PROP(DT_ALIAS(i2s_tx), silabs_slot_width)
#else
#define SLOT_WIDTH 16
#endif

#define NUM_BLOCKS     8
#define PREFILL_BLOCKS 6
#define VERIFY_ROUNDS  200U

#define SAMPLES_PER_FRAME 32

#define SLOT_BYTES 2U
#define SAMPLE_T   int16_t

#define STEREO_FRAME_BYTES (2U * SLOT_BYTES)
#define MONO_FRAME_BYTES   (1U * SLOT_BYTES)
#define STEREO_BLOCK_SIZE  (SAMPLES_PER_FRAME * STEREO_FRAME_BYTES)
#define MONO_BLOCK_SIZE    (SAMPLES_PER_FRAME * MONO_FRAME_BYTES)
#define SLAB_BLOCK_SIZE    STEREO_BLOCK_SIZE

#define RX_OFFSET_MAX 4

/* silabs,mono-tx-slot enum indexes (binding order: left, right). */
#define MONO_TX_SLOT_LEFT  0
#define MONO_TX_SLOT_RIGHT 1

#if DT_NODE_HAS_PROP(DT_ALIAS(i2s_tx), silabs_mono_tx_slot)
#define MONO_TX_SLOT DT_ENUM_IDX(DT_ALIAS(i2s_tx), silabs_mono_tx_slot)
#else
#define MONO_TX_SLOT MONO_TX_SLOT_RIGHT
#endif

static uint32_t block_seq;
static uint32_t rx_expect_id;

static char __aligned(WB_UP(32)) _tx_slab_mem[(NUM_BLOCKS) * WB_UP(SLAB_BLOCK_SIZE)];
static char __aligned(WB_UP(32)) _rx_slab_mem[(NUM_BLOCKS) * WB_UP(SLAB_BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_slab, _tx_slab_mem, WB_UP(SLAB_BLOCK_SIZE), NUM_BLOCKS);

static STRUCT_SECTION_ITERABLE(k_mem_slab, rx_slab) =
	Z_MEM_SLAB_INITIALIZER(rx_slab, _rx_slab_mem, WB_UP(SLAB_BLOCK_SIZE), NUM_BLOCKS);

static int16_t level_for_id(uint32_t id)
{
	return (int16_t)(1000 + (id % 30U) * 1000);
}

static void store_sample(SAMPLE_T *s, uint32_t idx, int16_t sample)
{
	s[idx] = sample;
}

static int16_t load_sample(const SAMPLE_T *s, uint32_t idx)
{
	return s[idx];
}

static int16_t left_sample_at(const SAMPLE_T *s, uint32_t frame_idx)
{
	return load_sample(s, 2U * frame_idx);
}

static int16_t right_sample_at(const SAMPLE_T *s, uint32_t frame_idx)
{
	return load_sample(s, 2U * frame_idx + 1U);
}

static void fill_block_stereo(void *tx_block, uint32_t id)
{
	SAMPLE_T *s = tx_block;
	int16_t level = level_for_id(id);

	for (uint32_t i = 0; i < SAMPLES_PER_FRAME; i++) {
		store_sample(s, 2U * i, level);
		store_sample(s, 2U * i + 1U, 0);
	}
}

static void fill_block_mono(void *tx_block, uint32_t id)
{
	SAMPLE_T *s = tx_block;
	int16_t level = level_for_id(id);

	for (uint32_t i = 0; i < SAMPLES_PER_FRAME; i++) {
		store_sample(s, i, level);
	}
}

static int write_one(const struct device *dev, uint32_t id, uint8_t tx_channels)
{
	void *mem;
	size_t block_size = (tx_channels == 1U) ? MONO_BLOCK_SIZE : STEREO_BLOCK_SIZE;
	int ret;

	ret = k_mem_slab_alloc(&tx_slab, &mem, K_FOREVER);
	if (ret < 0) {
		return ret;
	}
	if (tx_channels == 1U) {
		fill_block_mono(mem, id);
	} else {
		fill_block_stereo(mem, id);
	}
	ret = i2s_write(dev, mem, block_size);
	if (ret < 0) {
		k_mem_slab_free(&tx_slab, mem);
	}
	return ret;
}

/*
 * Active slot carries the staircase; the other is silence.
 * Stereo TX: left active. Mono TX: silabs,mono-tx-slot.
 */
static int16_t expect_left(uint32_t id, uint8_t tx_channels)
{
	if (tx_channels == 1U) {
		return (MONO_TX_SLOT == MONO_TX_SLOT_LEFT) ? level_for_id(id) : 0;
	}
	return level_for_id(id);
}

static int16_t expect_right(uint32_t id, uint8_t tx_channels)
{
	if (tx_channels == 1U) {
		return (MONO_TX_SLOT == MONO_TX_SLOT_RIGHT) ? level_for_id(id) : 0;
	}
	return 0;
}

static int16_t active_sample_at(const SAMPLE_T *s, uint32_t frame_idx, uint8_t tx_channels)
{
	if (tx_channels == 1U && MONO_TX_SLOT == MONO_TX_SLOT_RIGHT) {
		return right_sample_at(s, frame_idx);
	}
	return left_sample_at(s, frame_idx);
}

static int find_rx_offset(const SAMPLE_T *s, int16_t expect_active, uint8_t tx_channels)
{
	for (int off = 0; off <= RX_OFFSET_MAX; off++) {
		if ((uint32_t)(off + 4) > SAMPLES_PER_FRAME) {
			break;
		}
		if (active_sample_at(s, (uint32_t)off, tx_channels) == expect_active) {
			return off;
		}
	}
	return -1;
}

static int verify_rx_block(void *mem, size_t len, uint32_t id, uint8_t tx_channels)
{
	const SAMPLE_T *s = mem;
	int16_t want_l = expect_left(id, tx_channels);
	int16_t want_r = expect_right(id, tx_channels);
	int16_t want_active = (tx_channels == 1U && MONO_TX_SLOT == MONO_TX_SLOT_RIGHT) ?
			      want_r :
			      want_l;
	int off;
	int bad = 0;

	if (len != STEREO_BLOCK_SIZE) {
		printf("rx len %u != %u\n", (unsigned int)len, STEREO_BLOCK_SIZE);
		return -EINVAL;
	}

	off = find_rx_offset(s, want_active, tx_channels);
	if (off < 0) {
		printf("rx id %u: no offset for active=%d (first L=%d R=%d)\n",
		       id, want_active,
		       (int)left_sample_at(s, 0),
		       (int)right_sample_at(s, 0));
		return -EIO;
	}

	for (int i = off; i < (int)SAMPLES_PER_FRAME; i++) {
		if (left_sample_at(s, (uint32_t)i) != want_l) {
			bad++;
		}
		if (right_sample_at(s, (uint32_t)i) != want_r) {
			bad++;
		}
	}

	if (bad > 0) {
		printf("rx id %u off %d: %d mismatches (expect L=%d R=%d)\n",
		       id, off, bad, want_l, want_r);
		return -EIO;
	}

	return 0;
}

static int read_one(const struct device *dev, uint32_t id, uint8_t tx_channels)
{
	void *mem;
	size_t len;
	int ret;

	ret = i2s_read(dev, &mem, &len);
	if (ret < 0) {
		printf("read id %u failed %d\n", id, ret);
		return ret;
	}

	ret = verify_rx_block(mem, len, id, tx_channels);
	k_mem_slab_free(&rx_slab, mem);
	return ret;
}

static void i2s_bus_cfg(struct i2s_config *cfg, uint8_t channels, size_t block_size)
{
	cfg->word_size = WORD_SIZE;
	cfg->channels = channels;
	cfg->format = I2S_FMT_DATA_FORMAT_I2S;
	cfg->frame_clk_freq = FRAME_CLK_HZ;
	cfg->block_size = block_size;
	cfg->timeout = 2000;
	cfg->options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER |
		       I2S_OPT_BIT_CLK_GATED;
}

static int i2s_power_down(const struct device *dev)
{
	struct i2s_config cfg = {0};
	int ret;

	ret = i2s_configure(dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		printf("TX power-down failed %d\n", ret);
		return ret;
	}
	ret = i2s_configure(dev, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		printf("RX power-down failed %d\n", ret);
	}
	return ret;
}

static int run_case(const struct device *dev, uint8_t tx_channels, const char *name)
{
	struct i2s_config cfg;
	size_t tx_block_size = (tx_channels == 1U) ? MONO_BLOCK_SIZE : STEREO_BLOCK_SIZE;
	int ret;
	uint32_t n;
	uint32_t ok;
	uint32_t round;

	ret = i2s_power_down(dev);
	if (ret < 0) {
		return ret;
	}

	i2s_bus_cfg(&cfg, tx_channels, tx_block_size);
	cfg.mem_slab = &tx_slab;
	ret = i2s_configure(dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		printf("[%s] configure TX failed %d\n", name, ret);
		return ret;
	}

	i2s_bus_cfg(&cfg, 2U, STEREO_BLOCK_SIZE);
	cfg.mem_slab = &rx_slab;
	ret = i2s_configure(dev, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		printf("[%s] configure RX failed %d\n", name, ret);
		return ret;
	}

	block_seq = 0U;
	rx_expect_id = 0U;

	printf("--- %s (TX channels=%u, RX stereo) ---\n", name, tx_channels);
	printf("word_size=%u slot_width=%u (W%uD16) LRCLK=%u Hz, %u frames/block, TX block=%u B\n",
	       (unsigned int)WORD_SIZE, (unsigned int)SLOT_WIDTH, (unsigned int)SLOT_WIDTH,
	       FRAME_CLK_HZ, SAMPLES_PER_FRAME, (unsigned int)tx_block_size);
	if (tx_channels == 1U) {
		printf("mono slot=%s (audio), other=0\n",
		       (MONO_TX_SLOT == MONO_TX_SLOT_LEFT) ? "left" : "right");
	} else {
		printf("stereo: L=staircase R=0\n");
	}
	printf("Prefill %u TX blocks, START RX then TX\n", PREFILL_BLOCKS);

	for (n = 0; n < PREFILL_BLOCKS; n++) {
		ret = write_one(dev, block_seq++, tx_channels);
		if (ret < 0) {
			printf("[%s] prefill write %u failed %d\n", name, n, ret);
			return ret;
		}
	}

	ret = i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("[%s] RX START failed %d\n", name, ret);
		return ret;
	}

	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("[%s] TX START failed %d\n", name, ret);
		return ret;
	}

	ok = 0U;
	for (round = 0; round < VERIFY_ROUNDS; round++) {
		ret = read_one(dev, rx_expect_id, tx_channels);
		if (ret == 0) {
			ok++;
		}

		ret = write_one(dev, block_seq++, tx_channels);
		if (ret < 0) {
			printf("[%s] write failed at round %u: %d\n", name, round, ret);
			break;
		}

		if ((rx_expect_id % 16U) == 0U) {
			printf("[%s] rx id %u ok_total %u / expect L=%d R=%d\n",
			       name, rx_expect_id, ok,
			       expect_left(rx_expect_id, tx_channels),
			       expect_right(rx_expect_id, tx_channels));
		}
		rx_expect_id++;
	}

	printf("[%s] done: %u / %u blocks verified\n", name, ok, VERIFY_ROUNDS);

	/*
	 * DRAIN/STOP leave the stream in STOPPING until DMA settles, which
	 * blocks the next case's configure/power-down (-EINVAL). DROP forces
	 * READY so stereo -> mono reconfigure can proceed.
	 */
	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		printf("[%s] TX DROP failed %d\n", name, ret);
	}
	ret = i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		printf("[%s] RX DROP failed %d\n", name, ret);
	}

	if (ok < VERIFY_ROUNDS) {
		return -EIO;
	}
	return 0;
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(i2s_tx));
	int ret;

	if (!device_is_ready(dev)) {
		printf("I2S not ready\n");
		return -ENODEV;
	}

	printf("TX loopback verify (slot width %u): jumper TX SD -> RX SD\n",
	       (unsigned int)SLOT_WIDTH);

	ret = run_case(dev, 2U, "stereo TX");
	if (ret < 0) {
		printf("FAIL: stereo TX\n");
		return ret;
	}

	ret = run_case(dev, 1U, "mono TX");
	if (ret < 0) {
		printf("FAIL: mono TX\n");
		return ret;
	}

	printf("PASS: stereo + mono TX cases\n");
	return 0;
}
