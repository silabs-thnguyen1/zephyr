/*
 * LOCAL ONLY — looping 1 kHz sine for scope (MCLK + I2S).
 * Default: 16 kHz, 16-bit, mono left (CMake FRAME_CLK_HZ / WORD_SIZE / CHANNEL_MODE).
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>

#ifndef FRAME_CLK_HZ
#define FRAME_CLK_HZ 16000
#endif
#ifndef WORD_SIZE
#define WORD_SIZE 16
#endif
#ifndef I2S_CHANNELS
#define I2S_CHANNELS 1
#endif

#define NUM_BLOCKS 8

/*
 * One period of 1 kHz at 16 kHz (16 samples). Other FRAME_CLK_HZ values
 * still play this table, so tone = 1 kHz * (FRAME_CLK_HZ / 16000).
 */
static const int16_t sine_1khz_16k[] = {
	     0,  12540,  23170,  30274,  32767,  30274,  23170,  12540,
	     0, -12540, -23170, -30274, -32767, -30274, -23170, -12540,
};

#define SAMPLE_NO ARRAY_SIZE(sine_1khz_16k)
#define FRAME_BYTES ((WORD_SIZE / 8) * I2S_CHANNELS)
#define BLOCK_SIZE  (SAMPLE_NO * FRAME_BYTES)

static void fill_buf(void *tx_block)
{
	if (WORD_SIZE == 16 && I2S_CHANNELS == 1) {
		memcpy(tx_block, sine_1khz_16k, sizeof(sine_1khz_16k));
		return;
	}

	if (WORD_SIZE == 16 && I2S_CHANNELS == 2) {
		int16_t *dst = tx_block;

		for (int i = 0; i < SAMPLE_NO; i++) {
			dst[2 * i] = sine_1khz_16k[i];
			dst[2 * i + 1] = 0;
		}
		return;
	}

	/* 32-bit slot: 16-bit sine in low half */
	if (WORD_SIZE == 32 && I2S_CHANNELS == 1) {
		int32_t *dst = tx_block;

		for (int i = 0; i < SAMPLE_NO; i++) {
			dst[i] = (int32_t)sine_1khz_16k[i];
		}
		return;
	}

	int32_t *dst = tx_block;

	for (int i = 0; i < SAMPLE_NO; i++) {
		dst[2 * i] = (int32_t)sine_1khz_16k[i];
		dst[2 * i + 1] = 0;
	}
}

static char __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
			       WB_UP(BLOCK_SIZE), NUM_BLOCKS);

int main(void)
{
	struct i2s_config i2s_cfg;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2s_tx));
	int ret;
	bool started = false;

	if (!device_is_ready(dev_i2s)) {
		printf("I2S device not ready\n");
		return -ENODEV;
	}

	i2s_cfg.word_size = WORD_SIZE;
	i2s_cfg.channels = I2S_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = FRAME_CLK_HZ;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S stream (%d)\n", ret);
		return ret;
	}

	printf("I2S loop 1kHz sine: WS=%u Hz, %u-bit, ch=%u, BCLK~%u Hz\n",
	       FRAME_CLK_HZ, WORD_SIZE, I2S_CHANNELS,
	       FRAME_CLK_HZ * 2U * WORD_SIZE);

	for (;;) {
		void *mem;

		ret = k_mem_slab_alloc(&tx_0_mem_slab, &mem, K_FOREVER);
		if (ret < 0) {
			printf("slab alloc failed\n");
			return ret;
		}
		fill_buf(mem);

		ret = i2s_write(dev_i2s, mem, BLOCK_SIZE);
		if (ret < 0) {
			printf("i2s_write failed (%d)\n", ret);
			k_mem_slab_free(&tx_0_mem_slab, mem);
			return ret;
		}

		if (!started) {
			ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
			if (ret < 0) {
				printf("I2S START failed (%d)\n", ret);
				return ret;
			}
			started = true;
		}
	}
}
