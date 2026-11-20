/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Verify I2S TX (W32D16 by default) by looping PC0 (TX SD) to PC1 (RX SD).
 * Left slot carries a per-block PCM staircase; right slot is zero. RX checks
 * that the pattern matches what TX DMA shifted onto the bus (200 blocks).
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>

#ifndef FRAME_CLK_HZ
#define FRAME_CLK_HZ 16000
#endif
#ifndef WORD_SIZE
#define WORD_SIZE 32
#endif

#if (WORD_SIZE != 16) && (WORD_SIZE != 32)
#error WORD_SIZE must be 16 or 32
#endif

#define NUM_BLOCKS     8
#define PREFILL_BLOCKS 6
#define VERIFY_ROUNDS  200U

#define SAMPLES_PER_FRAME 32

#if WORD_SIZE == 16
#define SLOT_BYTES      2U
#define SAMPLE_T        int16_t
#else
#define SLOT_BYTES      4U
#define SAMPLE_T        int32_t
#endif

#define FRAME_BYTES (2U * SLOT_BYTES)
#define BLOCK_SIZE  (SAMPLES_PER_FRAME * FRAME_BYTES)

#define RX_OFFSET_MAX 4

static uint32_t block_seq;
static uint32_t rx_expect_id;

static char __aligned(WB_UP(32)) _tx_slab_mem[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];
static char __aligned(WB_UP(32)) _rx_slab_mem[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_slab, _tx_slab_mem, WB_UP(BLOCK_SIZE), NUM_BLOCKS);

static STRUCT_SECTION_ITERABLE(k_mem_slab, rx_slab) =
	Z_MEM_SLAB_INITIALIZER(rx_slab, _rx_slab_mem, WB_UP(BLOCK_SIZE), NUM_BLOCKS);

static int16_t level_for_id(uint32_t id)
{
	return (int16_t)(1000 + (id % 30U) * 1000);
}

#if WORD_SIZE == 32
static int16_t w32d16_load_sample(const int32_t *words, uint32_t i)
{
	int32_t raw = words[i];
	int16_t sample = (int16_t)(raw & 0xffff);

	if (sample == 0 && ((uint32_t)raw >> 16) != 0U) {
		sample = (int16_t)(raw >> 16);
	}

	return sample;
}

static void w32d16_store_sample(int32_t *words, uint32_t i, int16_t sample)
{
	words[i] = (int32_t)sample;
}
#endif

static void fill_block(void *tx_block, uint32_t id)
{
	SAMPLE_T *s = tx_block;
	int16_t level = level_for_id(id);

	for (uint32_t i = 0; i < SAMPLES_PER_FRAME; i++) {
#if WORD_SIZE == 16
		s[2U * i] = level;
		s[2U * i + 1U] = 0;
#else
		w32d16_store_sample(s, 2U * i, level);
		w32d16_store_sample(s, 2U * i + 1U, 0);
#endif
	}
}

static int write_one(const struct device *dev, uint32_t id)
{
	void *mem;
	int ret;

	ret = k_mem_slab_alloc(&tx_slab, &mem, K_FOREVER);
	if (ret < 0) {
		return ret;
	}
	fill_block(mem, id);
	ret = i2s_write(dev, mem, BLOCK_SIZE);
	if (ret < 0) {
		k_mem_slab_free(&tx_slab, mem);
	}
	return ret;
}

static int16_t left_sample_at(const SAMPLE_T *s, uint32_t frame_idx)
{
#if WORD_SIZE == 16
	return s[2U * frame_idx];
#else
	return w32d16_load_sample(s, 2U * frame_idx);
#endif
}

static int16_t right_sample_at(const SAMPLE_T *s, uint32_t frame_idx)
{
#if WORD_SIZE == 16
	return s[2U * frame_idx + 1U];
#else
	return w32d16_load_sample(s, 2U * frame_idx + 1U);
#endif
}

static int find_rx_offset(const SAMPLE_T *s, int16_t expect)
{
	for (int off = 0; off <= RX_OFFSET_MAX; off++) {
		if ((uint32_t)(off + 4) > SAMPLES_PER_FRAME) {
			break;
		}
		if (left_sample_at(s, (uint32_t)off) == expect) {
			return off;
		}
	}
	return -1;
}

static int verify_rx_block(void *mem, size_t len, uint32_t id)
{
	const SAMPLE_T *s = mem;
	int16_t expect = level_for_id(id);
	int off;
	int bad = 0;

	if (len != BLOCK_SIZE) {
		printf("rx len %u != %u\n", (unsigned int)len, BLOCK_SIZE);
		return -EINVAL;
	}

	off = find_rx_offset(s, expect);
	if (off < 0) {
		printf("rx id %u: no offset for level %d (first L=%d R=%d)\n",
		       id, expect,
		       (int)left_sample_at(s, 0),
		       (int)right_sample_at(s, 0));
		return -EIO;
	}

	for (int i = off; i < (int)SAMPLES_PER_FRAME; i++) {
		if (left_sample_at(s, (uint32_t)i) != expect) {
			bad++;
		}
		if (right_sample_at(s, (uint32_t)i) != 0) {
			bad++;
		}
	}

	if (bad > 0) {
		printf("rx id %u off %d: %d sample mismatches (expect L=%d R=0)\n",
		       id, off, bad, expect);
		return -EIO;
	}

	return 0;
}

static int read_one(const struct device *dev, uint32_t id)
{
	void *mem;
	size_t len;
	int ret;

	ret = i2s_read(dev, &mem, &len);
	if (ret < 0) {
		printf("read id %u failed %d\n", id, ret);
		return ret;
	}

	ret = verify_rx_block(mem, len, id);
	k_mem_slab_free(&rx_slab, mem);
	return ret;
}

static void i2s_same_bus_cfg(struct i2s_config *cfg)
{
	cfg->word_size = WORD_SIZE;
	cfg->channels = 2U;
	cfg->format = I2S_FMT_DATA_FORMAT_I2S;
	cfg->frame_clk_freq = FRAME_CLK_HZ;
	cfg->block_size = BLOCK_SIZE;
	cfg->timeout = 2000;
	cfg->options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER |
		       I2S_OPT_BIT_CLK_GATED;
}

int main(void)
{
	struct i2s_config cfg;
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(i2s_tx));
	int ret;
	uint32_t n;
	uint32_t ok;
	uint32_t round;

	if (!device_is_ready(dev)) {
		printf("I2S not ready\n");
		return -ENODEV;
	}

	i2s_same_bus_cfg(&cfg);
	cfg.mem_slab = &tx_slab;
	ret = i2s_configure(dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		printf("configure TX failed %d\n", ret);
		return ret;
	}

	cfg.mem_slab = &rx_slab;
	ret = i2s_configure(dev, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		printf("configure RX failed %d\n", ret);
		return ret;
	}

	block_seq = 0U;
	rx_expect_id = 0U;

	printf("TX loopback verify (word_size=%u): jumper TX SD -> RX SD\n",
	       (unsigned int)WORD_SIZE);
	printf("LRCLK=%u Hz, block=%u frames, %u bytes, L=staircase R=0\n",
	       FRAME_CLK_HZ, SAMPLES_PER_FRAME, BLOCK_SIZE);
	printf("Prefill %u TX blocks, START RX then TX\n", PREFILL_BLOCKS);

	for (n = 0; n < PREFILL_BLOCKS; n++) {
		ret = write_one(dev, block_seq++);
		if (ret < 0) {
			printf("prefill write %u failed %d\n", n, ret);
			return ret;
		}
	}

	ret = i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("RX START failed %d\n", ret);
		return ret;
	}

	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("TX START failed %d\n", ret);
		return ret;
	}

	ok = 0U;
	for (round = 0; round < VERIFY_ROUNDS; round++) {
		ret = read_one(dev, rx_expect_id);
		if (ret == 0) {
			ok++;
		}

		ret = write_one(dev, block_seq++);
		if (ret < 0) {
			printf("write failed at round %u: %d\n", round, ret);
			break;
		}

		if ((rx_expect_id % 16U) == 0U) {
			printf("rx id %u ok_total %u / expect L=%d\n",
			       rx_expect_id, ok, level_for_id(rx_expect_id));
		}
		rx_expect_id++;
	}

	printf("done: %u / %u blocks verified (TX path)\n", ok, VERIFY_ROUNDS);
	if (ok < VERIFY_ROUNDS) {
		return -EIO;
	}

	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		printf("TX DRAIN failed %d\n", ret);
	}
	ret = i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
	if (ret < 0) {
		printf("RX STOP failed %d\n", ret);
	}

	return 0;
}