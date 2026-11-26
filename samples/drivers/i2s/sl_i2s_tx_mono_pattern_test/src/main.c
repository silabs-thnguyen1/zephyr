/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Same mono TX staircase as tx_loopback_verify (level 1000 + (id % 30) * 1000
 * per block, streaming prefill + pipelined writes). No RX, loopback, or verify.
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/devicetree.h>

#ifndef FRAME_CLK_HZ
#define FRAME_CLK_HZ 16000
#endif

#define WORD_SIZE 16

#if DT_NODE_HAS_PROP(DT_ALIAS(i2s_tx), silabs_slot_width)
#define SLOT_WIDTH DT_PROP(DT_ALIAS(i2s_tx), silabs_slot_width)
#else
#define SLOT_WIDTH 16
#endif

#define NUM_BLOCKS     8
#define PREFILL_BLOCKS 6
#define TX_ROUNDS      200U

#define SAMPLES_PER_FRAME 32

#define MONO_FRAME_BYTES (1U * 2U)
#define MONO_BLOCK_SIZE  (SAMPLES_PER_FRAME * MONO_FRAME_BYTES)

/* silabs,mono-tx-slot enum indexes (binding order: left, right). */
#define MONO_TX_SLOT_LEFT  0
#define MONO_TX_SLOT_RIGHT 1

#if DT_NODE_HAS_PROP(DT_ALIAS(i2s_tx), silabs_mono_tx_slot)
#define MONO_TX_SLOT DT_ENUM_IDX(DT_ALIAS(i2s_tx), silabs_mono_tx_slot)
#else
#define MONO_TX_SLOT MONO_TX_SLOT_RIGHT
#endif

static uint32_t block_seq;

static char __aligned(WB_UP(32)) _tx_slab_mem[(NUM_BLOCKS) * WB_UP(MONO_BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_slab, _tx_slab_mem, WB_UP(MONO_BLOCK_SIZE), NUM_BLOCKS);

static int16_t level_for_id(uint32_t id)
{
	return (int16_t)(1000 + (id % 30U) * 1000);
}

static void fill_block_mono(void *tx_block, uint32_t id)
{
	int16_t *s = tx_block;
	int16_t level = level_for_id(id);

	for (uint32_t i = 0; i < SAMPLES_PER_FRAME; i++) {
		s[i] = level;
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
	fill_block_mono(mem, id);
	ret = i2s_write(dev, mem, MONO_BLOCK_SIZE);
	if (ret < 0) {
		k_mem_slab_free(&tx_slab, mem);
	}
	return ret;
}

static void i2s_tx_cfg(struct i2s_config *cfg)
{
	cfg->word_size = WORD_SIZE;
	cfg->channels = 1U;
	cfg->format = I2S_FMT_DATA_FORMAT_I2S;
	cfg->frame_clk_freq = FRAME_CLK_HZ;
	cfg->block_size = MONO_BLOCK_SIZE;
	cfg->timeout = 2000;
	cfg->options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER |
		       I2S_OPT_BIT_CLK_GATED;
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(i2s_tx));
	struct i2s_config cfg;
	uint32_t n;
	uint32_t round;
	int ret;

	if (!device_is_ready(dev)) {
		printf("I2S not ready\n");
		return -ENODEV;
	}

	i2s_tx_cfg(&cfg);
	cfg.mem_slab = &tx_slab;
	ret = i2s_configure(dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		printf("configure TX failed %d\n", ret);
		return ret;
	}

	block_seq = 0U;

	printf("--- mono TX pattern (same as tx_loopback_verify mono case) ---\n");
	printf("word_size=%u slot_width=%u (W%uD16) LRCLK=%u Hz, %u frames/block, TX block=%u B\n",
	       (unsigned int)WORD_SIZE, (unsigned int)SLOT_WIDTH, (unsigned int)SLOT_WIDTH,
	       FRAME_CLK_HZ, SAMPLES_PER_FRAME, (unsigned int)MONO_BLOCK_SIZE);
	printf("mono slot=%s on bus; level = 1000 + (id %% 30) * 1000 per block\n",
	       (MONO_TX_SLOT == MONO_TX_SLOT_LEFT) ? "left" : "right");
	printf("Prefill %u TX blocks, then START TX; stream %u more blocks (no RX)\n",
	       PREFILL_BLOCKS, TX_ROUNDS);

	for (n = 0; n < PREFILL_BLOCKS; n++) {
		ret = write_one(dev, block_seq++);
		if (ret < 0) {
			printf("prefill write %u failed %d\n", n, ret);
			return ret;
		}
	}

	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("TX START failed %d\n", ret);
		return ret;
	}

	for (round = 0; round < TX_ROUNDS; round++) {
		ret = write_one(dev, block_seq++);
		if (ret < 0) {
			printf("write failed at round %u: %d\n", round, ret);
			return ret;
		}

		if ((round % 16U) == 0U) {
			printf("tx id %u level %d\n", block_seq - 1U,
			       (int)level_for_id(block_seq - 1U));
		}
	}

	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		printf("TX DROP failed %d\n", ret);
		return ret;
	}

	printf("done: %u TX blocks queued (prefill %u + stream %u)\n",
	       PREFILL_BLOCKS + TX_ROUNDS, PREFILL_BLOCKS, TX_ROUNDS);

	return 0;
}