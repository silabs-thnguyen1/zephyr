# SPDX-License-Identifier: Apache-2.0
# sl_i2s_tx_mono_pattern_test: local mono TX staircase; not for upstream CI.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sl_i2s_tx_mono_pattern_test)

set(FRAME_CLK_HZ 16000 CACHE STRING "I2S frame (LRCLK) rate in Hz")

target_sources(app PRIVATE src/main.c)
target_compile_definitions(app PRIVATE FRAME_CLK_HZ=${FRAME_CLK_HZ})