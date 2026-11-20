# SPDX-License-Identifier: Apache-2.0
# Bring-up: verify Silabs USART I2S TX (especially W32D16) via GPIO loopback.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tx_loopback_verify)

set(FRAME_CLK_HZ 16000 CACHE STRING "I2S frame (LRCLK) rate in Hz")
set(WORD_SIZE 32 CACHE STRING "I2S word size in bits (16 or 32)")

target_sources(app PRIVATE src/main.c)
target_compile_definitions(app PRIVATE
	FRAME_CLK_HZ=${FRAME_CLK_HZ}
	WORD_SIZE=${WORD_SIZE}
)