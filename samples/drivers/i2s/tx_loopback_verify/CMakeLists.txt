# SPDX-License-Identifier: Apache-2.0
# Bring-up: verify Silabs USART I2S TX (W16D16 / W32D16) via GPIO loopback.

cmake_minimum_required(VERSION 3.20.0)

set(SLOT_WIDTH 16 CACHE STRING "I2S bus slot width in bits (16: W16D16, 32: W32D16)")
if(NOT SLOT_WIDTH STREQUAL "16" AND NOT SLOT_WIDTH STREQUAL "32")
	message(FATAL_ERROR "SLOT_WIDTH must be 16 or 32")
endif()
if(SLOT_WIDTH STREQUAL "32")
	list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/slot_width_32.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tx_loopback_verify)

set(FRAME_CLK_HZ 16000 CACHE STRING "I2S frame (LRCLK) rate in Hz")

target_sources(app PRIVATE src/main.c)
target_compile_definitions(app PRIVATE
	FRAME_CLK_HZ=${FRAME_CLK_HZ}
)
