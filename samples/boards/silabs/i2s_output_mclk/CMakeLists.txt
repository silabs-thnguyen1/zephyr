# LOCAL ONLY — do not push.
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2s_output_mclk_loop)

set(FRAME_CLK_HZ 16000 CACHE STRING "I2S WS / frame clock (Hz)")
set(WORD_SIZE 16 CACHE STRING "I2S word size (16 or 32)")
set(CHANNEL_MODE mono_left CACHE STRING "stereo | mono_left | mono_right")

target_sources(app PRIVATE src/main.c)
target_compile_definitions(app PRIVATE
  FRAME_CLK_HZ=${FRAME_CLK_HZ}
  WORD_SIZE=${WORD_SIZE}
)

if(CHANNEL_MODE STREQUAL "mono_left")
  target_compile_definitions(app PRIVATE I2S_CHANNELS=1)
elseif(CHANNEL_MODE STREQUAL "mono_right")
  target_compile_definitions(app PRIVATE I2S_CHANNELS=1)
  message(WARNING "mono_right also needs silabs,mono-tx-slot = \"right\" in the overlay")
elseif(CHANNEL_MODE STREQUAL "stereo")
  target_compile_definitions(app PRIVATE I2S_CHANNELS=2)
else()
  message(FATAL_ERROR "CHANNEL_MODE must be stereo, mono_left, or mono_right")
endif()
