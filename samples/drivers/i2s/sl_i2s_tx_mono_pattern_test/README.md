# Silicon Labs I2S TX mono-pattern test

**Sample path:** ``samples/drivers/i2s/sl_i2s_tx_mono_pattern_test``  
**CMake project:** ``sl_i2s_tx_mono_pattern_test``

Mono TX bring-up on **xG27-RB4194A**: same streaming pattern as the **mono TX** case in
``samples/drivers/i2s/tx_loopback_verify`` — per-block PCM level
``1000 + (id % 30) * 1000``, **prefill 6** blocks, **START TX**, then **200** pipelined
writes. **No RX**, GPIO loopback, or software verify.

Each block is 32 LRCLK frames at 16 kHz (64 bytes mono). Audio appears on the bus slot
selected by ``silabs,mono-tx-slot`` (default right), same as loopback verify.

Requires devicetree **``txbl``** + **``txblright``** (mono DMA split).

## Build and flash

```text
west build -p always -b xg27_rb4194a \
  zephyr/samples/drivers/i2s/sl_i2s_tx_mono_pattern_test \
  -d build/sl_i2s_tx_mono_pattern_test

west flash -d build/sl_i2s_tx_mono_pattern_test
```

Console **115200 8N1**. Progress lines every 16 blocks; ends with
``done: 206 TX blocks queued (prefill 6 + stream 200)``.

## Signals (RB4194A)

| I2S signal | MCU | EXP |
|------------|-----|-----|
| TX SD | PC0 | EXP4 |
| BCLK | PC2 | EXP8 |
| WS | PC3 | EXP10 |

No jumper from TX to RX is required.