# Local MCLK + looping I2S (WIP / local)

Upstream `samples/drivers/i2s/output` bursts ~30 ms then DRAIN. This app loops
a 1 kHz sine so BCLK/WS stay on. Uses existing Silabs Series clock-output for MCLK
(`silabs,series-clock-output` — already in-tree).

## Build / flash

From a west workspace with this tree as `ZEPHYR_BASE`:

RB4194A — 1 kHz sine, mono left, 16 kHz, 16-bit:

```powershell
west build -p always -b xg27_rb4194a samples/boards/silabs/i2s_output_mclk `
  -d build/i2s_output_mclk_rb `
  -- "-DFRAME_CLK_HZ=16000" `
     "-DWORD_SIZE=16" `
     "-DCHANNEL_MODE=mono_left"
west flash -d build/i2s_output_mclk_rb
```

DK2602A: same CMake flags, `-b xg27_dk2602a -d build/i2s_output_mclk`

UART 115200: `I2S loop 1kHz sine: WS=16000 Hz, 16-bit, ch=1, BCLK~512000 Hz`

## Probe (this config)

| Signal | RB4194A | Expect |
|--------|---------|--------|
| MCLK | PB0 / EXP7 | ~2.4 MHz, continuous |
| BCLK | PC2 / EXP8 | ~512 kHz (16 kHz × 2 × 16) |
| WS | PC3 / EXP10 | 16 kHz |
| SD TX | PC0 / EXP4 | 1 kHz sine on **left** slot; right silence |

DK: WS is PB2 / EXP10. Keep mic enable (PC7) LOW so PB0 is free for MCLK.
