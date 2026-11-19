# I2S loopback verify — TX to RX (staircase)

Bring-up sample for **xG27-RB4194A** on a WSTK mainboard (BRD4001A / BRD4002A).
It transmits stereo I2S (left = PCM staircase by block ID, right = 0), receives
through a **GPIO jumper** from PC0 to PC1, and reports pass/fail for 200 blocks
on the UART console.

Pin map and expansion header: [UG551 BRD4194A](https://www.silabs.com/documents/public/user-guides/ug551-brd4194a-user-guide.pdf).

## Overview

The sample checks that RX DMA does not drop or corrupt data during sustained
TX/RX loopback. Each block uses a distinct left-channel level derived from a
monotonic block ID so mismatches are easy to spot in the log or on a logic
analyzer.

## Build and flash

Run **west** from the parent of the west workspace (the directory that contains
`zephyr/` and `modules/`). If the workspace points at a different Zephyr checkout
than the tree you edit, set:

```text
export ZEPHYR_BASE=/path/to/zephyr
```

```text
west build -p always -b xg27_rb4194a \
  zephyr/samples/drivers/i2s/loopback_verify \
  -d build/i2s_loopback_verify

west flash -d build/i2s_loopback_verify
```

Optional frame rate and word size (defaults: 16 kHz, 16-bit):

```text
west build -p always -b xg27_rb4194a \
  zephyr/samples/drivers/i2s/loopback_verify \
  -d build/i2s_loopback_verify \
  -- -DFRAME_CLK_HZ=16000 -DWORD_SIZE=16
```

The debug console uses the board controller USART (not USART0 used for I2S).
Open the J-Link/WSTK virtual COM port at **115200 8N1**.

Success ends with: `200 / 200 blocks verified`.

## Wiring (RB4194A expansion header)

USART0 I2S pins in the Zephyr overlay:

| I2S signal | MCU pin | EXP (RB4194A) | Notes |
|------------|---------|-----------------|-------|
| SD / TX (DOUT) | **PC0** | **EXP4** | Loopback source; LA ch 0 |
| SD / RX (DIN) | **PC1** | **EXP6** | Jumper destination; LA ch 1 (optional) |
| BCLK | **PC2** | **EXP8** | LA ch 2 |
| WS / LRCLK | **PC3** | **EXP10** | LA ch 3 |
| GND | GND | EXP GND | Common for LA, jumper, and board |

**Required loopback jumper (before running RX):**

```text
EXP4 (PC0, TX SD)  ---- short wire ----  EXP6 (PC1, RX SD)
```

- Jumper **data only**; do not tie BCLK or WS externally (they are on PC2/PC3).
- Disconnect **MAX98357** / **INMP411** while running this loopback test.
- In `boards/silabs/radio_boards/xg27/xg27_rb4194a.dts` (`exp-header` gpio-map),
  **PC1 is EXP6**, not EXP5. If the silkscreen disagrees, use **UG551** and wire
  **PC0 to PC1** on the correct expansion pins.

The overlay disables on-board flash (`mx25r80`) and the LCD because they share
USART0.

## Expected behavior

1. Prefill six TX blocks, then `START RX`, then `START TX`.
2. About 200 read/verify/write rounds; left-channel levels follow block IDs
   (1000, 2000, 3000, …, modulo 30).
3. Optional logic analyzer: decode 16-bit I2S; **left** = staircase, **right** ≈ 0.
   Trigger on reset or WS.

## Troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| `first L=0 R=0`, LA shows I2S on **DOUT/TX** (EXP4) only | Wrong jumper: use **EXP4 → EXP6**, not EXP5. LA proves TX only. |
| `write failed … -5` (`-EIO`) | TX out of credits or ERROR after bad RX — fix jumper and reset. |
| `read failed` | Missing `rxdatav` in the overlay, or TX not started (gated BCLK). |
| `rx id … mismatch` | Check `rxdatav`, PC0↔PC1 jumper, and that TX remains started. |