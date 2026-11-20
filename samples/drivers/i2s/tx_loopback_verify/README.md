# I2S TX loopback verify (W32D16)

Bring-up sample for **Silicon Labs USART I2S** on xG27 boards. It checks that
**TX DMA** shifts the expected stereo pattern on the bus by looping **TX SD to
RX SD** and comparing 200 consecutive blocks. Default **word_size is 32**
(W32D16), the format used by ``samples/drivers/i2s/echo`` on xG27.

RX here is only a witness; mismatches mean the TX path (LDMA beat size, slot
layout, or BCLK) is wrong.

## Build and flash

From the west workspace parent (set ``ZEPHYR_BASE`` if needed):

```text
west build -p always -b xg27_rb4194a \
  zephyr/samples/drivers/i2s/tx_loopback_verify

west flash
```

DK2602A:

```text
west build -p always -b xg27_dk2602a \
  zephyr/samples/drivers/i2s/tx_loopback_verify
```

Optional 16-bit regression (W16D16):

```text
west build -p always -b xg27_rb4194a \
  zephyr/samples/drivers/i2s/tx_loopback_verify \
  -- -DWORD_SIZE=16
```

Success on the UART console: ``200 / 200 blocks verified (TX path)``.

## Wiring

| Signal | RB4194A | DK2602A |
|--------|---------|---------|
| TX SD | EXP4 (PC0) | PC0 |
| RX SD | EXP6 (PC1) | PC1 |
| BCLK | EXP8 (PC2) | PC2 |
| WS | EXP10 (PC3) | PA8 |

**Required:** short **TX SD to RX SD** (do not tie BCLK/WS externally). Remove
INMP411 / MAX98357 while running loopback.

See ``samples/drivers/i2s/loopback_verify/README.md`` for RB4194A header details.

## Logic analyzer (optional)

Decode I2S with **32-bit slots**, 16 kHz frame rate. Left channel should show the
staircase; right should be near zero. BCLK is **64 × LRCLK** for W32D16.