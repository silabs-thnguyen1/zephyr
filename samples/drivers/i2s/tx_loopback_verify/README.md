# I2S TX loopback verify (W16D16 / W32D16)

Bring-up sample for **Silicon Labs USART I2S** on xG27 boards. It checks that
**TX DMA** shifts the expected pattern on the bus by looping **TX SD to RX SD**
and comparing 200 consecutive blocks, first with stereo TX and then with mono TX
(RX is always stereo).

The bus slot width is a devicetree property (``silabs,slot-width``), so it is
chosen at build time with ``-DSLOT_WIDTH``:

| SLOT_WIDTH | Format | BCLK |
|------------|--------|------|
| 16 (default) | W16D16 | 32 x LRCLK |
| 32 | W32D16 | 64 x LRCLK |

``i2s_config.word_size`` is 16 in both cases and memory holds one 16-bit word
per slot (W32D16 RX returns the 16 MSBs of each slot, TX pads with zeros), so
the data check is the same.

RX here is only a witness; mismatches mean the TX path (LDMA beat size, slot
layout, or BCLK) is wrong.

## Build and flash

From the west workspace parent:

```text
west build -p always -b xg27_rb4194a \
  silabs-zephyr/samples/drivers/i2s/tx_loopback_verify

west flash
```

W32D16 (adds ``slot_width_32.overlay``):

```text
west build -p always -b xg27_rb4194a \
  silabs-zephyr/samples/drivers/i2s/tx_loopback_verify \
  -- -DSLOT_WIDTH=32
```

DK2602A: use ``-b xg27_dk2602a``.

Success on the UART console: ``PASS: stereo + mono TX cases`` (each case reports
``200 / 200 blocks verified``). Run both ``SLOT_WIDTH`` variants.

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

Decode I2S with 16-bit (``SLOT_WIDTH=16``) or 32-bit (``SLOT_WIDTH=32``) slots,
16 kHz frame rate. In the stereo case the left channel shows the staircase and
the right is near zero; in the mono case the audio is on ``silabs,mono-tx-slot``
(right by default).
