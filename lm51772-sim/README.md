# LM51772 Sim — Black Pill #2 (I2C1 slave)

Emulates the LM51772 register interface for the translator (`../translator`)
and presents status on a 16x2 LCD. It is the **I2C1 slave**; the translator is
the master. The LCD lives on a **separate** hardware bus (I2C2) so the two
buses never collide.

Register-level CMSIS (HAL generated but unused). Builds independently:
`make` → `build/PhantomRail.{elf,hex,bin}`.

## Pin map

| Signal           | Pin           | Function |
|------------------|---------------|----------|
| Translator link  | PB6 / PB7     | **I2C1 slave** (addr 0x42) — receives source telemetry from BP#1 |
| LCD              | PB10 / PB3    | **I2C2 master** → PCF8574 1602 backpack (SCL=PB10 AF4, SDA=PB3 AF9) |
| Status LED       | PC13          | on-board LED (active low) |
| Enable / disable | TBD           | reads BP#1's state line |

## Status

- **LCD driver** (`Src/lcd1602.c`) — register-level hardware **I2C2**,
  PCF8574 + HD44780 4-bit, standard-mode 100 kHz, DWT µs delays, per-transfer
  timeouts + NACK detection. `lcd_init()` **auto-detects** the backpack
  address (0x27 or 0x3F) and returns it (0 = bus silent).
- **I2C1 slave** (`Src/i2c1_slave.c`) — interrupt-driven (clock-stretch-safe,
  so the blocking LCD writes never drop bytes), address `0x42`. Validates each
  frame (`../Protocol/phantom_link.h`, XOR check) and feeds the `IN:` row with
  the real source voltage + cell count from the translator. Shows
  `IN:--S --.--V` until a valid frame arrives or if the link goes stale
  (>1.5 s).
- **Display** — row0 (IN) shows the received source plus the translator's sag
  warning: `IN:6S 22.75V` charged, `…V LOW` in the 3.2-3.6 V/cell band, or
  `…(x_X)` below 3.2 V/cell. Row1 (OUT) is a stubbed LM51772 setpoint until FC
  control lands. A `LINK_DEBUG` toggle in `main.c` swaps row1 for I2C1 link
  counters during bring-up.
- **PC13 heartbeat** — 3 flashes at boot (proof of running from flash), then
  ~2 Hz when live source frames are arriving / ~3 Hz when the link is idle.

The link needs external pull-ups on SDA/SCL and a common ground — see the root
README.

## Notes / gotchas

- I2C2 AF asymmetry: **PB10/SCL = AF4, PB3/SDA = AF9** (setting both to AF4
  leaves SDA dead). Handled in the driver.
- Wrong address fails fast (NACK) → blank screen, not a hang.
- Blank-but-backlit is usually the contrast pot, not firmware.
