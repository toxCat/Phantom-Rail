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
| Translator link  | PB6 / PB7     | **I2C1 slave** (addr 0x42) — receives source telemetry, returns current on a read |
| Current sense    | PA0           | ADC1_IN0 ← ACS709 VIOUT (simulated LM51772 internal sensor) |
| LCD              | PB10 / PB3    | **I2C2 master** → PCF8574 1602 backpack (SCL=PB10 AF4, SDA=PB3 AF9) |
| Status LED       | PC13          | on-board LED (active low) |

## Status

- **LCD driver** (`Src/lcd1602.c`) — register-level hardware **I2C2**,
  PCF8574 + HD44780 4-bit, standard-mode 100 kHz, DWT µs delays, per-transfer
  timeouts + NACK detection. `lcd_init()` **auto-detects** the backpack
  address (0x27 or 0x3F) and returns it (0 = bus silent).
- **I2C1 slave** (`Src/i2c1_slave.c`) — interrupt-driven (clock-stretch-safe,
  so the blocking LCD writes never drop bytes), address `0x42`. On a master
  **write** it validates the telemetry frame (XOR) and feeds the `IN:` row; on
  a master **read** it transmits the current frame (`pr_build_current`). Shows
  `IN:--S --.--V` until a valid frame arrives or if the link goes stale (>1.5 s).
- **Current sense** (`acs_read_ma` in `main.c`) — register-level ADC on PA0
  reads the ACS709 VIOUT, converts to mA (tunable `ACS_ZERO_MV` /
  `ACS_SENS_MV_PER_A`), publishes it to the slave for the translator to read,
  and shows it on row1 as `…V X.XXA`. Exposed via `g_cur_ma` for SWD. Each
  conversion does a clean start (clears `ADC_SR` so a stale `EOC` can't be read
  as 0); samples are oversampled (`ACS_OVERSAMPLE`) and taken at 2 Hz
  (`CUR_SAMPLE_MS`) with last-good hold, so the reading is steady.
- **Display** — row0 (IN) shows the received source plus the translator's sag
  warning: `IN:6S 22.75V` charged, `…V LOW` in the 3.2-3.6 V/cell band, or
  `…(x_X)` below 3.2 V/cell. Row1 (OUT) shows the stubbed setpoint plus the live
  current, e.g. `OUT:12.00V 1.85A`. A `LINK_DEBUG` toggle in `main.c` swaps row1
  for I2C1 link counters during bring-up.
- **PC13 heartbeat** — 3 flashes at boot (proof of running from flash), then
  ~2 Hz when live source frames are arriving / ~3 Hz when the link is idle.

The link needs external pull-ups on SDA/SCL and a common ground — see the root
README.

## Notes / gotchas

- I2C2 AF asymmetry: **PB10/SCL = AF4, PB3/SDA = AF9** (setting both to AF4
  leaves SDA dead). Handled in the driver.
- Wrong address fails fast (NACK) → blank screen, not a hang.
- Blank-but-backlit is usually the contrast pot, not firmware.
