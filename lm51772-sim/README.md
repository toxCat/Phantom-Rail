# LM51772 Sim — Black Pill #2 (LM51772 model / I2C1 slave)

Presents the real **LM51772** I2C register interface (datasheet SNVSC22D) to
the translator (`../translator`) and shows state on a 16x2 LCD. It is the
**I2C1 slave at address 0x6A**; the translator is the host/master. The LCD
lives on a **separate** hardware bus (I2C2) so the two buses never collide.

Register-level CMSIS (HAL generated but unused). Builds independently:
`make` → `build/PhantomRail.{elf,hex,bin}`.

## Pin map

| Signal           | Pin           | Function |
|------------------|---------------|----------|
| Host link        | PB6 / PB7     | **I2C1 slave** — LM51772 register file @ 0x6A |
| Current sense    | PA0           | ADC1_IN0 ← ACS709 VIOUT = the IC's internal Iout |
| LCD              | PB10 / PB3    | **I2C2 master** → PCF8574 1602 backpack (SCL=PB10 AF4, SDA=PB3 AF9) |
| Status LED       | PC13          | on-board LED (active low) |

## Status

- **LCD driver** (`Src/lcd1602.c`) — register-level hardware **I2C2**,
  PCF8574 + HD44780 4-bit, standard-mode 100 kHz, DWT µs delays, per-transfer
  timeouts + NACK detection. `lcd_init()` **auto-detects** the backpack
  address (0x27 or 0x3F) and returns it (0 = bus silent).
- **LM51772 register slave** (`Src/i2c1_slave.c`) — interrupt-driven,
  clock-stretch-safe, register-addressed with auto-increment (write sets the
  pointer then streams data; read returns from the pointer). Holds the datasheet
  register file (`../Protocol/lm51772_regs.h`): reset values, VOUT_TARGET, ILIM,
  CONV_EN2, computed STATUS_BYTE / CC_OPERATION, CLEAR_FAULTS, and the
  Phantom-Rail battery extension registers (0xE0–0xE3).
- **Current sense** (`acs_read_ma` in `main.c`) — register-level ADC on PA0
  reads the ACS709 VIOUT → mA (tunable `ACS_ZERO_MV` / `ACS_SENS_MV_PER_A`);
  this is the IC's Iout, compared to the programmed ILIM to raise CC/OC. Each
  conversion clean-starts (`ADC_SR = 0` so a stale `EOC` can't read as 0);
  oversampled (`ACS_OVERSAMPLE`) at 2 Hz (`CUR_SAMPLE_MS`) with last-good hold.
- **Display** — row0 (IN, battery from the extension registers) `IN:6S 22.75V`
  with sag warning (`LOW` / `(x_X)`). Row1 (OUT, LM51772 view)
  `12.00V 1.85A ON` = commanded VOUT (from VOUT_TARGET), measured current, and
  status **ON** / **CC** (in current-limit) / **OC** (over-current latched) /
  **OFF** (CONV_EN2 = 0).
- **PC13 heartbeat**.

The link needs external pull-ups on SDA/SCL and a common ground — see the root
README.

## Notes / gotchas

- I2C2 AF asymmetry: **PB10/SCL = AF4, PB3/SDA = AF9** (setting both to AF4
  leaves SDA dead). Handled in the driver.
- Wrong address fails fast (NACK) → blank screen, not a hang.
- Blank-but-backlit is usually the contrast pot, not firmware.
