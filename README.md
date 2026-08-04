# Phantom-Rail

Bench-rig firmware for an LM51772 buck-boost control scheme (FPV gate
anti-collision lighting). Two STM32F411 "black pill" boards stand in for the
production parts. See `PROJECT.md` (handoff doc) for the full system design.

## Boards

The repo holds one firmware project per board. Both are STM32F411CEU6 and
share the CMSIS/HAL tree under `Drivers/`.

| Directory        | Board            | I2C1 role | Responsibilities |
|------------------|------------------|-----------|------------------|
| `translator/`    | Black Pill #1    | **master** | FC UART (USART1/MSP), source-sense ADC (PA0), drives the sim board on I2C1 (PB6/PB7), enable/disable + UVLO |
| `lm51772-sim/`   | Black Pill #2    | **slave**  | Emulates the LM51772 register interface, renders the 16x2 LCD over hardware I2C2 (PB10/PB3) |

The **translator is the master**: it reads the source voltage, detects the
pack, and pushes a telemetry frame to the **sim (slave)** over **I2C1 on
PB6/PB7**. The sim board owns the LCD presentation on its own bus (I2C2), so
the two I2C buses never collide.

```
  FC ── USART1 ──▶ translator (BP#1, master) ── I2C1 (PB6/PB7) ──▶ lm51772-sim (BP#2, slave) ── I2C2 ──▶ 1602 LCD
                        │
                     ADC1_IN0 (PA0, /9 divider) = source voltage
```

## Inter-board link

The I2C1 wire format lives in `Protocol/phantom_link.h`, `#include`d by both
projects so master and slave can't drift. The master writes a 6-byte telemetry
frame (`CMD, VIN_lo, VIN_hi, cells, flags, xor`) to slave address `0x42` each
control cycle; the slave receives it under interrupt (clock-stretch-safe) and
renders it.

**Wiring (do this before expecting anything on screen):**
- `BP#1 PB6 (SCL) ── BP#2 PB6 (SCL)` and `BP#1 PB7 (SDA) ── BP#2 PB7 (SDA)`
- **external pull-ups** on SDA and SCL — ~4.7 kΩ to 3V3 (the STM32 pins are
  open-drain; unlike the LCD backpack, this link has none of its own)
- **common ground** between the two boards

## Status

- `translator/` — **detects the pack and drives the link.** Register-level
  ADC1_IN0 reader (PA0, /9 → `g_source_mv`), Betaflight-style cell-count
  detection, and a register-level I2C1 **master** that pushes each reading to
  the sim. PC13 heartbeat. TODO: USART1/MSP, UVLO/hysteresis, selector logic.
- `lm51772-sim/` — **displays the real source over I2C1.** Interrupt-driven
  I2C1 **slave** feeds the LCD's `IN:nS XX.XXV` row (shows `--` on link loss);
  register-level hardware-I2C2 LCD driver (PCF8574 + HD44780, address
  auto-detect); PC13 heartbeat. OUT is still a stubbed LM51772 setpoint.

## Build & flash

Each board builds independently from its own directory (needs
`arm-none-eabi-gcc`):

```bash
cd translator      # or: cd lm51772-sim
make               # -> build/<target>.{elf,hex,bin}
```

Flash over SWD with an ST-Link V2 (BOOT0 low so it runs from flash):

```bash
st-flash --reset write build/translator.bin 0x08000000       # translator
st-flash --reset write build/PhantomRail.bin 0x08000000      # lm51772-sim
```

Or via OpenOCD:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/<target>.elf verify reset exit"
```
