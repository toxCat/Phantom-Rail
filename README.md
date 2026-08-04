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

The **translator is the master**: it acquires the VREF/source reading and
pushes it to the **sim (slave)** over **I2C1 on PB6/PB7**. The sim board owns
the LCD presentation on its own bus (I2C2), so the two I2C buses never collide.

```
  FC ── USART1 ──▶ translator (BP#1, master) ── I2C1 (PB6/PB7) ──▶ lm51772-sim (BP#2, slave) ── I2C2 ──▶ 1602 LCD
                        │
                     ADC1_IN0 (PA0, /9 divider) = source voltage
```

## Status

- `lm51772-sim/` — LCD bring-up working: register-level hardware-I2C2 driver
  (PCF8574 + HD44780), PC13 heartbeat, auto-detects the backpack address
  (0x27/0x3F). Still stubbed: the I2C1 slave link and the real IN voltage.
- `translator/` — skeleton: register-level ADC1_IN0 source-sense reader
  (PA0, /9 → millivolts in `g_source_mv`) + PC13 heartbeat. TODO: USART1/MSP,
  the I2C1 master link to the sim, UVLO/hysteresis, static-selector logic.

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
