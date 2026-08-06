# Phantom-Rail

Bench-rig firmware for an LM51772 buck-boost control scheme (FPV gate
anti-collision lighting). Two STM32F411 "black pill" boards stand in for the
production parts. See `PROJECT.md` (handoff doc) for the full system design.

## Boards

The repo holds one firmware project per board. Both are STM32F411CEU6 and
share the CMSIS/HAL tree under `Drivers/`.

| Directory        | Board            | I2C1 role | Responsibilities |
|------------------|------------------|-----------|------------------|
| `translator/`    | Black Pill #1    | **host / master** | Source-sense ADC (PA0), pack detection + sag monitor, Power FET (PA1), drives the sim's LM51772 registers over I2C1 (PB6/PB7); FC UART next |
| `lm51772-sim/`   | Black Pill #2    | **LM51772 / slave** | Presents the real LM51772 register interface (addr 0x6A), current sense (ACS709 on PA0), 16x2 LCD over hardware I2C2 (PB10/PB3) |

The **translator is the host/master**: it acts like a flight controller driving
a real LM51772 — writing the sim's control registers (output voltage, current
limit, enable) and reading status. The sim board owns the LCD on its own bus
(I2C2), so the two I2C buses never collide.

```
  FC ── USART1 ──▶ translator (BP#1, host) ── I2C1 (PB6/PB7) ──▶ lm51772-sim (BP#2, LM51772 @0x6A) ── I2C2 ──▶ 1602 LCD
                        │                                              │
                   PA0 = battery (VIN), PA1 = Power FET          PA0 = ACS709 output current (Iout)
```

## Inter-board link (LM51772 register interface)

The sim mimics the TI **LM51772** buck-boost controller's I2C register map
(datasheet SNVSC22D); `Protocol/lm51772_regs.h` is the shared definition
(`#include`d by both projects). Standard register-addressed I2C at slave
address **0x6A** — a write is `S ADDR+W REG D0 [D1 …] P` (auto-increment); a
read is `S ADDR+W REG Sr ADDR+R D0 … nA P`. The slave services both under
interrupt (clock-stretch-safe).

Key registers the host drives: **VOUT_TARGET** (0x0C/0x0D, 12-bit ×20 mV →
commanded output), **ILIM_THRESHOLD** (0x0A → current limit), **CONV_EN2**
(0x81 bit0 → enable), read-back **STATUS_BYTE** (0x78) / **CC_OPERATION**
(0x21). The battery/sag view (no VIN telemetry exists in the real IC) is passed
to the sim's display via clearly-marked Phantom-Rail **extension registers**
(0xE0–0xE3).

**Wiring (do this before expecting anything on screen):**
- `BP#1 PB6 (SCL) ── BP#2 PB6 (SCL)` and `BP#1 PB7 (SDA) ── BP#2 PB7 (SDA)`
- **external pull-ups** on SDA and SCL — ~4.7 kΩ to 3V3 (the STM32 pins are
  open-drain; unlike the LCD backpack, this link has none of its own)
- **common ground** between the two boards

## Status

- `translator/` — **host controller + FC bridge.** Register-level ADC1_IN0
  reader (PA0, /9), cell-count detection latched at plug-in, N-channel **Power
  FET on PA1**. Polls Betaflight over **MSP** (USART1, PA9/PA10 @ 115200) for the
  transmitter's pot/switch (AUX1 → voltage, AUX2 → enable), then writes the sim's
  LM51772 registers (VOUT_TARGET from the pot, ILIM = 2 A, CONV_EN2 from the
  switch) and reads back CC_OPERATION; a **debounced over-current** (CC held
  ≥1 s) latches the FET off, and a **critical battery vetoes** the enable. PC13
  heartbeat. TODO: transmitter Lua script (Task 3).
- `lm51772-sim/` — **LM51772 register model.** Interrupt-driven register-addressed
  I2C1 slave @ 0x6A with the datasheet register file. Row0 shows the battery IN
  (from extension registers) with the sag warning (`LOW`/`(x_X)`); row1 shows the
  LM51772 output view `12.00V 1.85A ON` (commanded VOUT, measured ACS709 current,
  status ON/CC/OC/OFF). Register-level hardware-I2C2 LCD driver (PCF8574 +
  HD44780, auto-detect); PC13 heartbeat.

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
