# Translator — Black Pill #1 (I2C1 master)

FC-facing board. Reads the source voltage, talks MSP to the flight controller,
and drives the LM51772 sim board (`../lm51772-sim`) as **I2C1 master**.

Register-level CMSIS, no HAL. Builds independently: `make` → `build/translator.{elf,hex,bin}`.

## Pin map (target)

| Signal            | Pin        | Function |
|-------------------|------------|----------|
| Source sense      | PA0        | ADC1_IN0, via 120k/15k divider (/9, ~29 V full scale) |
| FC UART           | PA9 / PA10 | USART1 TX/RX ↔ FC (MSP) |
| Sim link          | PB6 / PB7  | **I2C1 master** → Black Pill #2 |
| Status LED        | PC13       | on-board LED (active low) |
| Enable / selectors| TBD        | enable-disable out + 4 static selectors in |

## Status

Skeleton. Implemented this pass:
- **ADC1_IN0 source-sense reader** — PA0, 12-bit, 480-cycle sample (high-Z
  120k divider), scaled by /9 into `g_source_mv` (millivolts). It's a
  `volatile` global, so with no other output yet you can watch it live over
  SWD (e.g. `gdb`/`st-util`).
- **PC13 heartbeat** (~2 Hz) as a liveness signal.

TODO: USART1/MSP polling, the I2C1 master transfer that pushes the reading to
the slave, the enable/disable GPIO, UVLO (hysteresis + debounce), and the
static-selector truth table.
