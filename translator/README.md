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

Implemented:
- **ADC1_IN0 source-sense reader** — PA0, 12-bit, 480-cycle sample (high-Z
  120k divider), scaled by 135/15 (=/9) into `g_source_mv` (millivolts), also
  a `volatile` global you can watch over SWD.
- **Pack detection** — LiPo cell count via the Betaflight convention (smallest
  S whose 4.3 V/cell ceiling covers the reading), in `g_cells`.
- **I2C1 master** (`Core/Src/i2c1_master.c`) — register-level, standard-mode
  100 kHz on PB6/PB7. Each ~250 ms it builds a telemetry frame
  (`../Protocol/phantom_link.h`) and writes it to the sim at `0x42`;
  `g_link_ok` reflects whether the last push was ACKed.
- **PC13 heartbeat** (~2 Hz).

Bench check: a 22.94 V 6S pack read 2.528 V at PA0 → 22.75 V computed → `6S`
detected (22.75/6 = 3.79 V/cell). The ~0.8 % low reading is resistor
tolerance; trim `VDIV_NUM`/`VDIV_DEN` in `main.c` with measured resistor
values if you want it exact.

The link needs external pull-ups on SDA/SCL and a common ground — see the root
README.

TODO: USART1/MSP polling, the enable/disable GPIO, UVLO (hysteresis +
debounce), and the static-selector truth table.
