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
| Power FET gate    | PA1        | N-channel Power FET enable, active-high |
| Status LED        | PC13       | on-board LED (active low) |
| Selectors         | TBD        | 4 static selectors in |

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
  `g_link_ok` reflects whether the last push was ACKed. On a failed push it
  calls `i2c1_master_recover()` (SWRST) so a stuck-BUSY from start-up NACKs
  self-heals — no manual reset needed when the sim boots later than the
  translator.
- **Power FET + sag monitor** (Task 1 + 2) — N-channel gate on **PA1**,
  active-high, starts low (fail-safe). The cell count is **latched at plug-in**
  (so a sagging pack isn't re-counted as fewer cells and mask the sag), and
  per-cell voltage against that count picks one of three bands:
  | per-cell        | band     | FET | sim shows          |
  |-----------------|----------|-----|--------------------|
  | 3.6 – 4.2 V     | charged  | ON  | `IN:6S 22.75V`     |
  | 3.2 – 3.6 V     | low/sag  | ON  | `…V LOW`           |
  | < 3.2 V         | critical | OFF | `…(x_X)`           |
  The band is sent to the sim in the frame flags; `g_fet_on` / `g_cell_mv`
  expose state over SWD.
- **Over-current soft-fail** (Task 3) — each loop the master **reads** the
  sim's ACS709 current back over I2C1 (`i2c1_master_read`). The cutoff is
  **debounced**: the draw must stay ≥ **2 A** (`CUR_LIMIT_MA`) continuously for
  `OC_DEBOUNCE_MS` (1 s) before the FET latches off, so a motor-ramp transient
  doesn't nuisance-trip — the FET is the sustained-fault backstop, not a fast
  limiter (the LM51772 handles that). The latch clears when the pack is removed
  (re-arm). The translator never senses current directly. `g_current_ma` /
  `g_oc_active` / `g_oc_fault` expose state over SWD.
- **PC13 heartbeat**.

Bench check: a 22.94 V 6S pack read 2.528 V at PA0 → 22.75 V computed → `6S`
detected (22.75/6 = 3.79 V/cell). The ~0.8 % low reading is resistor
tolerance; trim `VDIV_NUM`/`VDIV_DEN` in `main.c` with measured resistor
values if you want it exact.

The link needs external pull-ups on SDA/SCL and a common ground — see the root
README.

TODO: USART1/MSP polling, the enable/disable GPIO, UVLO (hysteresis +
debounce), and the static-selector truth table.
