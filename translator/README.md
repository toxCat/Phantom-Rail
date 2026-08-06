# Translator — Black Pill #1 (I2C1 host / master)

FC-facing board and the **host controller** for the LM51772 sim
(`../lm51772-sim`): it reads the source voltage, runs pack detection + the sag
monitor, drives the Power FET, and controls the sim exactly as a flight
controller would drive a real LM51772 — writing its control registers and
reading status over **I2C1** (`../Protocol/lm51772_regs.h`, addr 0x6A).

Register-level CMSIS, no HAL. Builds independently: `make` → `build/translator.{elf,hex,bin}`.

## Pin map (target)

| Signal            | Pin        | Function |
|-------------------|------------|----------|
| Source sense      | PA0        | ADC1_IN0, via 120k/15k divider (/9, ~29 V full scale) |
| FC UART           | PA9 / PA10 | USART1 TX/RX ↔ Betaflight (MSP @ 115200) |
| Sim link          | PB6 / PB7  | **I2C1 master** → Black Pill #2 (LM51772 @ 0x6A) |
| Power FET gate    | PA1        | N-channel Power FET enable, active-high |
| Status LED        | PC13       | on-board LED (active low) |

## Status

Implemented:
- **ADC1_IN0 source-sense reader** — PA0, 12-bit, 480-cycle sample (high-Z
  120k divider), scaled by 135/15 (=/9) into `g_source_mv` (millivolts), also
  a `volatile` global you can watch over SWD.
- **Pack detection** — LiPo cell count via the Betaflight convention (smallest
  S whose 4.3 V/cell ceiling covers the reading), in `g_cells`.
- **FC link / MSP master** (`Core/Src/usart1.c`, `msp.c`) — USART1 on PA9/PA10,
  8N1 @ 115200. Each cycle it polls Betaflight with **MSP_RC** (105) and reads
  the RC channels that carry the transmitter's pot/switch: **AUX1** (index 4) →
  output voltage (1000–2000 µs mapped to `VOUT_MIN_MV`–`VOUT_MAX_MV`, 3.3–24 V),
  **AUX2** (index 5) → Power-FET enable (`≥ FET_ON_US`). If no valid MSP for
  `FC_LINK_TIMEOUT_MS` it fails safe (disable). `g_vout_cmd_mv` / `g_fc_enable`
  / `g_fc_link` over SWD. (Channel indices are `#define`s — align with your TX.)
- **LM51772 host** (`Core/Src/i2c1_master.c`) — register-addressed I2C1 master,
  100 kHz on PB6/PB7. Each cycle it writes the sim's registers: **VOUT_TARGET**
  (0x0C/0x0D, from the FC pot, ×20 mV code), **ILIM_THRESHOLD** (0x0A,
  `CUR_LIMIT_MA` = 2 A), **CONV_EN2** (0x81, from the FC switch), plus the
  battery view into extension registers 0xE0–0xE3; then it reads
  **USB_PD_STATUS_0** (0x21). `i2c1_master_read_reg()` does the write-pointer +
  repeated-start read; a failed transfer calls `i2c1_master_recover()` (SWRST).
- **Power FET + battery veto** — N-channel gate on **PA1**, active-high, starts
  low (fail-safe). The FC now commands enable; the battery only **vetoes** —
  cell count latched at plug-in, and a **critical** pack (`<3.2 V/cell`) forces
  the output off regardless of the transmitter. `3.2–3.6 V` still shows the LOW
  warning. `g_fet_on` / `g_cell_mv` over SWD.
- **Over-current soft-fail (via the IC model)** — the sim compares its ACS709
  reading to the ILIM the host programmed and raises **CC_OPERATION**. The host
  reads it back and, when it stays set ≥ **`OC_DEBOUNCE_MS`** (1 s), latches the
  FET off — the sustained-fault backstop (the LM51772 does fast limiting
  itself). On pack removal it re-arms and writes **CLEAR_FAULTS** (0x03) to the
  sim. `g_oc_active` / `g_oc_fault` / `g_pd_status` over SWD.
- **PC13 heartbeat**.

Bench check: a 22.94 V 6S pack read 2.528 V at PA0 → 22.75 V computed → `6S`
detected (22.75/6 = 3.79 V/cell). The ~0.8 % low reading is resistor
tolerance; trim `VDIV_NUM`/`VDIV_DEN` in `main.c` with measured resistor
values if you want it exact.

The I2C link needs external pull-ups on SDA/SCL and a common ground — see the
root README.

## FC (Betaflight) hookup

- Wire **translator PA9 (TX) → FC UART RX**, **PA10 (RX) → FC UART TX**, common
  ground. (3V3 logic on both.)
- In Betaflight Configurator → **Ports**, enable **MSP** on that UART at
  **115200**. Nothing else on the port.
- On the transmitter (Task 3), map the voltage pot to **AUX1** and the FET
  switch to **AUX2** (or change `MSP_CH_VOLTAGE` / `MSP_CH_FET` in `main.c` to
  match your channel layout).

Note: the Power FET now stays **off until the FC commands enable** (AUX2 high) —
without a connected/ armed FC the output is disabled by design (fail-safe).

TODO (Task 3): the transmitter-side Lua script (voltage steps per the datasheet
registers + pot / switch bindings).
