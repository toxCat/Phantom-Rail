# Transmitter — EdgeTX/OpenTX control scripts

Lua for the radio, so an operator can remotely set the **LM51772 output voltage**
and toggle the **Power FET** from the sticks/pots/switches. The values ride the
normal RC link to Betaflight, which forwards them to the translator over MSP;
the translator writes the LM51772 registers.

```
TX (pot/switch + Lua) ─RC→ Betaflight ─MSP(UART)→ translator ─I2C→ LM51772 sim
        │                                              │
   voltage + FET as channels                    VOUT_TARGET + CONV_EN2
```

Written for **EdgeTX** (Lua 5.2). Two files:

| File | Type | Role |
|------|------|------|
| `SCRIPTS/MIXES/prvout.lua` | mixer script | drives the **voltage channel** from the pot or a fixed setpoint, snapped to the datasheet **20 mV** register steps |
| `SCRIPTS/TOOLS/prvout.lua` | tool script  | operator UI: shows commanded V + register code + FET state, and sets the FIXED voltage in 20 mV steps |

## The channel contract (this is what fixes the min/max-only symptom)

The translator reads two MSP RC channels (`MSP_CH_VOLTAGE` / `MSP_CH_FET` in the
translator `main.c`), defaulting to **AUX1 (voltage)** and **AUX2 (FET)** —
MSP_RC order is Roll,Pitch,Yaw,Throttle,AUX1,AUX2,… so indices **4** and **5**.

> If the voltage only jumps between the min and max (3.3 V / 24 V) with no
> gradient, the **pot and switch are swapped**: the voltage channel is reading
> the 2-position switch. Put the **pot/voltage source on AUX1** and the **FET
> switch on AUX2** (or swap the two `MSP_CH_*` indices to match your layout).

Voltage range is **3.3–24 V** on both sides (translator `VOUT_MIN_MV`/`VOUT_MAX_MV`
= the script's `VMIN`/`VMAX`). The translator re-quantizes to the exact 20 mV
register code, so whatever the channel carries lands on a real datasheet step.

## Setup

1. **Copy** the two files to the SD card under `SCRIPTS/MIXES/` and
   `SCRIPTS/TOOLS/`. Edit the `POT` / `MODESW` / `FETSW` / `GV_FIXED` constants
   at the top of each to match your radio (they must agree between the two).
2. **Mixer — voltage channel (AUX1):** add a mix whose source is
   `LUA prvout Vout`; assign its two inputs to your **pot** and a **mode switch**
   (switch high = FIXED setpoint, low = live pot).
3. **Mixer — FET channel (AUX2):** map your chosen **aux switch** directly to
   that channel (no Lua needed) — high ≥ ~1700 µs enables the FET.
4. **GVAR:** the FIXED setpoint lives in `GV_FIXED` (default GV6) as the register
   code (voltage ÷ 20 mV). Set it from the **tool** (Tools menu → prvout) with
   the roller/+- keys, or the radio's GVAR screen.
5. **FC side:** enable **MSP** at **115200** on the UART wired to the translator
   (see `../translator/README.md`).

## Using it

- **Manual:** mode switch to POT, sweep the pot → the sim's row-1 output voltage
  tracks 3.3–24 V in 20 mV steps.
- **Fixed:** mode switch to FIXED, open the tool, dial the voltage in 20 mV
  steps (shows volts + register code). The channel holds that value.
- **FET:** flip the aux switch → the sim status goes `OFF` ↔ `ON` and the Power
  FET follows (subject to the translator's battery/over-current safety).

Note: firmware safety still applies on top of these commands — a critically low
battery or a sustained over-current will cut the FET regardless of the switch.
