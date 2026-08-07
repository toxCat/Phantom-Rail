# Transmitter — EdgeTX/OpenTX control scripts

Lua for the radio so an operator can remotely set the **LM51772 output voltage**
and toggle the **Power FET**. Values ride the RC link to Betaflight, which
forwards them to the translator over MSP; the translator writes the LM51772
registers.

```
TX (pot/switch + Lua) ─RC→ Betaflight ─MSP(UART)→ translator ─I2C→ LM51772 sim
        │                                              │
   voltage + FET as channels                    VOUT_TARGET + CONV_EN2
```

EdgeTX (Lua 5.2). Two files:

| File | Type | Role |
|------|------|------|
| `SCRIPTS/MIXES/prvout.lua` | mixer script | drives the **voltage channel** from the pot or a fixed setpoint, snapped to 20 mV register steps |
| `SCRIPTS/TOOLS/prvout.lua` | tool script  | operator UI (encoder/PAGE/RETURN): choose POT/FIXED and set the fixed voltage in 20 mV steps |

Two shared GVARs carry the config from the tool to the mixer (both default to
flight-mode 0): **GV5 = mode** (0 POT / 1 FIXED), **GV6 = fixed voltage as the
register code** (voltage ÷ 20 mV).

## Using the tool (config controls only — no extra switch)

Open **Tools → prvout**. Everything is done with the standard config inputs:

- **rotate encoder** — move between `Mode` and `Set` (or press **PAGE**)
- **press encoder** — start/stop editing the highlighted field
- **rotate while editing** — change it (Voltage steps by **20 mV**, one register code)
- **RETURN** — leave edit, or exit the tool

`Out` shows the live commanded voltage, `FET` the switch state. The mixer keeps
driving the channel from your GVAR/pot even after you close the tool.

Ranges: pot **3.3–24 V** (continuous), fixed setpoint **3.3–20 V** in 20 mV steps
(capped by the GVAR's ±1024 range). The translator re-quantizes to the exact
20 mV register code either way.

## Setup

1. Copy the two files to the SD card under `SCRIPTS/MIXES/` and `SCRIPTS/TOOLS/`.
   Set `POT` / `FETSW` (and `GV_MODE`/`GV_FIXED` if you change them) at the top.
2. **Voltage channel:** add a mix whose source is `LUA prvout Vout`; assign its
   one input to your **pot**.
3. **FET channel:** map your chosen **aux switch** directly to a channel (no Lua)
   — high enables the FET.
4. **FC:** enable **MSP** at **115200** on the UART wired to the translator.

## Troubleshooting the channel mapping (the "only min/max" symptom)

If the voltage only jumps between the extremes (3.3 / 24 V), and sits at **mid
(~13.75 V) with the TX off**, the translator's *voltage* channel is reading a
**2-position switch**, not the pot. (A switch idles at center → mid-scale on
failsafe; a pot would sweep.) The pot is on a **different channel** than the
translator reads.

The translator reads two channels, `MSP_CH_VOLTAGE` and `MSP_CH_FET`
(`translator/Core/Src/main.c`), defaulting to **AUX1 = index 4** and
**AUX2 = index 5**. MSP_RC order is Roll, Pitch, Yaw, Throttle, AUX1, AUX2, … so
**AUXn = index 3 + n**.

Find the right indices, two ways:

- **Betaflight Configurator → Receiver tab:** wiggle the pot and note which bar
  moves (e.g. "AUX 3" → index 6); flip the switch and note its bar. Set
  `MSP_CH_VOLTAGE` / `MSP_CH_FET` to those indices and reflash the translator.
- **Over SWD:** watch the `g_fc_ch[0..7]` array (raw µs). Move the pot → the
  index that sweeps 1000↔2000 is the voltage channel; the one that snaps
  1000/2000 is the switch. `g_fc_nch` shows how many channels the FC returned.

Make sure the pot's mix (or the Lua `Vout`) lands on the channel index you set
for `MSP_CH_VOLTAGE`, and the switch on `MSP_CH_FET`.

### No-TX behaviour

With the FC connected but the **TX off**, Betaflight sends its failsafe channel
values, so the translator sees a valid MSP link and applies them (hence the mid
voltage). If you want TX-loss to disable the output, set the FET aux channel's
**Betaflight failsafe** to low. (A dropped *MSP/FC* link — cable out — instead
trips the translator's own fail-safe and disables after `FC_LINK_TIMEOUT_MS`.)
