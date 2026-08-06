-- Phantom-Rail : LM51772 output-voltage channel (EdgeTX/OpenTX MIXER script)
--
-- Produces the channel value that carries the commanded LM51772 output voltage
-- to the flight controller. Betaflight forwards it over MSP to the translator,
-- which writes VOUT_TARGET (the translator re-quantizes to the exact 20 mV
-- register code, so what you set here is what the IC gets).
--
-- Two inputs:
--   Pot  : manual voltage (a pot/slider source, -1024..1024)
--   Mode : a switch source; > 0 selects FIXED (use the GVAR), else POT
--
-- FIXED setpoint is GVAR GV_FIXED, stored as the LM51772 register CODE
-- (voltage / 20 mV), so it steps in exact datasheet increments. Set it with the
-- companion tool (TOOLS/prvout.lua) or the radio's GVAR screen.
--
-- Install: copy to SCRIPTS/MIXES/prvout.lua, then in the model mixer add a mix
-- on your VOLTAGE channel with source "LUA prvout Vout". Assign the Pot/Mode
-- inputs on the script's mixer line. Put the FET switch on its OWN channel.
-- The translator expects VOLTAGE on AUX1 and FET on AUX2 by default.

local VMIN  = 3300      -- mV, lower clamp (SEL_FB_DIV20=1 -> 3.3 V)
local VMAX  = 24000     -- mV, upper clamp (pot range)
local VSTEP = 20        -- mV, datasheet register step
local GV_FIXED = 5      -- GVAR index (0-based); GV6 holds the fixed code

local inputs  = { { "Pot", SOURCE }, { "Mode", SOURCE } }
local outputs = { "Vout" }

-- map millivolts -> channel value (-1024..1024), consistent with the
-- translator's 1000..2000 us -> VMIN..VMAX mapping.
local function mv_to_chan(mv)
  if mv < VMIN then mv = VMIN end
  if mv > VMAX then mv = VMAX end
  return math.floor((mv - VMIN) * 2048 / (VMAX - VMIN) - 1024)
end

local function run(pot, mode)
  local mv
  if mode > 0 then
    -- FIXED: GVAR holds the register code (voltage / 20 mV)
    mv = model.getGlobalVariable(GV_FIXED, 0) * VSTEP
  else
    -- POT: -1024..1024 -> VMIN..VMAX
    mv = VMIN + (pot + 1024) * (VMAX - VMIN) / 2048
  end
  -- snap to the 20 mV register grid so the transmitter shows true steps
  mv = math.floor((mv + VSTEP / 2) / VSTEP) * VSTEP
  return mv_to_chan(mv)
end

return { input = inputs, output = outputs, run = run }
