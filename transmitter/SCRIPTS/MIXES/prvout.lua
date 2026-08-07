-- Phantom-Rail : LM51772 output-voltage channel (EdgeTX/OpenTX MIXER script)
--
-- Produces the channel value that carries the commanded LM51772 output voltage
-- to the flight controller. Betaflight forwards it over MSP to the translator,
-- which writes VOUT_TARGET (and re-quantizes to the exact 20 mV register code).
--
-- Mode and the fixed setpoint are chosen ON-SCREEN with the companion tool
-- (TOOLS/prvout.lua) using the encoder + RTN -- no physical mode switch. This
-- script just reads the two shared GVARs and the pot:
--   GV_MODE  (GV5): 0 = POT (manual), non-zero = FIXED (use GV_FIXED)
--   GV_FIXED (GV6): fixed voltage as the register code (voltage / 20 mV)
--
-- Install: copy to SCRIPTS/MIXES/prvout.lua, then on your VOLTAGE channel add a
-- mix with source "LUA prvout Vout" and assign its one input to the pot. Put
-- the FET switch on its OWN channel. Defaults: VOLTAGE on AUX1, FET on AUX2.

local VMIN  = 3300     -- mV, channel-map lower end (matches the translator)
local VMAX  = 24000    -- mV, channel-map upper end (pot reaches this)
local VSTEP = 20       -- mV, datasheet register step
local GV_MODE  = 4     -- GV5
local GV_FIXED = 5     -- GV6

local inputs  = { { "Pot", SOURCE } }
local outputs = { "Vout" }

local function run(pot)
  local mv
  if model.getGlobalVariable(GV_MODE, 0) ~= 0 then
    mv = model.getGlobalVariable(GV_FIXED, 0) * VSTEP     -- FIXED: code -> mV
  else
    mv = VMIN + (pot + 1024) * (VMAX - VMIN) / 2048        -- POT: -1024..1024
  end
  if mv < VMIN then mv = VMIN end
  if mv > VMAX then mv = VMAX end
  mv = math.floor((mv + VSTEP / 2) / VSTEP) * VSTEP         -- snap to 20 mV grid
  return math.floor((mv - VMIN) * 2048 / (VMAX - VMIN) - 1024)
end

return { input = inputs, output = outputs, run = run }
