-- Phantom-Rail : LM51772 remote control (EdgeTX/OpenTX TOOL script)
--
-- Operator UI for the LM51772 output voltage. Shows the commanded voltage in
-- exact datasheet 20 mV steps (with the register code), the control mode, and
-- the Power-FET switch state. In FIXED mode the roller/+- keys set the voltage
-- in 20 mV steps, stored as the register code in GVAR GV_FIXED -- which the
-- mixer script (MIXES/prvout.lua) turns into the voltage channel.
--
-- Set the source names below to match your radio, then copy to
-- SCRIPTS/TOOLS/prvout.lua and open it from the Tools menu.

local POT      = "S1"     -- manual voltage pot / slider
local MODESW   = "SA"     -- switch: > 0 => FIXED, else POT
local FETSW    = "SB"     -- Power-FET enable switch (also routed to a channel)
local GV_FIXED = 5        -- GVAR index (0-based) shared with the mixer script

local VMIN  = 3300        -- mV
local VMAX  = 24000       -- mV
local VSTEP = 20          -- mV, register step

local function clampCode(c)
  local lo = VMIN / VSTEP
  local hi = VMAX / VSTEP
  if c < lo then c = lo end
  if c > hi then c = hi end
  return c
end

local function run(event)
  local mode  = getValue(MODESW) or 0
  local fixed = mode > 0
  local fet   = (getValue(FETSW) or 0) > 0

  if fixed then
    local c = model.getGlobalVariable(GV_FIXED, 0)
    if     event == EVT_VIRTUAL_INC then model.setGlobalVariable(GV_FIXED, 0, clampCode(c + 1))
    elseif event == EVT_VIRTUAL_DEC then model.setGlobalVariable(GV_FIXED, 0, clampCode(c - 1))
    end
  end

  local mv
  if fixed then
    mv = model.getGlobalVariable(GV_FIXED, 0) * VSTEP
  else
    local pot = getValue(POT) or 0
    mv = VMIN + (pot + 1024) * (VMAX - VMIN) / 2048
  end
  mv = math.floor((mv + VSTEP / 2) / VSTEP) * VSTEP
  local code   = math.floor(mv / VSTEP)
  local vwhole = math.floor(mv / 1000)
  local vcenti = math.floor((mv % 1000) / 10)

  lcd.clear()
  lcd.drawText(1,  1, "LM51772 Remote", INVERS)
  lcd.drawText(1, 12, fixed and "Mode: FIXED" or "Mode: POT")
  lcd.drawText(1, 23, string.format("Vout: %d.%02d V", vwhole, vcenti))
  lcd.drawText(78, 23, string.format("code %d", code))
  lcd.drawText(1, 34, "FET : " .. (fet and "ON" or "OFF"))
  lcd.drawText(1, 45, fixed and "[+/-] 20mV   [RTN] exit"
                             or  "pot sets Vout [RTN] exit")

  if event == EVT_VIRTUAL_EXIT then return 2 end
  return 0
end

return { run = run }
