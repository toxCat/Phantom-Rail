-- Phantom-Rail : LM51772 remote control (EdgeTX/OpenTX TOOL script)
--
-- Operator UI for the LM51772 output. Driven ENTIRELY by the radio's config
-- controls -- encoder wheel (rotate + press), PAGE, RETURN -- no physical mode
-- switch needed:
--   * rotate encoder  -> move between fields (Mode / Voltage), or PAGE
--   * press encoder   -> start/stop editing the highlighted field
--   * while editing, rotate -> change it (Voltage steps 20 mV, a real register step)
--   * RETURN          -> leave edit, or exit the tool
--
-- It writes two shared GVARs that the mixer script (MIXES/prvout.lua) reads:
--   GV_MODE  (GV5): 0 = POT (manual), 1 = FIXED
--   GV_FIXED (GV6): fixed voltage as register code (voltage / 20 mV)
--
-- Set the pot/FET source names to match your radio, and open from Tools.

local POT      = "S1"     -- manual voltage pot / slider (display only here)
local FETSW    = "SB"     -- Power-FET enable switch     (display only here)
local GV_MODE  = 4        -- GV5
local GV_FIXED = 5        -- GV6

local VMIN  = 3300        -- mV
local VMAX  = 24000       -- mV, pot range
local VSTEP = 20          -- mV, register step
local CMIN  = VMIN / VSTEP           -- 165  (3.30 V)
local CMAX  = 20000 / VSTEP          -- 1000 (20.00 V) -- fixed cap: GVAR range

local sel  = 1            -- 1 = Mode, 2 = Voltage
local edit = false

local function fmtV(mv)
  return string.format("%d.%02dV", math.floor(mv / 1000), math.floor((mv % 1000) / 10))
end

local function run(event)
  local inc  = (event == EVT_VIRTUAL_INC) or (event == EVT_VIRTUAL_NEXT)
  local dec  = (event == EVT_VIRTUAL_DEC) or (event == EVT_VIRTUAL_PREV)
  local page = (event == EVT_VIRTUAL_NEXT_PAGE) or (event == EVT_VIRTUAL_PREV_PAGE)
  local ent  = (event == EVT_VIRTUAL_ENTER)
  local quit = (event == EVT_VIRTUAL_EXIT)

  local mode = model.getGlobalVariable(GV_MODE, 0)
  local code = model.getGlobalVariable(GV_FIXED, 0)

  if edit then
    if sel == 1 then                              -- Mode: toggle
      if inc or dec then mode = (mode ~= 0) and 0 or 1 end
    else                                          -- Voltage: 20 mV steps
      if inc then code = code + 1 end
      if dec then code = code - 1 end
    end
    if code < CMIN then code = CMIN end
    if code > CMAX then code = CMAX end
    model.setGlobalVariable(GV_MODE, 0, mode)
    model.setGlobalVariable(GV_FIXED, 0, code)
    if ent or quit then edit = false end
  else
    if inc or page then sel = sel + 1 end
    if dec        then sel = sel - 1 end
    if sel < 1 then sel = 2 end
    if sel > 2 then sel = 1 end
    if ent  then edit = true end
    if quit then return 2 end                     -- exit the tool
  end

  -- live values for display
  mode = model.getGlobalVariable(GV_MODE, 0)
  code = model.getGlobalVariable(GV_FIXED, 0)
  local fixedMv = code * VSTEP
  local potMv   = VMIN + ((getValue(POT) or 0) + 1024) * (VMAX - VMIN) / 2048
  potMv = math.floor((potMv + VSTEP / 2) / VSTEP) * VSTEP
  local outMv = (mode ~= 0) and fixedMv or potMv
  local fet   = (getValue(FETSW) or 0) > 0

  local function fl(i) return (sel == i) and (edit and (INVERS + BLINK) or INVERS) or 0 end

  lcd.clear()
  lcd.drawText(1,  1, "LM51772 Remote", INVERS)
  lcd.drawText(1, 14, "Mode")
  lcd.drawText(46, 14, (mode ~= 0) and "FIXED" or "POT", fl(1))
  lcd.drawText(1, 26, "Set")
  lcd.drawText(46, 26, fmtV(fixedMv), fl(2))
  lcd.drawText(1, 40, "Out  " .. fmtV(outMv))
  lcd.drawText(1, 52, "FET  " .. (fet and "ON" or "OFF"))
  return 0
end

return { run = run }
