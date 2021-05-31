-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.values = {
  throttle = 0,
  brake = 0,
  steering = 0,
  clutch = 0,
  wheelspeed = 0,
  avgWheelAV = 0,
  airspeed = 0,
  horn = false
}

M.disabledState = {}

local smoothers = {
  wheelspeed = newExponentialSmoothing(10),
  gear_A = newExponentialSmoothing(10),
  --gear_M = newExponentialSmoothing(10),
  rpm = newExponentialSmoothing(10),
  lights = newExponentialSmoothing(10),
  fuel = newExponentialSmoothing(100),
  oiltemp = newExponentialSmoothing(100),
  watertemp = newExponentialSmoothing(100),
  turnsignal = newExponentialSmoothing(10),
  airspeed = newExponentialSmoothing(10),
  airflowspeed = newExponentialSmoothing(10),
  altitude = newExponentialSmoothing(10)
}

local rpmSmoother = newTemporalSigmoidSmoothing(50000, 75000, 50000, 75000, 0)

local lightsState = 0
local lightsSavedState = 0

local signalRightState = false
local signalLeftState = false
local signalWarnState = false
local lightbarState = 0
local hornState = false

local fogLightsState = false

local blinkPulse = false
local blinkTimerThreshold = 0.4
local blinkTimer = 0
local rpmspin = 0

-- sounds
local hornSound = nil
local sirenSound = nil
local indStartSnd = nil
local indStopSnd = nil
local indLoopSnd1 = nil
local indLoopSnd2 = nil
local lightOn = nil
local lightOff = nil
local hasSteered = false -- used to see whether right/left-turn has been finished

-- set to nop in the beginning - this avoids conflict with the warn signal
local automatic_indicator_stop = nop

local function generateBlinkPulse(dt)
  blinkTimer = blinkTimer + dt
  if blinkTimer > blinkTimerThreshold then
    if signalLeftState or signalRightState then
      if blinkPulse then
        sounds.playSound(indLoopSnd1)
      else
        sounds.playSound(indLoopSnd2)
      end
    end
    blinkPulse = not blinkPulse
    blinkTimer = 0
  end
end

-- stops automatically indicator if turn has been finished or if wheel is steered in opposite direction
local function manage_automatic_indicator_stop()
  local controlPoint = 100
  local steering = M.values.steering
  if steering == nil then
    return
  end

  --check whether user has steered in the desired direction
  if signalLeftState and steering > controlPoint then
    hasSteered = true
  elseif signalRightState and steering < -controlPoint then
    hasSteered = true
  end

  --if the wheel has returned to the neutral position, turn indicator off
  if signalLeftState and hasSteered and steering <= 0 then
    signalLeftState = false
    hasSteered = false
    sounds.playSound(indStopSnd)
    automatic_indicator_stop = nop
  elseif signalRightState and hasSteered and steering >= 0 then
    signalRightState = false
    hasSteered = false
    sounds.playSound(indStopSnd)
    automatic_indicator_stop = nop
  end
end

-- user input functions
local function toggle_left_signal()
  if not signalWarnState then
    signalLeftState = not signalLeftState
  else
    signalLeftState = true
  end
  if signalLeftState then
    signalRightState = false
    signalWarnState = false
    sounds.playSound(indStartSnd)
    automatic_indicator_stop = manage_automatic_indicator_stop
  end
  if not signalLeftState then
    sounds.playSound(indStopSnd)
    automatic_indicator_stop = nop
  end
end

local function toggle_right_signal()
  if not signalWarnState then
    signalRightState = not signalRightState
  else
    signalRightState = true
  end
  if signalRightState then
    signalLeftState = false
    signalWarnState = false
    sounds.playSound(indStartSnd)
    automatic_indicator_stop = manage_automatic_indicator_stop
  end
  if not signalRightState then
    automatic_indicator_stop = nop
    sounds.playSound(indStopSnd)
  end
end

local function toggleSound(val, snd)
  if not snd then
    return
  end
  if val then
    obj:setVolume(snd, 1)
    obj:cutSFX(snd)
    obj:playSFX(snd)
  else
    obj:stopSFX(snd)
  end
end

local function update(dt)
  generateBlinkPulse(dt)

  local vals = M.values
  -- the primary source values

  automatic_indicator_stop()

  rpmspin = rpmspin + (dt * (vals.rpm or 0))
  if rpmspin > 360 then
    rpmspin = rpmspin - 360
  end
  vals.rpmspin = rpmspin

  vals.parkingbrake = vals.parkingbrake_input
  vals.lights = lightsState
  vals.lights_state = lightsState
  if signalWarnState then
    vals.turnsignal = 0
  elseif signalRightState then
    vals.turnsignal = 1
  elseif signalLeftState then
    vals.turnsignal = -1
  else
    vals.turnsignal = 0
  end

  vals.airspeed = obj:getGroundSpeed()
  vals.airflowspeed = obj:getAirflowSpeed()
  vals.altitude = obj:getAltitude()
  vals.parking = 0 -- TODO: input.parkinglights
  vals.reverse = (vals.gearIndex or 0) < 0

  -- and then the derived values
  vals.signal_L = (vals.signal_left_input == 1 and blinkPulse)
  vals.signal_R = (vals.signal_right_input == 1 and blinkPulse)
  vals.signal_right_input = (signalRightState)
  vals.signal_left_input = (signalLeftState)
  vals.hazard = (signalWarnState and blinkPulse)
  vals.hazard_enabled = signalWarnState
  vals.lightbar = lightbarState
  vals.lowpressure = (beamstate.lowpressure)
  vals.oil = (vals.oiltemp or 0) >= 130
  vals.lowhighbeam = (lightsState == 1 or lightsState == 2)
  vals.lowbeam = (lightsState == 1)
  vals.highbeam = (lightsState == 2)
  vals.fog = fogLightsState
  vals.horn = hornState

  vals.rpmTacho = rpmSmoother:get(vals.rpm or 0, dt)

  -- inject imported electrics events
  beamstate.updateRemoteElectrics()

  for f, v in pairs(vals) do
    if M.disabledState[f] ~= nil then
      vals[f] = nil
    else
      if type(v) == "boolean" then
        vals[f] = vals[f] and 1 or 0
      end
    end
  end

  for f, s in pairs(smoothers) do
    if vals[f] ~= nil then
      vals[f] = s:get(vals[f])
    end
  end

  --obj:setVolume(indicatorLoopSound, math.max(vals.signal_L, vals.signal_R))
end

local function reset()
  M.disabledState = {}

  for _, s in pairs(smoothers) do
    s:set(0)
  end

  M.values.throttle = 0
  M.values.brake = 0
  M.values.steering = 0
  M.values.clutch = 0
  M.values.wheelspeed = 0
  M.values.avgWheelAV = 0
  M.values.airspeed = 0
  M.values.airflowspeed = 0
  M.values.horn = false
  lightbarState = 0
  lightsState = 0
  lightsSavedState = 0
end

local function init()
  reset()

  hornSound = hornSound or sounds.createSoundscapeSound("horn")
  sirenSound = sirenSound or sounds.createSoundscapeSound("siren")
  indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
  indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  indLoopSnd1 = indLoopSnd1 or sounds.createSoundscapeSound("indLoop1")
  indLoopSnd2 = indLoopSnd2 or sounds.createSoundscapeSound("indLoop2")
  lightOn = lightOn or sounds.createSoundscapeSound("LightOn")
  lightOff = lightOff or sounds.createSoundscapeSound("LightOff")
end

local function set_warn_signal(value)
  signalWarnState = value
  signalRightState = signalWarnState
  signalLeftState = signalWarnState
end

local function toggle_warn_signal()
  set_warn_signal(not signalWarnState)
end

--Modified code
local function toggle_lights()
  lightsState = lightsState + 1
  if lightsState == 1 then
    sounds.playSound(lightOn)
  elseif lightsState == 2 then
    sounds.playSound(lightOn)
    obj:queueGameEngineLua("scripts_driver__assistance__angelo234_extension.onHeadlightsOn()")
  elseif lightsState == 3 then
    lightsState = 0
    sounds.playSound(lightOff)
    obj:queueGameEngineLua("scripts_driver__assistance__angelo234_extension.onHeadlightsOff()")
  end
end

local function light_flash_highbeams(enabled)
  if enabled then
    lightsSavedState = lightsState
    lightsState = 2
  else
    lightsState = lightsSavedState
  end
end

local function set_lightbar_signal(state)
  if state == lightbarState then return end
  lightbarState = state % (sirenSound ~= nil and 3 or 2)

  -- 1 = lights, no sound
  -- 2 = lights + sound
  toggleSound(lightbarState == 2, sirenSound)
end

local function toggle_lightbar_signal()
  set_lightbar_signal(lightbarState + 1)
end

local function setLightsState(newval)
  lightsState = newval
end

local function toggle_fog_lights()
  fogLightsState = not fogLightsState
end

local function set_fog_lights(state)
  fogLightsState = state
end

local function horn(state)
  hornState = state
  -- we do it here, as we do not want to do 'change' detection in the electrics.
  -- the electrics can only read the state, not set it
  toggleSound(hornState, hornSound)
end

-- public interface
M.update = update
M.toggle_left_signal = toggle_left_signal
M.toggle_right_signal = toggle_right_signal
M.toggle_warn_signal = toggle_warn_signal
M.set_warn_signal = set_warn_signal
M.toggle_lightbar_signal = toggle_lightbar_signal
M.set_lightbar_signal = set_lightbar_signal
M.toggle_fog_lights = toggle_fog_lights
M.set_fog_lights = set_fog_lights
M.toggle_lights = toggle_lights
M.light_flash_highbeams = light_flash_highbeams
M.setLightsState = setLightsState
M.horn = horn
M.reset = reset
M.init = init
return M
