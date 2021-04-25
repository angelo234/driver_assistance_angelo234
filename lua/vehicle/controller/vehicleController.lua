-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
--Mandatory controller parameters
M.type = "main"
M.relevantDevice = nil
M.defaultOrder = 500

M.fireEngineTemperature = 0
M.throttle = 0
M.brake = 0
M.clutchRatio = 0
-----

local settings = require("simplesettings")

local min = math.min
local max = math.max
local abs = math.abs
local floor = math.floor

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local gearboxHandling = {
  behaviors = {"arcade", "realistic"},
  behaviorLookup = nil,
  behavior = nil,
  previousBehavior = nil,
  logic = nil,
  autoClutch = true,
  autoThrottle = true,
  gearboxSafety = true,
  availableLogic = {},
  arcadeAutoBrakeAmount = 0.2,
  arcadeAutoBrakeAVThreshold = 5,
  isArcadeSwitched = nil,
  useSmartAggressionCalculation = true
}

local energyStorageData = {
  ratio = 0,
  volume = 0,
  capacity = 0,
  invEnergyStorageCount = 0
}

local shiftPreventionData = {
  wheelSlipUpThreshold = 0,
  wheelSlipDownThreshold = 0,
  wheelSlipShiftDown = false,
  wheelSlipShiftUp = false
}

local shiftBehavior = {
  shiftUpAV = 0,
  shiftDownAV = 0
}

local smoother = {
  throttle = nil,
  brake = nil,
  throttleInput = nil,
  brakeInput = nil,
  aggression = nil,
  avgAV = nil,
  wheelSlipShiftUp = nil,
  groundContactSmoother = nil
}

--Used for decision logic, smoothed throttle is not actually used as the throttle value
local smoothedValues = {
  throttle = 0,
  brake = 0,
  throttleInput = 0,
  brakeInput = 0,
  drivingAggression = 0,
  throttleUpShiftThreshold = 0.05,
  avgAV = 0
}

M.engineInfo = {
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  "manual",
  obj:getID(),
  0,
  0,
  1,
  0,
  0,
  0
}

local engine = nil
local gearbox = nil
local gearboxType = nil

local timerConstants = {gearChangeDelay = 0, shiftDelay = 0, neutralSelectionDelay = 0, aggressionHoldOffThrottleDelay = 0}
local timer = {
  gearChangeDelayTimer = 0,
  shiftDelayTimer = 0,
  neutralSelectionDelayTimer = 0,
  stalledEngineMessageTimer = 0,
  stalledEngineTryingToStartTimer = 0,
  aggressionHoldOffThrottleTimer = 0
}

local handBrakeHandling = {
  smartParkingBrakeActive = false,
  smartParkingBrakeSlip = 0
}

local shiftPoints = {}
local currentGearIndex = 0
local lastAggressionThrottle = 0
local aggressionOverride
local topSpeedLimit = 0
local topSpeedLimitReverse = 0
local topSpeedLimitPID
local rpmLedsEnabled = false

local inputValues = {throttle = 0, clutch = 0}

local isFrozen = false

local controlLogicModule = nil

local function setAggressionOverride(aggression)
  aggressionOverride = aggression
end

local function getGearboxBehaviorName()
  if gearboxHandling.behavior == "arcade" then
    return "vehicle.drivetrain.shifterModeNameArcade"
  elseif gearboxHandling.behavior == "realistic" then
    return "vehicle.drivetrain.shifterModeNameRealistic"
  end

  return "vehicle.drivetrain.shifterModeNameUnknown"
end

local function setGearboxBehavior(behavior)
  if not gearboxHandling.behaviorLookup[behavior] then
    log("E", "vehicleController.setGearboxBehavior", "Unknown gearbox behavior: " .. (behavior or "nil"))
    return
  end

  gearboxHandling.behavior = behavior
  gui.message({txt = "vehicle.drivetrain.shifterModeChanged", context = {shifterModeName = getGearboxBehaviorName()}}, 2, "vehicle.shiftermode")
  gearboxHandling.previousBehavior = gearboxHandling.behavior

  controlLogicModule.gearboxBehaviorChanged(behavior)
end

--------------------------------------
--------------- Shared ---------------
--------------------------------------

local sharedFunctions = {}

sharedFunctions.selectShiftPoints = function(gearIndex)
  --interpolate based on aggression between high/low ranges
  local aggression = min(max((smoothedValues.drivingAggression - 0.2) / 0.8, 0), 1)
  local aggressionCoef = min(max(aggression * aggression, 0), 1)
  local shiftPoint = shiftPoints[gearIndex]
  shiftBehavior.shiftDownAV = shiftPoint.lowShiftDownAV + (shiftPoint.highShiftDownAV - shiftPoint.lowShiftDownAV) * aggressionCoef
  shiftBehavior.shiftUpAV = shiftPoint.lowShiftUpAV + (shiftPoint.highShiftUpAV - shiftPoint.lowShiftUpAV) * aggressionCoef
  controlLogicModule.shiftBehavior = shiftBehavior
end

sharedFunctions.getShiftPoints = function()
  return shiftPoints or {}
end

sharedFunctions.switchToRealisticBehavior = function(gearIndex)
  gui.message({txt = "vehicle.drivetrain.usingHshifter"}, 2, "vehicle.shiftermode")
  setGearboxBehavior("realistic")
  controlLogicModule.shiftToGearIndex(gearIndex)
end

sharedFunctions.warnCannotShiftSequential = function()
  gui.message({txt = "vehicle.drivetrain.cannotShiftSequential", context = {shifterModeName = getGearboxBehaviorName()}}, 2, "vehicle.shiftLogic.cannotShift")
end

sharedFunctions.updateAvgAVSingleDevice = function(deviceName, deviceProperty)
  deviceProperty = deviceProperty or "outputAV1"
  local device = powertrain.getDevice(deviceName)
  return (device and device[deviceProperty]) and device[deviceProperty] or 0
end

sharedFunctions.updateAvgAVDeviceType = function(deviceType)
  local avSum = 0
  local avCount = 0
  for _, v in ipairs(powertrain.getDevicesByType(deviceType)) do
    avSum = avSum + v.outputAV1
    avCount = avCount + 1
  end
  return avSum / avCount
end

sharedFunctions.updateAvgAVDeviceCategory = function(deviceCategory)
  local avSum = 0
  local avCount = 0
  for _, v in ipairs(powertrain.getDevicesByCategory(deviceCategory)) do
    avSum = avSum + v.outputAV1
    avCount = avCount + 1
  end
  return avSum / avCount
end

sharedFunctions.getEnergyStorages = function(engines)
  local storages = {}
  for _, v in ipairs(engines) do
    for _, w in ipairs(v.energyStorage or {}) do
      local energyStorage = energyStorage.getStorage(w)
      if energyStorage and energyStorage.energyType == v.requiredEnergyType then
        table.insert(storages, w)
      end
    end
  end
  return storages
end

--------------------------------------
--------------------------------------
--------------------------------------

local function handleStalling(dt)
  if not engine then
    return
  end

  timer.stalledEngineMessageTimer = max(timer.stalledEngineMessageTimer - dt, 0)

  if engine.isStalled and not engine.isDisabled and engine.ignitionCoef > 0 and engine.starterEngagedCoef <= 0 then
    if not engine.starterDisabled then
      if gearboxHandling.behavior == "arcade" and M.throttle > 0 then
        engine:activateStarter()
        gui.message({txt = "vehicle.drivetrain.stalledStarting"}, 2, "vehicle.engine.isStalling")
        timer.stalledEngineMessageTimer = 1.8
      elseif timer.stalledEngineMessageTimer <= 0 then
        local message
        if gearboxHandling.autoClutch then
          message = "vehicle.drivetrain.stalledAutoClutch"
        else
          message = "vehicle.drivetrain.stalled"
        end
        gui.message({txt = message}, 2, "vehicle.engine.isStalling")
        timer.stalledEngineMessageTimer = 1.8
      end
    end
  end
end

local function updateWheelSlip(dt)
  local overallWheelSlip = 0
  local wheelSlipCount = 0
  local hasGroundContact = false
  for _, wi in ipairs(wheels.wheels) do
    if wi.isPropulsed and not wi.isBroken then
      if wi.contactMaterialID1 >= 0 and wi.contactDepth == 0 then
        overallWheelSlip = overallWheelSlip + wi.slipEnergy
        wheelSlipCount = wheelSlipCount + 1
      end
      hasGroundContact = wi.contactMaterialID1 >= 0 or hasGroundContact
    end

    handBrakeHandling.smartParkingBrakeSlip = handBrakeHandling.smartParkingBrakeSlip + wi.slipEnergy
  end
  local groundContactCoef = smoother.groundContactSmoother:getUncapped(hasGroundContact and 1 or 0, dt)

  overallWheelSlip = smoother.wheelSlipShiftUp:get(overallWheelSlip, dt)
  local averagePropulsedWheelSlip = wheelSlipCount > 0 and overallWheelSlip / wheelSlipCount or 0
  handBrakeHandling.smartParkingBrakeSlip = handBrakeHandling.smartParkingBrakeSlip / wheels.wheelCount

  shiftPreventionData.wheelSlipShiftDown = true
  if (averagePropulsedWheelSlip > shiftPreventionData.wheelSlipDownThreshold or groundContactCoef < 1) then -- and M.throttle <= 0.5 then
    shiftPreventionData.wheelSlipShiftDown = false
  end
  shiftPreventionData.wheelSlipShiftUp = groundContactCoef >= 1 and averagePropulsedWheelSlip < shiftPreventionData.wheelSlipUpThreshold

  controlLogicModule.shiftPreventionData = shiftPreventionData

  --print(string.format("%d / %d slip, groundcontact: %s -> canShiftDown: %s", averagePropulsedWheelSlip, shiftPreventionData.wheelSlipDownThreshold, groundContactCoef, shiftPreventionData.wheelSlipShiftDown))
end

local function updateAggression(dt)
  local throttle = (gearboxHandling.isArcadeSwitched and inputValues.brake or inputValues.throttle) or 0 --read our actual throttle input value, depending on which input is currently used for throttle

  if gearboxHandling.useSmartAggressionCalculation then --use the new smart aggression logic for newer cars and all manuals
    if throttle <= 0 then
      timer.aggressionHoldOffThrottleTimer = max(timer.aggressionHoldOffThrottleTimer - dt, 0)
    else
      timer.aggressionHoldOffThrottleTimer = timerConstants.aggressionHoldOffThrottleDelay
    end

    local usesKeyboard = input.state.throttle.filter == FILTER_KBD or input.state.throttle.filter == FILTER_KBD2
    local brakeUse = M.brake > 0.25
    local aggression
    local sportModeAdjust = (controlLogicModule.isSportModeActive and 0.5 or 0)

    if usesKeyboard then
      aggression = brakeUse and smoothedValues.drivingAggression or throttle * 1.333
      aggression = electrics.values.wheelspeed < 1 and 0.75 or aggression
      aggression = smoother.aggressionKey:get(max(aggression, sportModeAdjust), dt)
      smoother.aggressionAxis:set(aggression) --keep the other smoother in sync
    else
      local throttleHold = throttle <= 0 and electrics.values.wheelspeed > 2 and (timer.aggressionHoldOffThrottleTimer > 0 or controlLogicModule.isSportModeActive)
      local holdAggression = brakeUse or throttleHold
      local dThrottle = min(max((throttle - lastAggressionThrottle) / dt, 1), 20)
      aggression = holdAggression and smoothedValues.drivingAggression or throttle * 1.333 * dThrottle
      aggression = electrics.values.wheelspeed < 1 and 0.75 or aggression
      aggression = smoother.aggressionAxis:get(max(aggression, sportModeAdjust), dt)
      smoother.aggressionKey:set(aggression) --keep the other smoother in sync
    end
    smoothedValues.drivingAggression = min(aggression, 1) --previous smoother outputs max out at 1.333 to give some headroom, but now we cap them to 1 for the rest of the code
  else --use old logic for old manuals
    smoothedValues.drivingAggression = throttle * throttle * throttle
  end

  smoothedValues.drivingAggression = aggressionOverride or smoothedValues.drivingAggression

  lastAggressionThrottle = throttle
end

-- will smartly decide whether the user is actually parking the car (toggle), or just drifting around (temporary brake)
local function smartParkingBrake(ivalue, filter)
  local speed = electrics.values.wheelspeed
  if not speed then -- not a typical car, so just set the pbrake as instructed
    input.event("parkingbrake", ivalue, filter)
  end

  -- are we sliding or rolling?
  local isAxis = filter == FILTER_DIRECT or filter == FILTER_PAD
  local rolling = abs(speed) > 2.8
  -- ~10km/h
  local skidding = handBrakeHandling.smartParkingBrakeSlip > 10000

  -- decide what to do, based on context
  if rolling or skidding or isAxis then
    input.event("parkingbrake", ivalue, filter) -- transparent use / temporary pbrake
  elseif ivalue > 0.5 then -- car is parked, use onDown to toggle pbrake
    input.toggleEvent("parkingbrake")
  elseif input.state.parkingbrake.val > 0.5 then
    handBrakeHandling.smartParkingBrakeActive = true
  end
end

local function updateGFXGeneric(dt)
  inputValues.throttle = electrics.values.throttleOverride or min(max(input.throttle or 0, 0), 1)
  --Only change in file
  inputValues.brake = electrics.values.brakeOverride or min(max(input.brake or 0, 0), 1)
  inputValues.clutch = min(max(input.clutch or 0, 0), 1)

  --read avg AV from the device specified by the shiftlogic module
  local avgAV = controlLogicModule.smoothedAvgAVInput or 0
  smoothedValues.avgAV = smoother.avgAV:get(avgAV, dt)

  timer.gearChangeDelayTimer = max(timer.gearChangeDelayTimer - dt, 0)
  timer.shiftDelayTimer = max(timer.shiftDelayTimer - dt, 0)
  timer.neutralSelectionDelayTimer = max(timer.neutralSelectionDelayTimer - dt, 0)

  updateWheelSlip(dt)

  controlLogicModule.gearboxHandling = gearboxHandling
  controlLogicModule.timer = timer
  controlLogicModule.timerConstants = timerConstants
  controlLogicModule.inputValues = inputValues
  controlLogicModule.smoothedValues = smoothedValues

  controlLogicModule.updateGearboxGFX(dt)

  M.throttle = controlLogicModule.throttle
  M.brake = controlLogicModule.brake
  M.clutchRatio = controlLogicModule.clutchRatio
  gearboxHandling.isArcadeSwitched = controlLogicModule.isArcadeSwitched
  currentGearIndex = controlLogicModule.currentGearIndex
  local gearName = controlLogicModule.getGearName()
  local gearPosition = controlLogicModule.getGearPosition()

  handleStalling(dt)

  local vehicleSpeed = electrics.values.wheelspeed or 0
  local speedLimit = (type(gearName) == "string" and gearName:sub(1, 1) == "R") and topSpeedLimitReverse or topSpeedLimit
  if speedLimit > 0 then
    local throttleCoef = 1 - topSpeedLimitPID:get(-vehicleSpeed, -speedLimit, dt)
    M.throttle = M.throttle * throttleCoef
  end

  smoothedValues.throttle = smoother.throttle:getUncapped(M.throttle, dt)
  smoothedValues.brake = smoother.brake:getUncapped(M.brake, dt)
  smoothedValues.throttleInput = smoother.throttleInput:getUncapped(inputValues.throttle, dt)
  smoothedValues.brakeInput = smoother.brakeInput:getUncapped(inputValues.brake, dt)

  updateAggression(dt)

  M.fireEngineTemperature = (engine and engine.thermals) and engine.thermals.exhaustTemperature or obj:getEnvTemperature() --TODO

  if isFrozen then
    M.brake = 1
  end

  if handBrakeHandling.smartParkingBrakeActive and electrics.values.parkingbrake > 0 and M.throttle > 0 then
    smartParkingBrake(0, FILTER_DIRECT)
    handBrakeHandling.smartParkingBrakeActive = false
  end

  energyStorageData.ratio = 0
  energyStorageData.volume = 0

  for _, s in ipairs(controlLogicModule.energyStorages or {}) do
    local energyStorage = energyStorage.getStorage(s)
    if energyStorage and energyStorage.type ~= "n2oTank" then
      energyStorageData.ratio = energyStorageData.ratio + energyStorage.remainingRatio
      energyStorageData.volume = energyStorageData.volume + energyStorage.remainingVolume
    end
  end
  energyStorageData.ratio = energyStorageData.ratio * energyStorageData.invEnergyStorageCount

  electrics.values.fuel = energyStorageData.ratio or 0
  electrics.values.lowfuel = electrics.values.fuel < 0.1
  electrics.values.fuelCapacity = energyStorageData.invEnergyStorageCount > 0 and (energyStorageData.capacity or 0) or 1
  electrics.values.fuelVolume = energyStorageData.volume

  electrics.values.throttle = M.throttle
  electrics.values.brake = M.brake
  electrics.values.brakelights = M.brake
  electrics.values.clutch = 1 - M.clutchRatio
  electrics.values.clutchRatio = M.clutchRatio
  electrics.values.isShifting = controlLogicModule.isShifting or false
  electrics.values.gear = gearName
  electrics.values.gear_A = gearPosition
  electrics.values.gearIndex = currentGearIndex
  electrics.values.rpm = controlLogicModule.rpm or 0
  electrics.values.oiltemp = controlLogicModule.oilTemp or 0
  electrics.values.watertemp = controlLogicModule.waterTemp or 0
  electrics.values.checkengine = controlLogicModule.checkEngine or false
  electrics.values.ignition = controlLogicModule.ignition == nil and true or controlLogicModule.ignition
  electrics.values.engineThrottle = controlLogicModule.engineThrottle or 0
  electrics.values.engineLoad = controlLogicModule.engineLoad or 0
  electrics.values.running = controlLogicModule.ignition == nil and true or controlLogicModule.ignition
  electrics.values.engineRunning = controlLogicModule.isEngineRunning or 0
  electrics.values.radiatorFanSpin = (engine and engine.thermals) and engine.thermals.radiatorFanSpin or 0

  if streams.willSend("engineInfo") then
    M.engineInfo[1] = controlLogicModule.idleRPM or 0
    M.engineInfo[2] = controlLogicModule.maxRPM or 0
    M.engineInfo[5] = controlLogicModule.rpm
    M.engineInfo[6] = gearName
    M.engineInfo[7] = controlLogicModule.maxGearIndex or 0
    M.engineInfo[8] = controlLogicModule.minGearIndex or 0
    M.engineInfo[9] = controlLogicModule.engineTorque or 0
    M.engineInfo[10] = controlLogicModule.gearboxTorque or 0
    M.engineInfo[11] = obj:getGroundSpeed() -- airspeed
    M.engineInfo[12] = electrics.values.fuelVolume
    M.engineInfo[13] = electrics.values.fuelCapacity
    M.engineInfo[16] = 0
    M.engineInfo[17] = electrics.values.gearIndex
    M.engineInfo[18] = controlLogicModule.isEngineRunning or 1
    M.engineInfo[19] = electrics.values.engineLoad
    M.engineInfo[20] = wheels.wheelTorque
    M.engineInfo[21] = wheels.wheelPower
  end

  if rpmLedsEnabled and playerInfo.firstPlayerSeated and engine then
    -- hydros.sendRPMLeds(engine.outputAV1 or 0, (engine.maxAV or 1) * 0.8, engine.maxAV or 1)
    obj:playRPMLeds(engine.outputAV1 or 0, (engine.maxAV or 1) * 0.8, engine.maxAV or 1) -- currentValue, firstLEDValue, lastLEDValue)
  end

  if gearbox then
    if streams.willSend("gearboxData") then
      gui.send(
        "gearboxData",
        {
          gearIndex = gearbox.gearIndex or "",
          clutchRatio = M.clutchRatio or "",
          dctGearIndex1 = gearbox.gearIndex1 or "",
          dctGearIndex2 = gearbox.gearIndex2 or "",
          dctClutchRatio1 = gearbox.clutchRatio1 or "",
          dctClutchRatio2 = gearbox.clutchRatio2 or ""
        }
      )
    end

    if streams.willSend("shiftDecisionData") then
      gui.send(
        "shiftDecisionData",
        {
          shiftUpRPM = shiftBehavior.shiftUpAV * constants.avToRPM,
          shiftDownRPM = shiftBehavior.shiftDownAV * constants.avToRPM,
          aggression = smoothedValues.drivingAggression,
          wheelSlipDown = shiftPreventionData.wheelSlipShiftDown,
          wheelSlipUp = shiftPreventionData.wheelSlipShiftUp
        }
      )
    end
  end
end

local function shiftUp()
  controlLogicModule.shiftUp()
end

local function shiftDown()
  controlLogicModule.shiftDown()
end

local function shiftToGearIndex(index)
  controlLogicModule.shiftToGearIndex(index)
end

local function cycleGearboxBehaviors()
  -- if not gearbox then
  --   return
  -- end

  local found = false
  local newBehavior = gearboxHandling.behavior
  for _, v in pairs(gearboxHandling.behaviors) do
    if found then
      newBehavior = v
      found = false
      break
    elseif gearboxHandling.behavior == v then
      found = true
    end
  end

  if found then
    newBehavior = gearboxHandling.behaviors[next(gearboxHandling.behaviors)]
  end

  setGearboxBehavior(newBehavior)
end

local function setStarter(enabled)
  if engine and enabled then
    engine:activateStarter()
  end
end

local function setFreeze(mode)
  isFrozen = mode == 1
  if gearbox then
    gearbox:setLock(isFrozen)
  end
end

local function setEngineIgnition(enabled)
  if engine then
    engine:setIgnition(enabled and 1 or 0)
  end
end

local function sendTorqueData()
  if not playerInfo.firstPlayerSeated then
    return
  end
  controlLogicModule.sendTorqueData()
end

local function sendShiftPointDebugData()
  if gearbox and gearbox.children and gearbox.children[1] and gearbox.gearRatios and #wheels.wheels > 0 then
    local shiftPointData = {forward = {}, reverse = {}}
    local wheelRadius = wheels.wheels[0].radius or 0 --TODO this should be auto calculated from the powertrain
    local postTransmissionGearRatioCoef = 1 / gearbox.children[1].cumulativeGearRatio

    for i = gearbox.minGearIndex, gearbox.maxGearIndex, 1 do
      local gearRatio = gearbox.gearRatios[i]
      if i > 0 then
        shiftPointData.forward[i] = {
          wheelSpeedCoef = postTransmissionGearRatioCoef * (1 / gearRatio) * wheelRadius * constants.rpmToAV,
          shiftUpHigh = shiftPoints[i].highShiftUpAV * constants.avToRPM,
          shiftDownHigh = shiftPoints[i].highShiftDownAV * constants.avToRPM,
          shiftUpLow = shiftPoints[i].lowShiftUpAV * constants.avToRPM,
          shiftDownLow = shiftPoints[i].lowShiftDownAV * constants.avToRPM,
          minRPM = engine.idleRPM or 0,
          maxRPM = engine.maxRPM
        }
      elseif i < 0 then
        shiftPointData.reverse[i] = {
          wheelSpeedCoef = postTransmissionGearRatioCoef * (1 / gearRatio) * wheelRadius * constants.rpmToAV,
          shiftUpHigh = shiftPoints[i].highShiftUpAV * constants.avToRPM,
          shiftDownHigh = shiftPoints[i].highShiftDownAV * constants.avToRPM,
          shiftUpLow = shiftPoints[i].lowShiftUpAV * constants.avToRPM,
          shiftDownLow = shiftPoints[i].lowShiftDownAV * constants.avToRPM,
          minRPM = engine.idleRPM or 0,
          maxRPM = engine.maxRPM
        }
      end
    end
    --dump(shiftPointData)
    guihooks.trigger("ShiftPointDebugDataChanged", shiftPointData)
  end
end

local function settingsChanged(noRefresh)
  if not noRefresh then
    settings.refresh()
  end

  gearboxHandling.autoClutch = settings.getValue("autoClutch", nil)
  if gearboxHandling.autoClutch == nil then
    log("W", "vehicleController.settingsChanged", "Got no autoClutch value from settings system, using default...")
    gearboxHandling.autoClutch = true
  end

  gearboxHandling.autoThrottle = settings.getValue("autoThrottle", nil)
  if gearboxHandling.autoThrottle == nil then
    log("W", "vehicleController.settingsChanged", "Got no autoThrottle value from settings system, using default...")
    gearboxHandling.autoThrottle = true
  end

  gearboxHandling.gearboxSafety = settings.getValue("gearboxSafety", nil)
  if gearboxHandling.gearboxSafety == nil then
    log("W", "vehicleController.settingsChanged", "Got no gearboxSafety value from settings system, using default...")
    gearboxHandling.gearboxSafety = true
  end

  rpmLedsEnabled = settings.getValue("rpmLedsEnabled") and false -- disabled until buggy ffb is fixed
end

local function calculateOptimalLoadShiftPoints(shiftDownRPMOffsetCoef)
  local torqueCurve = engine.torqueData.curves[engine.torqueData.finalCurveName].torque
  for k, v in pairs(gearbox.gearRatios) do
    local shiftUpRPM = nil
    local shiftDownRPM = nil
    if v ~= 0 then
      local currentGearRatio = v
      local nextGearRatio = gearbox.gearRatios[k + fsign(k)] or 0
      local previousGearRatio = gearbox.gearRatios[k - fsign(k)] or 0

      for i = 100, engine.maxRPM - 100, 50 do
        local currentWheelTorque = torqueCurve[i] * currentGearRatio
        local nextGearRPM = min(max(floor(i * (nextGearRatio / currentGearRatio)), 1), engine.maxRPM)
        local previousGearRPM = min(max(floor(i * (previousGearRatio / currentGearRatio)), 1), engine.maxRPM)
        local nextWheelTorque = torqueCurve[nextGearRPM] * nextGearRatio
        local previousWheelTorque = torqueCurve[previousGearRPM] * previousGearRatio

        if currentWheelTorque * fsign(currentGearRatio) < nextWheelTorque * fsign(currentGearRatio) and not shiftUpRPM and currentWheelTorque * currentGearRatio > 0 and currentGearRatio * nextGearRatio > 0 then
          shiftUpRPM = i
        end

        if previousWheelTorque * fsign(currentGearRatio) > currentWheelTorque * 1.05 * fsign(currentGearRatio) and previousGearRPM < engine.maxRPM * 0.9 and currentWheelTorque * currentGearRatio > 0 and currentGearRatio * previousGearRatio > 0 then
          shiftDownRPM = i
        end
      end

      if shiftDownRPM and shiftPoints[k - fsign(k)] and shiftPoints[k - fsign(k)].highShiftUpAV > 0 then
        local offsetCoef = shiftDownRPMOffsetCoef * (currentGearRatio / previousGearRatio)
        shiftDownRPM = min(shiftDownRPM, max(floor(shiftPoints[k - fsign(k)].highShiftUpAV / previousGearRatio * currentGearRatio * offsetCoef * constants.avToRPM), (engine.idleRPM or 0) * 1.05))
      end
    end

    shiftPoints[k].highShiftUpAV = (shiftUpRPM or engine.maxRPM * 0.97) * constants.rpmToAV
    shiftPoints[k].highShiftDownAV = (shiftDownRPM or 0) * constants.rpmToAV

    --print(string.format("Gear %d: Up: %d, Down: %d", k, shiftPoints[k].highShiftUpAV * constants.avToRPM, shiftPoints[k].highShiftDownAV * constants.avToRPM))
  end
end

local function init(jbeamData)
  M.throttle = 0
  M.brake = 0
  M.clutchRatio = 0
  gearboxHandling.isArcadeSwitched = false

  timer = {
    gearChangeDelayTimer = 0,
    shiftDelayTimer = 0,
    neutralSelectionDelayTimer = 0,
    stalledEngineMessageTimer = 0,
    stalledEngineTryingToStartTimer = 0,
    aggressionHoldOffThrottleTimer = 0
  }

  lastAggressionThrottle = 0

  settingsChanged(true)

  smoothedValues = {
    throttle = 0,
    brake = 0,
    throttleInput = 0,
    brakeInput = 0,
    drivingAggression = 0.75,
    throttleUpShiftThreshold = 0.05,
    avgAV = 0
  }

  shiftPreventionData = {
    wheelSlipShiftDown = false,
    wheelSlipShiftUp = false
  }

  energyStorageData = {
    ratio = 0,
    volume = 0,
    capacity = 0,
    invEnergyStorageCount = 0
  }

  handBrakeHandling = {
    smartParkingBrakeActive = false,
    smartParkingBrakeSlip = 0
  }

  local throttleSmoothingIn = jbeamData.gearboxDecisionSmoothingDown or 2
  local throttleSmoothingOut = jbeamData.gearboxDecisionSmoothingUp or 5
  local brakeSmoothingIn = jbeamData.gearboxDecisionSmoothingDown or 2
  local brakeSmoothingOut = jbeamData.gearboxDecisionSmoothingUp or 5
  local wheelSlipShiftUpSmoothingIn = jbeamData.wheelSlipShiftUpSmoothingIn or 10
  local wheelSlipShiftUpSmoothingOut = jbeamData.wheelSlipShiftUpSmoothingOut or 20
  local aggressionSmoothingOut = jbeamData.aggressionSmoothingUp or 1.5
  local aggressionSmoothingIn = jbeamData.aggressionSmoothingDown or 0.15

  smoother.throttle = newTemporalSmoothing(throttleSmoothingIn, throttleSmoothingOut)
  smoother.brake = newTemporalSmoothing(brakeSmoothingIn, brakeSmoothingOut)
  smoother.throttleInput = newTemporalSmoothing(throttleSmoothingIn * 2, throttleSmoothingOut * 2)
  smoother.brakeInput = newTemporalSmoothing(brakeSmoothingIn * 2, brakeSmoothingOut * 2)
  smoother.aggressionAxis = newTemporalSmoothingNonLinear(aggressionSmoothingIn, aggressionSmoothingOut)
  smoother.aggressionKey = newTemporalSmoothingNonLinear(aggressionSmoothingIn, aggressionSmoothingOut * 0.2)
  smoother.avgAV = newTemporalSmoothingNonLinear(5, 5)
  smoother.wheelSlipShiftUp = newTemporalSmoothingNonLinear(wheelSlipShiftUpSmoothingIn, wheelSlipShiftUpSmoothingOut)
  smoother.groundContactSmoother = newTemporalSmoothing(100, 2)

  smoother.aggressionAxis:set(smoothedValues.drivingAggression)
  smoother.aggressionKey:set(smoothedValues.drivingAggression)

  gearboxHandling.useSmartAggressionCalculation = (jbeamData.useSmartAggressionCalculation == nil or jbeamData.useSmartAggressionCalculation)

  topSpeedLimit = jbeamData.topSpeedLimit or -1
  topSpeedLimitReverse = jbeamData.topSpeedLimitReverse or -1
  topSpeedLimitPID = newPIDParallel(1, 0.5, 0, 0, 1)

  shiftPreventionData.wheelSlipUpThreshold = jbeamData.wheelSlipUpThreshold or 20000
  shiftPreventionData.wheelSlipDownThreshold = jbeamData.wheelSlipDownThreshold or 30000

  engine = powertrain.getDevice("mainEngine")
  gearbox = powertrain.getDevice("gearbox")

  local hasGearbox = gearbox ~= nil
  local controlLogicName = "dummy"

  if hasGearbox then
    gearboxType = gearbox.type

    local shiftRPMNames = {"lowShiftDownRPM", "highShiftDownRPM", "lowShiftUpRPM", "highShiftUpRPM"}
    local defaultShiftPoints = {lowShiftDownRPM = 2000, highShiftDownRPM = 3500, lowShiftUpRPM = 2500, highShiftUpRPM = 5000}
    local gearCount = gearbox.gearCount + 1
    for _, v in pairs(shiftRPMNames) do
      if type(jbeamData[v]) ~= "table" then
        local shiftRPM = jbeamData[v] or defaultShiftPoints[v]
        jbeamData[v] = {}
        for _ = 0, gearCount, 1 do
          table.insert(jbeamData[v], shiftRPM)
        end
      else
        if tableSize(jbeamData[v]) ~= gearCount then
          for i = 1, gearCount, 1 do
            if not jbeamData[v][i] then
              jbeamData[v][i] = jbeamData[v][i - 1] or 0
            end
          end
        end
      end
    end

    local gearCounter = 1
    for i = gearbox.minGearIndex, gearbox.maxGearIndex, 1 do
      shiftPoints[i] = {
        lowShiftDownAV = jbeamData.lowShiftDownRPM[gearCounter] * constants.rpmToAV,
        highShiftDownAV = (jbeamData.highShiftDownRPM[gearCounter] or 0) * constants.rpmToAV,
        lowShiftUpAV = jbeamData.lowShiftUpRPM[gearCounter] * constants.rpmToAV,
        highShiftUpAV = (jbeamData.highShiftUpRPM[gearCounter] or 0) * constants.rpmToAV
      }
      gearCounter = gearCounter + 1
    end

    --dump(jbeamData)

    if jbeamData.calculateOptimalLoadShiftPoints then
      local shiftDownRPMOffsetCoef = jbeamData.shiftDownRPMOffsetCoef or 1.3
      calculateOptimalLoadShiftPoints(shiftDownRPMOffsetCoef)
    end

    --dump(shiftPoints)

    sendShiftPointDebugData()

    controlLogicName = gearboxType

    timerConstants.shiftDelay = jbeamData.transmissionShiftDelay or 0.2
    timerConstants.gearChangeDelay = jbeamData.transmissionGearChangeDelay or 0.5
    timerConstants.neutralSelectionDelay = jbeamData.neutralSelectionDelay or 0.5
    timerConstants.aggressionHoldOffThrottleDelay = jbeamData.aggressionHoldOffThrottleDelay or 2.25
  end

  if jbeamData.shiftLogicName then
    controlLogicName = jbeamData.shiftLogicName
  end

  gearboxHandling.behaviorLookup = {}
  for _, v in pairs(gearboxHandling.behaviors) do
    gearboxHandling.behaviorLookup[v] = true
  end

  local controlLogicModuleName = "controller/shiftLogic-" .. controlLogicName
  controlLogicModule = require(controlLogicModuleName)

  controlLogicModule.init(jbeamData, sharedFunctions)
  controlLogicModule.gearboxHandling = gearboxHandling
  controlLogicModule.timer = timer
  controlLogicModule.timerConstants = timerConstants
  controlLogicModule.inputValues = inputValues
  controlLogicModule.shiftPreventionData = shiftPreventionData
  controlLogicModule.shiftBehavior = shiftBehavior
  controlLogicModule.smoothedValues = smoothedValues

  M.setDefaultForwardMode = controlLogicModule.setDefaultForwardMode

  local energyStorageCount = 0
  for _, s in pairs(controlLogicModule.energyStorages or {}) do
    local energyStorage = energyStorage.getStorage(s)
    if energyStorage and energyStorage.type ~= "n2oTank" then
      energyStorageData.capacity = energyStorageData.capacity + energyStorage.capacity
      energyStorageCount = energyStorageCount + 1
    end
  end
  energyStorageData.invEnergyStorageCount = energyStorageCount > 0 and 1 / energyStorageCount or 0

  setGearboxBehavior(gearboxHandling.previousBehavior or settings.getValue("defaultGearboxBehavior") or "arcade")

  sendTorqueData()

  M.updateGFX = updateGFXGeneric
end

local function initLastStage()
  input.state.parkingbrake.val = 1
  handBrakeHandling.smartParkingBrakeActive = true
end

local function resetLastStage()
  input.state.parkingbrake.val = 1
  handBrakeHandling.smartParkingBrakeActive = true
end

local function vehicleActivated()
  if not playerInfo.firstPlayerSeated then
    return
  end
  sendTorqueData()
  sendShiftPointDebugData()
end

local function onDeserialize(data)
  if data.previousGearboxBehavior then
    setGearboxBehavior(data.previousGearboxBehavior)
  end
end

local function onSerialize()
  return {previousGearboxBehavior = gearboxHandling.behavior}
end

M.init = init
M.initLastStage = initLastStage
M.resetLastStage = resetLastStage
M.updateGFX = nop
M.settingsChanged = settingsChanged

M.onDeserialize = onDeserialize
M.onSerialize = onSerialize

M.setAggressionOverride = setAggressionOverride
M.setDefaultForwardMode = nop

--Mandatory main controller API
M.shiftUp = shiftUp
M.shiftDown = shiftDown
M.shiftToGearIndex = shiftToGearIndex
M.cycleGearboxModes = cycleGearboxBehaviors
M.setGearboxMode = setGearboxBehavior
M.smartParkingBrake = smartParkingBrake
M.setStarter = setStarter
M.setEngineIgnition = setEngineIgnition
M.setFreeze = setFreeze
M.sendTorqueData = sendTorqueData
M.vehicleActivated = vehicleActivated
-------------------------------

return M
