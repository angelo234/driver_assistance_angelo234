-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--global table of all vehicles acceleration vectors
--veh_accs_angelo234[id][1] = lateral (-x = right, +x = left)
--veh_accs_angelo234[id][2] = longitudinal (-x = accelerating, +x = braking)
--veh_accs_angelo234[id][3] = up/down direction (-x = down, +x = up)
veh_accs_angelo234 = {}

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local sensor_system = require('scripts/driver_assistance_angelo234/sensorSystem')
local fcm_system = require('scripts/driver_assistance_angelo234/forwardCollisionMitigationSystem')
local rcm_system = require('scripts/driver_assistance_angelo234/reverseCollisionMitigationSystem')
local acc_system = require('scripts/driver_assistance_angelo234/accSystem')
local hsa_system = require('scripts/driver_assistance_angelo234/hillStartAssistSystem')
--local auto_headlight_system = require('scripts/driver_assistance_angelo234/autoHeadlightSystem')

local first_update = true

local system_params = nil
local aeb_params = nil
local rev_aeb_params = nil
local parking_lines_params = nil
local beeper_params = nil

local fcm_system_on = true
local rcm_system_on = true
local auto_headlight_system_on = false
local prev_auto_headlight_system_on = false
local acc_system_on = false

local front_sensor_data = nil
local rear_sensor_data = nil

local other_systems_timer = 0
local hsa_system_update_timer = 0
local auto_headlight_system_update_timer = 0

M.curr_camera_mode = "orbit"
M.prev_camera_mode = "orbit"

local function init(player)
  local veh = be:getPlayerVehicle(player)

  if not veh then return end

  local veh_name = veh:getJBeamFilename()

  local default_param_file_dir = 'vehicles/common/parameters'
  local param_file_dir = 'vehicles/' .. veh_name .. '/parameters'

  if FS:fileExists(param_file_dir .. ".lua") then
    --load parameter lua file dependent on vehicle
    system_params = require(param_file_dir)
  else
    --use default parameters if they don't exist for current vehicle
    system_params = require(default_param_file_dir)
  end

  aeb_params = system_params.fwd_aeb_params
  beeper_params = system_params.beeper_params
  rev_aeb_params = system_params.rev_aeb_params
  parking_lines_params = system_params.rev_cam_params.parking_lines_params
end

local function onExtensionLoaded()
  init(0)
end

local function onVehicleSwitched(oid, nid, player)
  init(player)
end

local function onHeadlightsOff()
  --auto_headlight_system.onHeadlightsOff()
end

local function onHeadlightsOn()
  --auto_headlight_system.onHeadlightsOn()
end

--Functions called with key binding
local function toggleFCMSystem()
  if not extra_utils.getPart("forward_collision_mitigation_angelo234") then return end

  fcm_system_on = not fcm_system_on

  local msg = nil

  if fcm_system_on then
    msg = "ON"
  else
    msg = "OFF"
  end

  ui_message("Forward Collision Mitigation System switched " .. msg)
end

local function toggleRCMSystem()
  if not extra_utils.getPart("reverse_collision_mitigation_angelo234") then return end

  rcm_system_on = not rcm_system_on

  local msg = nil

  if rcm_system_on then
    msg = "ON"
  else
    msg = "OFF"
  end

  ui_message("Reverse Collision Mitigation System switched " .. msg)
end

local function toggleAutoHeadlightSystem()
  --[[

  if not extra_utils.getPart("auto_headlight_angelo234") then return end

  auto_headlight_system_on = not auto_headlight_system_on

  local msg = nil

  if auto_headlight_system_on then
    msg = "ON"
  else
    msg = "OFF"
  end

  ui_message("Auto Headlight Dimming switched " .. msg)

  ]]--
end

local function setACCSystemOn(on)
  if not extra_utils.getPart("acc_angelo234") then return end

  if acc_system_on ~= on then
    acc_system_on = on

    acc_system.onToggled(acc_system_on)
  end
end

local function toggleACCSystem()
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system_on = not acc_system_on

  acc_system.onToggled(acc_system_on)
end

local function setACCSpeed()
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system.setACCSpeed()
end

local function changeACCSpeed(amt)
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system.changeACCSpeed(amt)
end

local function changeACCFollowingDistance(amt)
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system.changeACCFollowingDistance(amt)
end

--Used for what camera to switch the player to when the player gets out of reverse gear using reverse camera
local function onCameraModeChanged(new_camera_mode)
  if new_camera_mode ~= M.curr_camera_mode then
    M.prev_camera_mode = M.curr_camera_mode
    M.curr_camera_mode = new_camera_mode
  end
end

local function getAllVehiclesPropertiesFromVELua(my_veh)
  for i = 0, be:getObjectCount() - 1 do
    local this_veh = be:getObject(i)
    local id = this_veh:getID()

    this_veh:queueLuaCommand('obj:queueGameEngineLua("veh_accs_angelo234[' .. id .. '] = {" .. sensors.gx2 .. "," .. sensors.gy2 .. "," .. sensors.gz2 .. "}")')
  end

  --Get properties of my vehicle
  my_veh:queueLuaCommand("if input.throttle ~= nil then obj:queueGameEngineLua('input_throttle_angelo234 = ' .. input.throttle ) end")
  my_veh:queueLuaCommand("if input.brake ~= nil then obj:queueGameEngineLua('input_brake_angelo234 = ' .. input.brake ) end")
  my_veh:queueLuaCommand("if input.clutch ~= nil then obj:queueGameEngineLua('input_clutch_angelo234 = ' .. input.clutch ) end")
  my_veh:queueLuaCommand("if input.parkingbrake ~= nil then obj:queueGameEngineLua('input_parkingbrake_angelo234 = ' .. input.parkingbrake ) end")

  my_veh:queueLuaCommand('obj:queueGameEngineLua("electrics_values_angelo234 = (\'" .. jsonEncode(electrics.values) .. "\')")')
  my_veh:queueLuaCommand("obj:queueGameEngineLua('angular_speed_angelo234 = ' .. obj:getYawAngularVelocity() )")
  my_veh:queueLuaCommand("obj:queueGameEngineLua('rotation_angelo234 = ' .. vec3(obj:getRollPitchYaw()):__tostring() )")

  --Gets whether gearbox is in arcade or realistic mode
  my_veh:queueLuaCommand('if controller.mainController.onSerialize ~= nil then obj:queueGameEngineLua("gearbox_mode_angelo234 = (\'" .. jsonEncode(controller.mainController.onSerialize()) .. "\')") end')

  if electrics_values_angelo234 == nil then
    return false
  end

  return veh_accs_angelo234 ~= nil
    and #electrics_values_angelo234 ~= 0
    and angular_speed_angelo234 ~= nil
    and input_throttle_angelo234 ~= nil
    and input_brake_angelo234 ~= nil
    and input_clutch_angelo234 ~= nil
    and input_parkingbrake_angelo234 ~= nil
    and gearbox_mode_angelo234 ~= nil and gearbox_mode_angelo234 ~= "null" and type(gearbox_mode_angelo234) ~= "table"
end

local yawSmooth = newExponentialSmoothing(10) --exponential smoothing for yaw rate

local function processVELuaData()
  --Decode json results
  electrics_values_angelo234 = jsonDecode(electrics_values_angelo234)
  gearbox_mode_angelo234 = jsonDecode(gearbox_mode_angelo234)

  --Smoothes angular velocity
  angular_speed_angelo234 = yawSmooth:get(angular_speed_angelo234)
end

--local p = LuaProfiler("my profiler")

local i = 0

local function onUpdate(dt)
  --p:start()

  if first_update then
    -- sensor_system.init()
    -- fcm_system.init()
    -- rcm_system.init()
    -- acc_system.init()
    -- hsa_system.init()
    --auto_headlight_system.init()
    first_update = false
  end

  local my_veh = be:getPlayerVehicle(0)
  if my_veh == nil then return end

  local ready = getAllVehiclesPropertiesFromVELua(my_veh)
  --If Vehicle Lua data is nil then return
  if not ready then return end

  --Process data gathered from Vehicle Lua to be usable in our context
  processVELuaData()

  if not be:getEnabled() or not system_params then return end

  local veh_props = extra_utils.getVehicleProperties(my_veh)

  if extra_utils.getPart("acc_angelo234")
  or extra_utils.getPart("forward_collision_mitigation_angelo234")
  or extra_utils.getPart("reverse_collision_mitigation_angelo234")
  then
    --Update at 120 Hz
    if other_systems_timer >= 1.0 / 120.0 then
      if i == 0 then
        --Get sensor data
        front_sensor_data = sensor_system.pollFrontSensors(other_systems_timer * 2, veh_props, system_params, aeb_params)
        rear_sensor_data = sensor_system.pollRearSensors(other_systems_timer * 2, veh_props, system_params, rev_aeb_params)

        i = 1

      elseif i == 1 then
        --Update Adaptive Cruise Control
        if extra_utils.getPart("acc_angelo234") and acc_system_on then
          acc_system.update(other_systems_timer * 2, my_veh, system_params, aeb_params, front_sensor_data)
        end

        --Update Forward Collision Mitigation System
        if extra_utils.getPart("forward_collision_mitigation_angelo234") and fcm_system_on then
          fcm_system.update(other_systems_timer * 2, my_veh, system_params, aeb_params, beeper_params, front_sensor_data)
        end

        --Update Reverse Collision Mitigation System
        if extra_utils.getPart("reverse_collision_mitigation_angelo234") and rcm_system_on then
          rcm_system.update(other_systems_timer * 2, my_veh, system_params, parking_lines_params, rev_aeb_params, beeper_params, rear_sensor_data)
        end

        i = 0
      end

      other_systems_timer = 0
    end
  end

  --Update at 10 Hz
  if hsa_system_update_timer >= 0.1 then
    --Update Hill Start Assist System
    if extra_utils.getPart("hill_start_assist_angelo234") then
      hsa_system.update(hsa_system_update_timer, my_veh)
    end

    hsa_system_update_timer = 0

    --p:add("hsa update")
  end

  --AUTO HEADLIGHT SYSTEM DISABLED TEMPORARILY WHILE FINDING A FIX

  --[[
  --Update at 4 Hz
  if auto_headlight_system_update_timer >= 0.25 then
    if extra_utils.getPart("auto_headlight_angelo234") and auto_headlight_system_on then
      if front_sensor_data ~= nil then
        if prev_auto_headlight_system_on ~= auto_headlight_system_on then
          auto_headlight_system.systemSwitchedOn()
        end

        auto_headlight_system.update(auto_headlight_system_update_timer, my_veh, front_sensor_data[2])
      end

      auto_headlight_system_update_timer = 0
    end

    prev_auto_headlight_system_on = auto_headlight_system_on

    --p:add("auto headlight update")
  end
  ]]--

  --Update timers for updating systems
  other_systems_timer = other_systems_timer + dt
  hsa_system_update_timer = hsa_system_update_timer + dt
  auto_headlight_system_update_timer = auto_headlight_system_update_timer + dt

  --p:finish(true)
end

local function onInit()
  setExtensionUnloadMode(M, "manual")
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleSwitched = onVehicleSwitched
M.onHeadlightsOff = onHeadlightsOff
M.onHeadlightsOn = onHeadlightsOn
M.toggleFCMSystem = toggleFCMSystem
M.toggleRCMSystem = toggleRCMSystem
M.toggleAutoHeadlightSystem = toggleAutoHeadlightSystem
M.setACCSystemOn = setACCSystemOn
M.toggleACCSystem = toggleACCSystem
M.setACCSpeed = setACCSpeed
M.changeACCSpeed = changeACCSpeed
M.changeACCFollowingDistance = changeACCFollowingDistance
M.onCameraModeChanged = onCameraModeChanged
M.onUpdate = onUpdate
M.onInit = onInit

return M
