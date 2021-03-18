-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

veh_accs = {}

local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')

local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')
local aeb_system = require('scripts/drivers_assistance_angelo234/aebSystem')
local parking_sensor_system = require('scripts/drivers_assistance_angelo234/parkingSensorSystem')

--Bunch of parameters
local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local M = {}

local time_elapsed_since_ready = 0
local world_ready = false
local finished_init = false

local function onWorldReadyState(state)

  if state == 2 and not world_ready then
    world_ready = true
  end
end


local function getAllVehiclesPropertiesFromVELua()
  local vehicles = getAllVehicles()
  local my_veh = be:getPlayerVehicle(0)

  for _, this_veh in pairs(vehicles) do
    local id = this_veh:getID()

    this_veh:queueLuaCommand('obj:queueGameEngineLua("veh_accs[' .. id .. '] = {" .. sensors.gx2 .. "," .. sensors.gy2 .. "," .. sensors.gz2 .. "}")')
  end

  --Get properties of my vehicle
  my_veh:queueLuaCommand("obj:queueGameEngineLua('throttle_pos = ' .. input.throttle )")
  my_veh:queueLuaCommand('obj:queueGameEngineLua("electrics_values = (\'" .. jsonEncode(electrics.values) .. "\')")')
  my_veh:queueLuaCommand("obj:queueGameEngineLua('angular_speed = ' .. obj:getYawAngularVelocity() )")

  if electrics_values == nil then
    return false
  end

  return veh_accs ~= nil
    and #electrics_values ~= 0
    and angular_speed ~= nil
    and throttle_pos ~= nil
end

local system_active = false
local stopped = false


local yawSmooth = newExponentialSmoothing(10) --exponential smoothing for the yaw rate

local last_angular_speed = 0
local last_steering_input = 0

local function onUpdate(dt)

  if not finished_init then
    if time_elapsed_since_ready > 5 then
      extensions.reload('core_camera')
      extensions.reload('core_sounds')
      extensions.reload('editor_main')
      
      finished_init = true
    else
      if world_ready then
        time_elapsed_since_ready = time_elapsed_since_ready + dt
      end
    end
  end

  

  local veh = be:getPlayerVehicle(0)
  if veh == nil then return end

  local veh_velocity = vec3(be:getPlayerVehicle(0):getVelocity())
  local veh_speed = veh_velocity:length()
  local the_veh_name = veh:getJBeamFilename()
  local is_supported = false

  for curr_veh_name, _ in pairs(params_per_veh) do
    if the_veh_name == curr_veh_name then
      is_supported = true
      break
    end
  end

  if not is_supported then return end

  local ready = getAllVehiclesPropertiesFromVELua()

  if not ready then return end

  --Decode electrics.values json result
  electrics_values = jsonDecode(electrics_values)

  --Smoothes angular velocity
  angular_speed = yawSmooth:get(angular_speed)

  local in_reverse = electrics_values["reverse"]
  local gear_selected = electrics_values["gear"]
  local esc_color = electrics_values["dseColor"]

  --Update systems
  for _, val in pairs(params_per_veh[the_veh_name].systems) do
    if val == "parking_sensors" then
      parking_sensor_system.update(dt, veh)

    elseif val == "aeb" then
      aeb_system.update(dt, veh)
    end

  end

  last_steering_input = electrics_values["steering_input"]
end

M.onUpdate = onUpdate
M.onWorldReadyState = onWorldReadyState

return M
