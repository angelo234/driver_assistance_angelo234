-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--global table of all vehicles acceleration vectors
veh_accs_angelo234 = {}

local M = {}

local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')

local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')
local aeb_system = require('scripts/drivers_assistance_angelo234/aebSystem')
local parking_sensor_system = require('scripts/drivers_assistance_angelo234/parkingSensorSystem')

--Bunch of parameters
local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local function getAllVehiclesPropertiesFromVELua()
  local vehicles = getAllVehicles()
  local my_veh = be:getPlayerVehicle(0)

  for _, this_veh in pairs(vehicles) do
    local id = this_veh:getID()

    this_veh:queueLuaCommand('obj:queueGameEngineLua("veh_accs_angelo234[' .. id .. '] = {" .. sensors.gx2 .. "," .. sensors.gy2 .. "," .. sensors.gz2 .. "}")')
  end

  --Get properties of my vehicle
  my_veh:queueLuaCommand("obj:queueGameEngineLua('throttle_pos_angelo234 = ' .. input.throttle )")
  my_veh:queueLuaCommand('obj:queueGameEngineLua("electrics_values_angelo234 = (\'" .. jsonEncode(electrics.values) .. "\')")')
  my_veh:queueLuaCommand("obj:queueGameEngineLua('angular_speed_angelo234 = ' .. obj:getYawAngularVelocity() )")

  if electrics_values_angelo234 == nil then
    return false
  end

  return veh_accs_angelo234 ~= nil
    and #electrics_values_angelo234 ~= 0
    and angular_speed_angelo234 ~= nil
    and throttle_pos_angelo234 ~= nil
end

local function doLuaReload()
  --Determine if Lua loaded for first time or was loaded another time
  --if loaded for first time, then reload Lua to load in the reverse camera properly
  --if not, then don't need to do anything
  
  local log_file_header_file = "prev_log_file_header_angelo234.txt"
  local curr_log_file_header = readFile("beamng.log"):match("^.-\r")
  
  if not FS:fileExists(log_file_header_file) then
    writeFile(log_file_header_file, curr_log_file_header)
    Lua:requestReload()
  else
    local prev_log_file_header = readFile(log_file_header_file)
    
    --Means different game instance so reload lua again
    if prev_log_file_header ~= curr_log_file_header then
      writeFile(log_file_header_file, curr_log_file_header)
      Lua:requestReload()    
    end 
  end
end

local function isVehicleSupported()
  local the_veh_name = veh:getJBeamFilename()
  
  for curr_veh_name, _ in pairs(params_per_veh) do
    if the_veh_name == curr_veh_name then
      return true
    end
  end
  
  return false
end

local yawSmooth = newExponentialSmoothing(10) --exponential smoothing for the yaw rate
local first_update = true

local function onUpdate(dt)
  --Check if Lua reload required
  if first_update then
    doLuaReload()   
    first_update = false
  end

  local veh = be:getPlayerVehicle(0)
  if veh == nil then return end

  if not isVehicleSupported() then return end

  local ready = getAllVehiclesPropertiesFromVELua()

  if not ready then return end

  --Decode electrics.values json result
  electrics_values_angelo234 = jsonDecode(electrics_values_angelo234)

  --Smoothes angular velocity
  angular_speed_angelo234 = yawSmooth:get(angular_speed_angelo234)

  local the_veh_name = veh:getJBeamFilename()

  --Update systems based on what is supported on a vehicle basis
  for _, val in pairs(params_per_veh[the_veh_name].systems) do
    if val == "parking_sensors" then
      parking_sensor_system.update(dt, veh)

    elseif val == "aeb" then
      aeb_system.update(dt, veh)
    end

  end
end

M.onUpdate = onUpdate

return M
