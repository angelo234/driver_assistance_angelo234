-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--global table of all vehicles acceleration vectors
veh_accs_angelo234 = {}

local M = {}

local aeb_system_ge = require('scripts/driver_assistance_angelo234/aebSystem')
local parking_aeb_system_ge = require('scripts/driver_assistance_angelo234/parkingAEBSystem')

local system_params = nil
local aeb_params = nil
local rev_aeb_params = nil
local parking_lines_params = nil
local beeper_params = nil

M.curr_camera_mode = "orbit"
M.prev_camera_mode = "orbit"

local fwd_aeb_on = true
local rev_aeb_on = true

local function init(player)
  local veh = be:getPlayerVehicle(player)
  
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

--Called with key binding
local function toggleFWDAEBSystem()
  fwd_aeb_on = not fwd_aeb_on
  
  local msg = nil
  
  if fwd_aeb_on then
    msg = "ON"
  else
    msg = "OFF"
  end
  
  ui_message("Forward AEB switched " .. msg)
end

--Called with key binding
local function toggleREVAEBSystem()
  rev_aeb_on = not rev_aeb_on
  
  local msg = nil
  
  if rev_aeb_on then
    msg = "ON"
  else
    msg = "OFF"
  end
  
  ui_message("Reverse AEB switched " .. msg)
end

--Used for what camera to switch the player to when the player gets out of reverse gear using reverse camera
local function onCameraModeChanged(new_camera_mode)
  if new_camera_mode ~= M.curr_camera_mode then
    M.prev_camera_mode = M.curr_camera_mode
    M.curr_camera_mode = new_camera_mode
  end
end

local function getAllVehiclesPropertiesFromVELua()
  local vehicles = getAllVehicles()
  local my_veh = be:getPlayerVehicle(0)

  for _, this_veh in pairs(vehicles) do
    local id = this_veh:getID()

    this_veh:queueLuaCommand('obj:queueGameEngineLua("veh_accs_angelo234[' .. id .. '] = {" .. sensors.gx2 .. "," .. sensors.gy2 .. "," .. sensors.gz2 .. "}")')
  end

  --Get properties of my vehicle
  my_veh:queueLuaCommand("if input.throttle ~= nil then obj:queueGameEngineLua('throttle_pos_angelo234 = ' .. input.throttle ) end")
  my_veh:queueLuaCommand('obj:queueGameEngineLua("electrics_values_angelo234 = (\'" .. jsonEncode(electrics.values) .. "\')")')
  my_veh:queueLuaCommand("obj:queueGameEngineLua('angular_speed_angelo234 = ' .. obj:getYawAngularVelocity() )")
  
  --Gets whether gearbox is in arcade or realistic mode
  my_veh:queueLuaCommand('if controller.mainController.onSerialize ~= nil then obj:queueGameEngineLua("gearbox_mode_angelo234 = (\'" .. jsonEncode(controller.mainController.onSerialize()) .. "\')") end')

  if electrics_values_angelo234 == nil then
    return false
  end

  return veh_accs_angelo234 ~= nil
    and #electrics_values_angelo234 ~= 0
    and angular_speed_angelo234 ~= nil
    and throttle_pos_angelo234 ~= nil
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

--Complicated way to load in camera (but it works quite seamlessly)
local function doLuaReload()
  --Determine if Lua loaded for first time or was loaded another time
  --if loaded for first time, then reload Lua to load in the reverse camera properly
  --if not, then don't need to do anything
  --And if log header changes from "Log started" to "Log rotated" then don't bother reloading
  --since by that time, the reverse camera probably was already loaded in
  
  local log_file_header_file = "prev_log_file_header_angelo234.txt"
  local curr_log_file_header = readFile("beamng.log"):match("^.-\r")
  local curr_log_status = curr_log_file_header:match("Log.-ed")
  
  --If log header doesn't state "Log started" then don't do anything
  if curr_log_status ~= "Log started" then
    return
  end
  
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

local first_update = true

local function onUpdate(dt)
  --Do Lua reload after first Lua initialization to load in the reverse camera
  if first_update then
    doLuaReload()   
    first_update = false
  end
  
  local my_veh = be:getPlayerVehicle(0)
  if my_veh == nil then return end
  
  local ready = getAllVehiclesPropertiesFromVELua()
  --If Vehicle Lua data is nil then return
  if not ready then return end

  --Process data gathered from Vehicle Lua to be usable in our context
  processVELuaData()
  
  if not be:getEnabled() or not system_params then return end
  
  local parts = extensions.core_vehicle_manager.getPlayerVehicleData().chosenParts

  if parts.forward_aeb_angelo234 == "forward_aeb_angelo234" and fwd_aeb_on then
    aeb_system_ge.update(dt, my_veh, system_params, aeb_params, beeper_params) 
  end
  
  if parts.reverse_aeb_angelo234 == "reverse_aeb_angelo234" and rev_aeb_on then
    parking_aeb_system_ge.update(dt, my_veh, system_params, parking_lines_params, rev_aeb_params, beeper_params)
  end
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleSwitched = onVehicleSwitched
M.toggleFWDAEBSystem = toggleFWDAEBSystem
M.toggleREVAEBSystem = toggleREVAEBSystem
M.onCameraModeChanged = onCameraModeChanged
M.onUpdate = onUpdate

return M
