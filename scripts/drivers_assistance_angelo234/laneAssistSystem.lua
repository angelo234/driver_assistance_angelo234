local M = {}

require("controlSystems") -- for newPIDStandard
local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')
local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')

local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local steering_pid = newPIDStandard(0.1, 2, 1.25, -0.2, 0.2)
local steering_smooth = newTemporalSmoothing(200, 200)

local system_active = false

local function performSteering(veh, val)
  veh:queueLuaCommand("input.event('steering'," .. val .. ", 0)")
end

local function getOffsetFromCenterOfLane(veh_props)
  local veh_wps_props = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.center_pos)

  local side_of_wp, in_wp_middle = extra_utils.getWhichSideOfWaypointsCarIsOn(veh_props, veh_wps_props.start_wp_pos, veh_wps_props.end_wp_pos)

  print(veh_wps_props.lane_width)

  if veh_wps_props.one_way then -- and lane_width < 3 then
    --One lane road
    local new_lat_dist = veh_wps_props.lat_dist_from_wp
    
    
    
    if side_of_wp == "left" then
      new_lat_dist = -new_lat_dist
    end
    return new_lat_dist 
  else
    --Two lane road
    return veh_wps_props.lat_dist_from_wp - (veh_wps_props.lane_width / 2.0)
  end
end

local function getSteeringValue(dt, offset) 
  --print(offset)

  local output = steering_pid:get(offset, 0, dt)
  --electrics.values.throttleOverride = steering_smooth:getUncapped(output, dt)
  
  return output
  --return steering_smooth:getUncapped(output, dt)
end

local function update(dt, veh)
  local veh_props = extra_utils.getVehicleProperties(veh)

  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]
  local esc_color = electrics_values_angelo234["dseColor"]

  --ESC must be in comfort mode, otherwise it is deactivated
  --sunburst uses different color for comfort mode
  if (veh_props.name == "sunburst" and esc_color == "98FB00")
    or (veh_props.name ~= "sunburst" and esc_color == "238BE6")
  then
     --Deactivate system based on any of these variables
    if in_reverse == nil or in_reverse == 1 or gear_selected == nil
      or gear_selected == 'P' or gear_selected == 0
      or veh_props.speed > aeb_params.max_speed or veh_props.speed <= aeb_params.min_speed then 
      if system_active then
        system_active = false
      end
      return 
    end
    
    local offset = getOffsetFromCenterOfLane(veh_props)
    local steering_val = getSteeringValue(dt, offset)
    
    performSteering(veh, steering_val)
  end
end

M.update = update

return M
