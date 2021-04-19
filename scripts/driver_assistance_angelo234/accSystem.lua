require("lua/common/luaProfiler")

local M = {}

local control_systems = require("controlSystems") -- for newPIDStandard
local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local acc_pid = newPIDStandard(0.05, 2, 1, -1, 0.35)
local acc_smooth = newTemporalSmoothing(200, 200)

local system_active = false

local function getVehicleAheadInLane(dt, my_veh_props, data_table)
  local distance = 9999
  local rel_vel = 0
  local curr_veh_in_path = nil

  --Analyze the trajectory of other vehicles with my trajectory
  --to see if collision imminent
  for _, data in pairs(data_table) do
    local my_veh_side = data.my_veh_wps_props.side_of_wp
    local other_veh_side = data.other_veh_wps_props.side_of_wp
    
    --Other vehicle properties
    local other_veh_props = extra_utils.getVehicleProperties(data.other_veh)

    local this_rel_vel = (my_veh_props.velocity - other_veh_props.velocity):length()

    --Capping to 5 seconds to prevent too much error in predicting position
    local ttc = math.min(data.distance / this_rel_vel, 5)
    
    local my_wp_dir = (data.my_veh_wps_props.end_wp_pos - data.my_veh_wps_props.start_wp_pos):normalized()
    local my_wp_perp_dir_right = vec3(my_wp_dir.y, -my_wp_dir.x)
    
    local other_wp_dir = (data.other_veh_wps_props.end_wp_pos - data.other_veh_wps_props.start_wp_pos):normalized()
    local other_wp_perp_dir_right = vec3(other_wp_dir.y, -other_wp_dir.x)
    
    local my_lat_dist_from_wp = data.my_veh_wps_props.lat_dist_from_wp
    local other_lat_dist_from_wp = data.other_veh_wps_props.lat_dist_from_wp
    
    local my_speed_in_wp_perp_dir = my_veh_props.velocity:dot(my_wp_perp_dir_right)
    local other_speed_in_wp_perp_dir = other_veh_props.velocity:dot(other_wp_perp_dir_right)

    if my_veh_side == "right" then
      my_lat_dist_from_wp = my_lat_dist_from_wp + my_speed_in_wp_perp_dir * ttc
    else
      my_lat_dist_from_wp = -my_lat_dist_from_wp + my_speed_in_wp_perp_dir * ttc
    end
    
    if other_veh_side == "right" then
      other_lat_dist_from_wp = other_lat_dist_from_wp + other_speed_in_wp_perp_dir * ttc
    else
      other_lat_dist_from_wp = -other_lat_dist_from_wp + other_speed_in_wp_perp_dir * ttc
    end
        
    --print("my_lat_dist_from_wp: " .. my_lat_dist_from_wp)
    --print("other_lat_dist_from_wp: " .. other_lat_dist_from_wp) 
        
    if my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
    and my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
    then
      --This may be the car to adapt to
    
      --If this distance is less than current min distance
      --then this is new min distance
      if data.distance <= distance then
        distance = data.distance
        rel_vel = this_rel_vel

        curr_veh_in_path = data.other_veh
        
        --debugDrawer:drawSphere((other_veh_props.center_pos):toPoint3F(), 1, ColorF(1,0,0,1))      
      end
    end   
  end

  return distance, rel_vel
end

local last_output = 0

--Maintaining distance to vehicle using PID controller
local function maintainDistanceFromVehicleAhead(dt, veh, distance, vel_rel, aeb_params)
  --print(distance)

  local output = acc_pid:get(-(distance - 10), 0, dt)

  output = acc_smooth:getUncapped(output, dt)

  if math.abs(output) > 0.4 then
  
    --Disable ACC System if large input needed to maintain specific distance
    if math.abs(last_output) > 0.4 then
      scripts_driver__assistance__angelo234_extension.toggleACCSystem()
    end
    
    last_output = output
    return
  end

  last_output = output

  if output > 0 then
    --Accelerate
    veh:queueLuaCommand("input.event('throttle'," .. output .. ", 2)")
    veh:queueLuaCommand("input.event('brake', 0, 2)")   
  else
    --Brake
    veh:queueLuaCommand("input.event('brake'," .. -output .. ", 2)")  
    veh:queueLuaCommand("input.event('throttle', 0, 2)")   
  end
end

local function update(dt, veh, vehs_in_same_lane_table, system_params, aeb_params)
  local veh_props = extra_utils.getVehicleProperties(veh)
  
  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]
  local esc_color = electrics_values_angelo234["dseColor"]

  --ESC must be in comfort mode, otherwise it is deactivated
  --sunburst uses different color for comfort mode
  --if (veh_name == "sunburst" and esc_color == "98FB00")
   --or (veh_name ~= "sunburst" and esc_color == "238BE6")
  --then
  
  --Deactivate system based on any of these variables
  if in_reverse == nil or in_reverse == 1 or gear_selected == nil
    or gear_selected == 'P' or gear_selected == 0 then
    if system_active then
      veh:queueLuaCommand("input.event('brake', 0, 2)")  
      system_active = false 
    end

    return
  end
  
  local distance = 9999
  local vel_rel = 0

  --If table is empty then return
  if next(vehs_in_same_lane_table) ~= nil then
    --Determine if a collision will actually occur and return the distance and relative velocity 
    --to the vehicle that I'm planning to collide with
    distance, vel_rel = getVehicleAheadInLane(dt, veh_props, vehs_in_same_lane_table)
  end
  
  if distance < 30 then
    --Use distance, relative velocity, and max acceleration to determine when to apply emergency braking
    maintainDistanceFromVehicleAhead(dt, veh, distance, vel_rel, aeb_params)
  else
    acc_smooth:reset()
    last_output = 0    
  end
end

M.update = update

return M