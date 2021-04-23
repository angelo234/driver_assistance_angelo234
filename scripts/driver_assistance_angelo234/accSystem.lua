require("lua/common/luaProfiler")

local M = {}

local control_systems = require("controlSystems") -- for newPIDStandard
local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

--PID for getting vehicle up to desired speed when no car in front
--Input = desired velocity, Output = acceleration
local speed_pid = newPIDStandard(0.3, 2, 0.0, 0, 1, 1, 1, 0, 2)
local speed_smooth = newTemporalSmoothing(200, 200)

--PID for setting car to following distance
--Input = desired distance, Output = acceleration
local dist_pid = newPIDStandard(0.1, 2, 0, -9.81 * 0.3, 9.81 * 0.3)
local dist_smooth = newTemporalSmoothing(200, 200)

local target_speed = 16.67
local ramped_target_speed = 0

local following_time = 1


local function setACCSpeed()
  local my_veh = be:getPlayerVehicle(0)
  local veh_speed = vec3(my_veh:getVelocity()):length()

  target_speed = veh_speed
  ramped_target_speed = veh_speed
  
  --Display speed in user's units

  local units = settings.getValue("uiUnitLength")
  local the_unit = "m/s"

  if units == "metric" then
    the_unit = "kph" 
    veh_speed = veh_speed * 3.6
  elseif units == "imperial" then
    the_unit = "mph" 
    veh_speed = veh_speed * 2.24
  end
  
  veh_speed = math.floor(veh_speed)

  ui_message("Adaptive Cruise Control speed set to " .. tostring(veh_speed) .. " " .. the_unit)
end

local function changeACCSpeed(amt)
  local my_veh = be:getPlayerVehicle(0)
  local veh_speed = vec3(my_veh:getVelocity()):length()
  
  --amt is 1 or -1 so increment/decrement by user's units

  local display_speed = target_speed

  local units = settings.getValue("uiUnitLength")
  local the_unit = "m/s"

  if units == "metric" then
    the_unit = "kph"   
    target_speed = target_speed + amt / 3.6
    
    display_speed = target_speed * 3.6
  elseif units == "imperial" then
    the_unit = "mph" 
    target_speed = target_speed + amt / 2.24 
  
    display_speed = target_speed * 2.24
  end
  
  --Minimum of 50 km/h or 31 mph
  if math.floor(target_speed) <= 13.8889 then
    target_speed = 13.9
    
    if units == "metric" then
      display_speed = 50
    elseif units == "imperial" then
      display_speed = 31
    end
  end
  
  ramped_target_speed = veh_speed
  
  display_speed = math.floor(display_speed)

  ui_message("Adaptive Cruise Control speed set to " .. tostring(display_speed) .. " " .. the_unit)
end

local function changeACCFollowingDistance(amt)
  following_time = following_time + amt

  if following_time <= 0 then
    following_time = 0.5
  elseif following_time > 4 then
    following_time = 4
  end

  ui_message("Adaptive Cruise Control following distance set to " .. tostring(following_time) .. "s")
end

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

    local my_lat_dist_from_wp = data.my_veh_wps_props.lat_dist_from_wp
    local other_lat_dist_from_wp = data.other_veh_wps_props.lat_dist_from_wp

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
        
        debugDrawer:drawSphere((other_veh_props.center_pos):toPoint3F(), 1, ColorF(1,0,0,1))      
      end
    else
      debugDrawer:drawTextAdvanced((other_veh_props.front_pos):toPoint3F(), String("Delta distance: " .. math.abs(my_lat_dist_from_wp - other_lat_dist_from_wp)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  
    end   
  end

  return distance, rel_vel
end

local function accelerateVehicle(dt, veh, output)
  if output > 0 then
    --Accelerate
    veh:queueLuaCommand("electrics.values.throttleOverride = " .. output)

    veh:queueLuaCommand("input.event('brake', 0, 2)")   
  else
    --Brake
    veh:queueLuaCommand("input.event('brake'," .. -output .. ", 2)")
    veh:queueLuaCommand("electrics.values.throttleOverride = 0")   
  end
end

--Maintaining distance to vehicle using PID controller
local function maintainDistanceFromVehicleAhead(dt, veh, veh_props, aeb_params, distance, vel_rel, following_distance)
  --minimum of 5 meter following distance
  --local output = -1.0 / acc_following_time * (0 + (following_distance - distance) - vel_rel)
  
  --return output

  --print(distance)

  --minimum of 5 meter following distance
  --local following_distance = math.max(veh_props.speed * acc_following_time, 5)

  --print(following_distance)

  local output = dist_pid:get(-(distance), -following_distance, dt)

  output = dist_smooth:getUncapped(output, dt)

  accelerateVehicle(dt, veh, output)
end

local function maintainSetSpeed(dt, veh, veh_props, aeb_params)
  --Code from BeamNG's cruiseControl.lua

  --ramp up/down our target speed with our desired target acceleration to avoid integral wind-up
  if ramped_target_speed ~= target_speed then
    local upperLimit = target_speed > ramped_target_speed and target_speed or ramped_target_speed
    local lowerLimit = target_speed < ramped_target_speed and target_speed or ramped_target_speed
    ramped_target_speed = clamp(ramped_target_speed + fsign(target_speed - ramped_target_speed) * 3 * dt, lowerLimit, upperLimit)
  end

  local currentSpeed = veh_props.speed
  local output = speed_pid:get(currentSpeed, ramped_target_speed, dt)
  output = speed_smooth:getUncapped(output, dt)

  accelerateVehicle(dt, veh, output)

  --print("Desired Acceleration: " .. output)

end

local x = 1

local function update(dt, veh, system_params, aeb_params, vehs_in_same_lane_table)
  if not dt then
    speed_pid:reset()
    speed_smooth:reset()
    dist_pid:reset()
    dist_smooth:reset()
    veh:queueLuaCommand("electrics.values.throttleOverride = nil") 
    
    ramped_target_speed = 0
    return
  end
  
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
    or gear_selected == 'P' or gear_selected == 0 or input_throttle_angelo234 > 0 then
    
    veh:queueLuaCommand("electrics.values.throttleOverride = " .. input_throttle_angelo234)
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
  
  
  local following_distance = veh_props.speed * following_time
  
  --If distance less than following distance to maintain and still hasn't reached target speed
  --use distance PID controller
  if distance < following_distance * x and veh_props.speed < target_speed then
    --Use distance, relative velocity, and max acceleration to determine when to apply emergency braking
    maintainDistanceFromVehicleAhead(dt, veh, veh_props, aeb_params, distance, vel_rel, following_distance)

    ramped_target_speed = veh_props.speed
    speed_pid:reset()
    speed_smooth:reset()
    
    x = 1.35
  else
    --Else use speed PID controller
  
    maintainSetSpeed(dt, veh, veh_props, aeb_params)
  
    --print(distance)
    dist_pid:reset()
    dist_smooth:reset()
    --veh:queueLuaCommand("electrics.values.throttleOverride = nil") 
    
    x = 1   
  end
end

M.setACCSpeed = setACCSpeed
M.changeACCSpeed = changeACCSpeed
M.changeACCFollowingDistance = changeACCFollowingDistance 
M.update = update

return M