local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

--ready = system not doing anything 
--braking = AEB active
--holding = car is stopped and system is holding the brakes 
local system_state = "ready"

--Uses predicted future pos and places it relative to the future waypoint
--based on relative position to the current waypoint
local function getFutureVehPosCorrectedWithWP(my_veh_props, veh_props, veh_pos_future, lat_dist_from_wp, my_veh_side)
  local veh_wps_props = extra_utils.getWaypointStartEndAdvanced(my_veh_props, veh_props, veh_pos_future)

  if veh_wps_props == nil then
    return veh_pos_future, nil
  end

  local wp_start_end = veh_wps_props.end_wp_pos - veh_wps_props.start_wp_pos
  local wp_dir = wp_start_end:normalized()

  local xnorm = veh_pos_future:xnormOnLine(veh_wps_props.start_wp_pos, veh_wps_props.end_wp_pos)

  local new_pos = xnorm * wp_start_end + veh_wps_props.start_wp_pos

  local perp_vec = nil

  if my_veh_side == "right" then
    --On right side of waypoints
    perp_vec = vec3(wp_dir.y, -wp_dir.x)
  elseif my_veh_side == "left" then
    --On left side
    perp_vec = vec3(-wp_dir.y, wp_dir.x)
  end

  perp_vec = perp_vec * lat_dist_from_wp

  new_pos = new_pos + perp_vec

  return new_pos, wp_dir
end

local function getMyVehBoundingBox(my_veh_props, my_veh_wp_dir)
  local my_bb = my_veh_props.bb

  local my_x = nil -- width
  local my_y = nil -- length
  local my_z = nil -- height

  if my_veh_wp_dir ~= nil then
    my_x = my_bb:getHalfExtents().x * vec3(my_veh_wp_dir.y, -my_veh_wp_dir.x, 0) * 1.05
    my_y = my_bb:getHalfExtents().y * my_veh_wp_dir * (1.25 + my_veh_props.speed * my_veh_props.speed / 200)
    my_z = my_bb:getHalfExtents().z * vec3(0,0,1) * 2
  else
    my_x = my_bb:getHalfExtents().x * vec3(my_bb:getAxis(0)) * 1.05
    my_y = my_bb:getHalfExtents().y * vec3(my_bb:getAxis(1)) * (1.25 + my_veh_props.speed * my_veh_props.speed / 200)
    my_z = my_bb:getHalfExtents().z * vec3(my_bb:getAxis(2)) * 2
  end

  return my_x, my_y, my_z
end

local function getOtherVehBoundingBox(other_veh_props, other_veh_wp_dir, distance)
  local other_bb = other_veh_props.bb

  local other_x = nil -- width
  local other_y = nil -- length
  local other_z = nil -- height

  if other_veh_wp_dir ~= nil then
    other_x = other_bb:getHalfExtents().x * vec3(other_veh_wp_dir.y, -other_veh_wp_dir.x, 0)
    other_y = other_bb:getHalfExtents().y * other_veh_wp_dir * (1 + distance / 25.0)
    other_z = other_bb:getHalfExtents().z * vec3(0,0,1) * 2

  else
    other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0)) * 1
    other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1)) * (1 + distance / 25.0)
    other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2)) * 2
  end

  return other_x, other_y, other_z
end

--Check if we'll actually crash at TTC
local function checkIfCarsIntersectAtTTC(my_veh_props, other_veh_props, data, lateral_acc_to_avoid_collision)
  --Calculate TTC
  local vel_rel = (my_veh_props.velocity - other_veh_props.velocity):length()
  local speed_rel = my_veh_props.speed - other_veh_props.speed

  --Deactivate system if this car is slower than other car
  if vel_rel <= 0 then
    return false
  end

  --Capping to 5 seconds to prevent too much error in predicting position
  local ttc = math.min(data.distance / vel_rel, 5)

  local my_veh_pos_future = extra_utils.getFuturePositionXY(my_veh_props, ttc, "front")
  local other_veh_pos_future = extra_utils.getFuturePositionXY(other_veh_props, ttc, "center")


  --Calculate the bounding boxes of both vehicles

  --My BB
  -- width, length, height
  local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, nil)

  --Other BB
  -- width, length, height
  local other_x, other_y, other_z = getOtherVehBoundingBox(other_veh_props, nil, data.distance)

  --Check for overlap between both bounding boxess
  local overlap = overlapsOBB_OBB(my_veh_pos_future, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)

  return overlap
end

local function getVehicleCollidingWith(dt, my_veh_props, data_table)
  local distance = 9999
  local rel_vel = 0
  
  --If table is empty then return
  if next(data_table) == nil then
    return distance, rel_vel
  end
  
  --Analyze the trajectory of other vehicles with my trajectory
  --to see if collision imminent
  for _, data in pairs(data_table) do
    --Other vehicle properties
    local other_veh_props = extra_utils.getVehicleProperties(data.other_veh)
  
    local this_rel_vel = (my_veh_props.velocity - other_veh_props.velocity):length()
  
    local overlap = checkIfCarsIntersectAtTTC(my_veh_props, other_veh_props, data, 0.1)
    
    if overlap then
      if data.distance <= distance then
        distance = data.distance
        rel_vel = this_rel_vel
      end
    end
  end
  
  return distance, rel_vel
end


local function getVehicleCollidingWithInLane(dt, my_veh_props, data_table, lateral_acc_to_avoid_collision)
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

    --Deactivate system if this car is slower than other car
    if this_rel_vel > 0 then
      --Capping to 5 seconds to prevent too much error in predicting position
      local ttc = math.min(data.distance / this_rel_vel, 5)

      local my_lat_dist_from_wp = data.my_veh_wps_props.lat_dist_from_wp
      local other_lat_dist_from_wp = data.other_veh_wps_props.lat_dist_from_wp

      --debugDrawer:drawTextAdvanced((other_veh_props.front_pos):toPoint3F(), String(other_lat_dist_from_wp),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

      if my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
      and my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
      then
        --Collision may be possible
      
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
  end

  return distance, rel_vel
end

local beeper_timer = 0

local function soundBeepers(dt, time_before_braking, vel_rel, beeper_params)
  beeper_timer = beeper_timer + dt

  --Sound warning tone if 1.0 * (0.5 + vel_rel / 40.0) seconds away from braking
  if time_before_braking <= 1.0 * (math.min(0.5 + vel_rel / 30.0, 1.0)) then
    --
    if beeper_timer >= 1.0 / beeper_params.fwd_warning_tone_hertz then
      Engine.Audio.playOnce('AudioGui','art/sound/proximity_tone_50ms_loud.wav')
      beeper_timer = 0
    end
  end
end

local release_brake_confidence_level = 0

local function performEmergencyBraking(dt, veh, aeb_params, time_before_braking, speed)
  --If throttle pedal is about half pressed then perform braking
  --But if throttle is highly requested then override braking
  if input_throttle_angelo234 > 0.5 then
    if system_state == "braking" then
      veh:queueLuaCommand("electrics.values.brakeOverride = nil")
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      system_state = "ready" 
    end
    return
  end
  
  --Stop car completely if below certain speed regardless of sensor information
  if system_state == "braking" and speed < aeb_params.brake_till_stop_speed then
    veh:queueLuaCommand("electrics.values.brakeOverride = 1")
    return
  end

  --debugDrawer:drawTextAdvanced((my_veh_props.front_pos + my_veh_props.dir * 2):toPoint3F(), String("Time till braking: " .. tostring(time_before_braking)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  --Maximum Braking
  if time_before_braking <= 0 then
    if input_throttle_angelo234 > 0.1 then
      veh:queueLuaCommand("electrics.values.throttleOverride = 0")
    end   
    veh:queueLuaCommand("electrics.values.brakeOverride = 1")
    
    --Turn off Adaptive Cruise Control
    scripts_driver__assistance__angelo234_extension.setACCSystemOn(false)

    system_state = "braking"

    release_brake_confidence_level = 0
  else
    release_brake_confidence_level = release_brake_confidence_level + 1

    --Only release brakes if confident
    if release_brake_confidence_level >= 5 then
      if system_state == "braking" then
        veh:queueLuaCommand("electrics.values.brakeOverride = nil")
        veh:queueLuaCommand("electrics.values.throttleOverride = nil")
        
        system_state = "ready"
      end
    end
  end
end

local function calculateTimeBeforeBraking(distance, vel_rel, system_params, aeb_params)
  --Max braking acceleration = gravity * coefficient of static friction
  --local acc = math.min(-veh_accs_angelo234[veh_props.id][3], system_params.gravity) * params_per_veh[veh_props.name].fwd_friction_coeff

  local acc = math.min(10, system_params.gravity) * system_params.fwd_friction_coeff

  --Calculate TTC
  local ttc = distance / vel_rel
  local time_to_brake = vel_rel / (2 * acc)

  --leeway time depending on speed
  local time_before_braking = ttc - time_to_brake - aeb_params.braking_time_leeway
  
  return time_before_braking
end

local function holdBrakes(veh, veh_props, aeb_params)
  if veh_props.speed <= aeb_params.min_speed then    
    if system_state == "braking" then
      --When coming to a stop with system activated, release brakes but apply parking brake in arcade mode :P
      if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
        veh:queueLuaCommand("electrics.values.brakeOverride = 1")
      else
        --Release brake and apply parking brake
        veh:queueLuaCommand("electrics.values.brakeOverride = 0") 
      end
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      
      system_state = "holding"    
    end
  end
  
  --If vehicle held by brake after AEB and user modulates throttle or brake pedal then release brakes
  if system_state == "holding" and (input_throttle_angelo234 > 0 or input_brake_angelo234 > 0) then
    veh:queueLuaCommand("electrics.values.brakeOverride = nil")
    
    system_state = "ready"
  end

  return system_state == "holding"
end

local function update(dt, veh, system_params, aeb_params, beeper_params, front_sensor_data)
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
    or gear_selected == 'P' then   
    if system_state ~= "ready" then
      veh:queueLuaCommand("electrics.values.brakeOverride = nil")     
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")  
      system_state = "ready" 
    end
    return
  end
  
  local veh_props = extra_utils.getVehicleProperties(veh)
  
  if holdBrakes(veh, veh_props, aeb_params) then return end

  local static_distance = 9999
  local distance = 9999
  local vel_rel = 0

  --Do static object detection up to certain speed
  if veh_props.speed < 11.11 then
    --Do static object raycasting
    static_distance = front_sensor_data[1]
  end

  --If speed less than certain value, use different AEB system (not using lane lines)
  if veh_props.speed < 8.33 then
    --Get nearby vehicles
    distance, vel_rel = getVehicleCollidingWith(dt, veh_props, front_sensor_data[2])
  else
    --Else at higher speeds, use lane lines 
    --If table is empty then return
    if next(front_sensor_data[3]) ~= nil then
      --Determine if a collision will actually occur and return the distance and relative velocity 
      --to the vehicle that I'm planning to collide with
      distance, vel_rel = getVehicleCollidingWithInLane(dt, veh_props, front_sensor_data[3], aeb_params.lateral_acc_to_avoid_collision)
    end 
  end

  --If static object is closer than nearest vehicle, then use static object raycast data
  if static_distance < distance then
    distance = static_distance
    vel_rel = veh_props.speed
  end

  --Takes 1 frame to actually brake so account for that
  --distance = distance - vel_rel * dt 

  local time_before_braking = calculateTimeBeforeBraking(distance, vel_rel, system_params, aeb_params)

  if extra_utils.checkIfPartExists("forward_aeb_angelo234") then
    --Use distance, relative velocity, and max acceleration to determine when to apply emergency braking
    performEmergencyBraking(dt, veh, aeb_params, time_before_braking, veh_props.speed)
  end
   
  if extra_utils.checkIfPartExists("forward_collision_warning_angelo234") then
    --At low speeds don't sound beepers
    if veh_props.speed > 11.11 then
      soundBeepers(dt, time_before_braking, vel_rel, beeper_params)
    end
  end
end

M.update = update

return M