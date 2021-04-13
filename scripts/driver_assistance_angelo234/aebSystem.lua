require("lua/common/luaProfiler")

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local system_params = nil
local aeb_params = nil
local beeper_params = nil

local system_active = false

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

local function getFreePathInLane(my_veh_side, in_wp_middle, lane_width, other_lat_dist_from_wp, my_veh_width, other_veh_width)
  --If we can't fit our vehicles in same lane, then no free path available
  if my_veh_width + other_veh_width + 0.5 > lane_width or in_wp_middle then
    return "none"
  end
  
  if my_veh_side == "left" then
    if other_lat_dist_from_wp <= lane_width / 2.0 then
      return "left" 
    else
      return "right"
    end
  elseif my_veh_side == "right" then
    if other_lat_dist_from_wp <= lane_width / 2.0 then
      return "right" 
    else
      return "left"
    end
  end
end

--We know cars are in same lane, now check if we'll actually crash at TTC
local function checkIfCarsIntersectAtTTC(my_veh_props, other_veh_props, data, lateral_acc_to_avoid_collision)
  local my_lat_dist_from_wp = data.my_veh_wps_props.lat_dist_from_wp
  local other_lat_dist_from_wp = data.other_veh_wps_props.lat_dist_from_wp
  local my_veh_side = data.my_veh_wps_props.side_of_wp
  local in_wp_middle = data.my_veh_wps_props.in_wp_middle
  local lane_width = data.my_veh_wps_props.lane_width
  
  --Calculate TTC
  local vel_rel = (my_veh_props.velocity - other_veh_props.velocity):length()
  local speed_rel = my_veh_props.speed - other_veh_props.speed

  --Deactivate system if this car is slower than other car
  if vel_rel <= 0 then
    return false
  end

  --Capping to 5 seconds to prevent too much error in predicting position
  local ttc = math.min(data.distance / vel_rel, 5)

  local my_veh_pos_future = extra_utils.getFuturePositionXYWithAcc(my_veh_props, ttc, vec3(), "front")
  local other_veh_pos_future = extra_utils.getFuturePositionXY(other_veh_props, ttc, "center")

  local my_veh_wp_dir = nil
  local other_veh_wp_dir = nil

  --If lane lines exist near our vehicles then predict using them
  --for the best accuracy
  
  --print(my_veh_side)

  if my_veh_side ~= nil and not in_wp_middle then
    my_veh_pos_future.z = my_veh_props.center_pos.z
    my_veh_pos_future, my_veh_wp_dir = getFutureVehPosCorrectedWithWP(my_veh_props, my_veh_props, my_veh_pos_future, my_lat_dist_from_wp, my_veh_side)
    my_veh_pos_future.z = 0

    other_veh_pos_future.z = other_veh_props.center_pos.z
    other_veh_pos_future, other_veh_wp_dir = getFutureVehPosCorrectedWithWP(my_veh_props, other_veh_props, other_veh_pos_future, other_lat_dist_from_wp, my_veh_side)
    other_veh_pos_future.z = 0
  end

  my_veh_pos_future.z = my_veh_props.center_pos.z
  other_veh_pos_future.z = other_veh_props.center_pos.z

  --debugDrawer:drawSphere((my_veh_pos_future):toPoint3F(), 1, ColorF(0,1,0,1))
  --debugDrawer:drawSphere((other_veh_pos_future):toPoint3F(), 1, ColorF(1,0,0,1))
  
  my_veh_pos_future.z = 0
  other_veh_pos_future.z = 0

  --Calculate the bounding boxes of both vehicles

  --My BB
  -- width, length, height
  local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, my_veh_wp_dir)

  --Other BB
  -- width, length, height
  local other_x, other_y, other_z = getOtherVehBoundingBox(other_veh_props, other_veh_wp_dir, data.distance)

  --Check for overlap between both bounding boxess
  local overlap = overlapsOBB_OBB(my_veh_pos_future, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)

  if overlap then
    --At low speeds, predict if light steering input (0.1 g's laterally) can avoid collision
    --then deactivate system
    if my_veh_props.speed < 20 or true then
      local free_path = getFreePathInLane(my_veh_side, in_wp_middle, lane_width, other_lat_dist_from_wp, 
      my_veh_props.bb:getHalfExtents().x * 2, other_veh_props.bb:getHalfExtents().x * 2)
            
      local turning_acc_vec = nil
    
      --print(free_path)
    
      if free_path == "left" then
        turning_acc_vec = vec3(lateral_acc_to_avoid_collision * 9.81 + my_veh_props.acceleration.x, 0, 0)
      elseif free_path == "right" then
        turning_acc_vec = vec3(-lateral_acc_to_avoid_collision * 9.81 + my_veh_props.acceleration.x, 0, 0)
      else
        return overlap
      end

      local my_veh_pos_future_turning = extra_utils.getFuturePositionXYWithAcc(my_veh_props, ttc, turning_acc_vec, "front")
  
      local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, nil)
  
      local overlap2 = overlapsOBB_OBB(my_veh_pos_future_turning, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)

      my_veh_pos_future_turning.z = my_veh_props.center_pos.z
      other_veh_pos_future.z = my_veh_props.center_pos.z

      --debugDrawer:drawSphere((my_veh_pos_future_turning):toPoint3F(), 0.5, ColorF(1,0,1,1))
      --debugDrawer:drawSphere((other_veh_pos_future):toPoint3F(), 0.5, ColorF(1,0,1,1))

      return overlap2
    end
  end

  return overlap
end

local function getNearestVehicleInPath(dt, my_veh_props, data_table, lateral_acc_to_avoid_collision)
  local distance = 9999
  local rel_vel = 0
  local curr_veh_in_path = nil

  --If table is empty then return
  if next(data_table) == nil then
    return distance, rel_vel
  end

  --Analyze the trajectory of other vehicles with my trajectory
  --to see if collision imminent
  for _, data in pairs(data_table) do
    local my_veh_side = data.my_veh_wps_props.side_of_wp
  
    --Other vehicle properties
    local other_veh_props = extra_utils.getVehicleProperties(data.other_veh)

    local this_rel_vel = (my_veh_props.velocity - other_veh_props.velocity):length()

    --local overlap_at_ttc = checkIfCarsIntersectAtTTC(my_veh_props, other_veh_props, data, lateral_acc_to_avoid_collision)

    local overlap_at_ttc = false
    
    --Calculate TTC
    local vel_rel = (my_veh_props.velocity - other_veh_props.velocity):length()
    local speed_rel = my_veh_props.speed - other_veh_props.speed
  
    --Deactivate system if this car is slower than other car
    if vel_rel <= 0 then
      return false
    end
  
    --Capping to 5 seconds to prevent too much error in predicting position
    local ttc = math.min(data.distance / vel_rel, 5)
    
    
    local my_wp_dir = (data.my_veh_wps_props.end_wp_pos - data.my_veh_wps_props.start_wp_pos):normalized()
    local my_wp_perp_dir_right = vec3(my_wp_dir.y, -my_wp_dir.x)
    
    local my_lat_dist_from_wp = data.my_veh_wps_props.lat_dist_from_wp
    local other_lat_dist_from_wp = data.other_veh_wps_props.lat_dist_from_wp
    
    local my_speed_in_wp_perp_dir = my_veh_props.velocity:dot(my_wp_perp_dir_right)
    local other_speed_in_wp_perp_dir = other_veh_props.velocity:dot(my_wp_perp_dir_right)


    if my_veh_side == "right" then
      my_lat_dist_from_wp = my_lat_dist_from_wp + my_speed_in_wp_perp_dir * ttc
      other_lat_dist_from_wp = other_lat_dist_from_wp + other_speed_in_wp_perp_dir * ttc
        
      if my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
      or my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
      then
        overlap_at_ttc = true
      end    
    else
      my_lat_dist_from_wp = my_lat_dist_from_wp - my_speed_in_wp_perp_dir * ttc
      other_lat_dist_from_wp = other_lat_dist_from_wp + other_speed_in_wp_perp_dir * ttc
       
      if my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
      or my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
      then
        overlap_at_ttc = true
      end  
    end

    --Collision may be possible
    if overlap_at_ttc then
      --debugDrawer:drawSphere((other_veh_props.center_pos):toPoint3F(), 1, ColorF(1,0,0,1))
    
      --If this distance is less than current min distance
      --then this is new min distance
      if data.distance <= distance then
        distance = data.distance
        rel_vel = this_rel_vel

        curr_veh_in_path = data.other_veh
      end
    end
  end

  return distance, rel_vel
end

local beeper_timer = 0

local function soundBeepers(dt, time_before_braking, vel_rel)
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

local function performEmergencyBraking(dt, veh, time_before_braking, speed)
  --If throttle pedal is about half pressed then perform braking
  --But if throttle is highly requested then override braking
  if electrics_values_angelo234["throttle"] > 0.4 then
    if system_active then
      veh:queueLuaCommand("input.event('brake', 0, 2)")
      system_active = false 
    end
    return
  end
  
  --Stop car completely if below certain speed regardless of sensor information
  if system_active and speed < aeb_params.brake_till_stop_speed then
    veh:queueLuaCommand("input.event('brake', 1, 2)")
    return
  end

  --debugDrawer:drawTextAdvanced((my_veh_props.front_pos + my_veh_props.dir * 2):toPoint3F(), String("Time till braking: " .. tostring(time_before_braking)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  --Maximum Braking
  if time_before_braking <= 0 then
    veh:queueLuaCommand("input.event('throttle', 0, 2)")
    veh:queueLuaCommand("input.event('brake', 1, 2)")
    system_active = true

    release_brake_confidence_level = 0
  else    
    release_brake_confidence_level = release_brake_confidence_level + 1

    --Only release brakes if confident
    if release_brake_confidence_level >= 15 then
      if system_active then
        veh:queueLuaCommand("input.event('brake', 0, 2)")
        system_active = false
      end
    end
  end
end

local function calculateTimeBeforeBraking(distance, vel_rel)
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

local function update(dt, veh, the_system_params, the_fwd_aeb_params, the_beeper_params)
  system_params = the_system_params
  aeb_params = the_fwd_aeb_params
  beeper_params = the_beeper_params

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
      
  if veh_props.speed <= aeb_params.min_speed then    
    if system_active then
      --When coming to a stop with system activated, release brakes but apply parking brake in arcade mode :P
      if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
        veh:queueLuaCommand("input.event('brake', 1, 2)")
      else
        --Release brake and apply parking brake
        veh:queueLuaCommand("input.event('brake', 0, 2)")
        veh:queueLuaCommand("input.event('parkingbrake', 1, 2)")   
      end
      system_active = false     
    end
    
    return   
  end

  local data = extra_utils.getNearbyVehiclesInSameLane(veh_props, aeb_params.vehicle_search_radius, aeb_params.min_distance_from_car, true)

  --Determine if a collision will actually occur and return the distance and relative velocity 
  --to the vehicle that I'm planning to collide with
  local distance, vel_rel = getNearestVehicleInPath(dt, veh_props, data, aeb_params.lateral_acc_to_avoid_collision)

  --Takes 1 frame to actually send data so account for that
  distance = distance - vel_rel * dt 


  local time_before_braking = calculateTimeBeforeBraking(distance, vel_rel)

  --At low speeds don't sound beepers
  if veh_props.speed > 5 then
    soundBeepers(dt, time_before_braking, vel_rel)
  end
  
  --Use distance, relative velocity, and max acceleration to determine when to apply emergency braking
  performEmergencyBraking(dt, veh, time_before_braking, veh_props.speed)
end

M.update = update

return M