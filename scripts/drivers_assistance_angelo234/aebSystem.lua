local M = {}

local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')
local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')

local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local system_active = false
local stopped = false

--Uses predicted future pos and places it relative to the future waypoint
local function getFutureVehPosCorrectedWithWP(my_veh, veh, veh_pos_future, lat_dist_from_wp, my_veh_side)
  local new_start_wp, new_end_wp, new_lat_dist = extra_utils.getWaypointStartEndBasedOnDir(my_veh, veh, veh_pos_future)

  if new_start_wp == nil or new_end_wp == nil then
    return veh_pos_future, nil
  end
  
  --Convert waypoints to positions
  local start_pos = extra_utils.getWaypointPosition(new_start_wp)
  local end_pos = extra_utils.getWaypointPosition(new_end_wp)
  
  --debugDrawer:drawSphere((start_pos):toPoint3F(), 5, ColorF(1,0,0,1))
  --debugDrawer:drawSphere((end_pos):toPoint3F(), 5, ColorF(0,1,0,1))
  local wp_start_end = end_pos - start_pos
  local wp_dir = wp_start_end:normalized()
  
  local xnorm = veh_pos_future:xnormOnLine(start_pos, end_pos)

  local new_pos = xnorm * wp_start_end + start_pos

  --debugDrawer:drawSphere((veh_pos_future):toPoint3F(), 10, ColorF(1,0,0,1))

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

local function getMyVehBoundingBox(my_veh_props, my_veh_wp_dir, distance)
  local my_bb = my_veh_props.bb

  local my_x = nil -- width
  local my_y = nil -- length
  local my_z = nil -- height

  if my_veh_wp_dir ~= nil then
    my_x = my_bb:getHalfExtents().x * vec3(my_veh_wp_dir.y, -my_veh_wp_dir.x, 0) * 0.95
    my_y = my_bb:getHalfExtents().y * my_veh_wp_dir * (1.25 + my_veh_props.speed * my_veh_props.speed / 200)
    my_z = my_bb:getHalfExtents().z * vec3(0,0,1) * 2
  else
    my_x = my_bb:getHalfExtents().x * vec3(my_bb:getAxis(0)) * 0.95
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

--We know cars are in same lane, now check if we'll actually crash at TTC
local function checkIfCarsIntersectAtTTC(dt, my_veh, other_veh, distance, my_lat_dist_from_wp, other_lat_dist_from_wp, my_veh_side)
  --My vehicle properties
  local my_veh_props = extra_utils.getVehicleProperties(my_veh)

  --Other vehicle properties
  local other_veh_props = extra_utils.getVehicleProperties(other_veh)

  --Calculate TTC
  local vel_rel = (my_veh_props.velocity - other_veh_props.velocity):length()
  local speed_rel = my_veh_props.speed - other_veh_props.speed

  --Deactivate system if this car is slower than other car
  if vel_rel <= 0 then
    return false
  end

  --Capping to 5 to prevent too much error
  local ttc = math.min(distance / vel_rel, 5)

  local my_veh_pos_future = extra_utils.getFuturePositionXY(my_veh, ttc, "front")
  local other_veh_pos_future = extra_utils.getFuturePositionXY(other_veh, ttc, "rear")
  
  local my_veh_wp_dir = nil
  local other_veh_wp_dir = nil
  
  --If lane lines exist near our vehicles then predict using them
  --for the best accuracy
  if my_veh_side ~= nil and my_veh_side ~= "both" then
    my_veh_pos_future.z = my_veh_props.center_pos.z
    my_veh_pos_future, my_veh_wp_dir = getFutureVehPosCorrectedWithWP(my_veh, my_veh, my_veh_pos_future, my_lat_dist_from_wp, my_veh_side)
    my_veh_pos_future.z = 0
    
    other_veh_pos_future.z = other_veh_props.center_pos.z
    other_veh_pos_future, other_veh_wp_dir = getFutureVehPosCorrectedWithWP(my_veh, other_veh, other_veh_pos_future, other_lat_dist_from_wp, my_veh_side)
    other_veh_pos_future.z = 0
  end
 

  --Calculate the bounding boxes of both vehicles

  --My BB
  -- width, length, height
  local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, my_veh_wp_dir, distance)

  --Other BB
  -- width, length, height
  local other_x, other_y, other_z = getOtherVehBoundingBox(my_veh_props, other_veh_wp_dir, distance)

  
  --Check for overlap between both bounding boxes
  local overlap = overlapsOBB_OBB(my_veh_pos_future, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)

  if overlap then
    --At low speeds, predict if moderate steering input (0.3 g's laterally) can avoid collision
    --then deactivate system
    if my_veh_props.speed < 15 then
      if math.abs(my_veh_props.acceleration.x) > 0.1 * 9.81 then
      
        local turning_acc_vec = vec3(my_veh_props.acceleration)
        
        --Going left
        if my_veh_props.acceleration.x > 0 then
          turning_acc_vec.x = turning_acc_vec.x + aeb_params.lateral_acc_to_avoid_collision * 9.81    
        else
          turning_acc_vec.x = turning_acc_vec.x - aeb_params.lateral_acc_to_avoid_collision * 9.81   
        end
      
        local my_veh_pos_future = extra_utils.getFuturePositionXYWithAcc(my_veh, ttc, turning_acc_vec, "front")
      
        local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, nil, distance)
      
        local overlap2 = overlapsOBB_OBB(my_veh_pos_future, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)
      
        my_veh_pos_future.z = my_veh_props.center_pos.z
        
        debugDrawer:drawSphere((my_veh_pos_future):toPoint3F(), 0.5, ColorF(1,0,1,1))

        return overlap2
      
        --[[
        local turning_acc_vec_left = vec3(my_veh_props.acceleration)
        turning_acc_vec_left.x = turning_acc_vec_left.x + aeb_params.lateral_acc_to_avoid_collision * 9.81
            
        local turning_acc_vec_right = vec3(my_veh_props.acceleration)
        turning_acc_vec_right.x = turning_acc_vec_right.x - aeb_params.lateral_acc_to_avoid_collision * 9.81
            
        local my_veh_pos_future_left = extra_utils.getFuturePositionXYWithAcc(my_veh, ttc, turning_acc_vec_left, "front")
        local my_veh_pos_future_right = extra_utils.getFuturePositionXYWithAcc(my_veh, ttc, turning_acc_vec_right, "front")
      
        local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, nil, distance)
      
        local overlap_left = overlapsOBB_OBB(my_veh_pos_future_left, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)
        local overlap_right = overlapsOBB_OBB(my_veh_pos_future_right, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)
      
        my_veh_pos_future_left.z = my_veh_props.center_pos.z
        my_veh_pos_future_right.z = my_veh_props.center_pos.z
      
        debugDrawer:drawSphere((my_veh_pos_future_left):toPoint3F(), 0.5, ColorF(1,0,1,1))
        debugDrawer:drawSphere((my_veh_pos_future_right):toPoint3F(), 0.5, ColorF(1,0,1,1))
      
        return overlap_left or overlap_right
        
        ]]--
      else
        local turning_acc_vec_left = vec3(0.15 * 9.81, 0, 0)
        local turning_acc_vec_right = vec3(-0.15 * 9.81, 0, 0)

        local my_veh_pos_future_left = extra_utils.getFuturePositionXYWithAcc(my_veh, ttc, turning_acc_vec_left, "front")
        local my_veh_pos_future_right = extra_utils.getFuturePositionXYWithAcc(my_veh, ttc, turning_acc_vec_right, "front")
      
        local my_x, my_y, my_z = getMyVehBoundingBox(my_veh_props, nil, distance)
      
        local overlap_left = overlapsOBB_OBB(my_veh_pos_future_left, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)
        local overlap_right = overlapsOBB_OBB(my_veh_pos_future_right, my_x, my_y, my_z, other_veh_pos_future, other_x, other_y, other_z)
      
        my_veh_pos_future_left.z = my_veh_props.center_pos.z
        my_veh_pos_future_right.z = my_veh_props.center_pos.z
      
        debugDrawer:drawSphere((my_veh_pos_future_left):toPoint3F(), 0.5, ColorF(1,0,1,1))
        debugDrawer:drawSphere((my_veh_pos_future_right):toPoint3F(), 0.5, ColorF(1,0,1,1))
      
        return overlap_left and overlap_right  
      end
    end
  end
  
  return overlap
end

local function getNearestVehicleInPath(dt, my_veh, other_vehs_data, in_reverse)
  local distance = 9999
  local rel_vel = 0
  local curr_veh_in_path = nil

  --If table is empty then return
  if next(other_vehs_data) == nil then
    return {distance, rel_vel}
  end

  local my_veh_velocity = vec3(my_veh:getVelocity())

  --Analyze the trajectory of other vehicles with my trajectory
  --to see if collision imminent
  for _, other_veh_data in pairs(other_vehs_data) do
    local other_veh = other_veh_data[1]
    local this_distance = other_veh_data[2]
    local my_lat_dist_from_wp = other_veh_data[3]
    local other_lat_dist_from_wp = other_veh_data[4]
    local my_veh_side = other_veh_data[5]

    local other_veh_velocity = vec3(other_veh:getVelocity())

    local this_rel_vel = (my_veh_velocity - other_veh_velocity):length()

    local overlap_at_ttc = checkIfCarsIntersectAtTTC(dt, my_veh, other_veh, this_distance, my_lat_dist_from_wp, other_lat_dist_from_wp, my_veh_side)

    --Collision may be possible
    if overlap_at_ttc then
      --If this distance is less than current min distance
      --then this is new min distance
      if this_distance <= distance then
        distance = this_distance
        rel_vel = this_rel_vel

        curr_veh_in_path = other_veh
      end
    end
  end

  return {distance, rel_vel}
end


local timeElapsed3 = 0
local i3 = 0

local function performEmergencyBraking(dt, my_veh, distance, vel_rel, in_reverse)
  local my_veh_props = extra_utils.getVehicleProperties(my_veh)

  if system_active and my_veh_props.speed < aeb_params.brake_till_stop_speed then
    my_veh:queueLuaCommand("input.event('brake', 1, -1)")
    return
  end

  --debugDrawer:drawLine(vec3(veh:getPosition()):toPoint3F(), (vec3(veh:getPosition()) + other_veh_velocity * 10):toPoint3F() , ColorF(1,0,0,1))

  --local acceleration = veh_accs[my_veh:getID()]
  --if acceleration == nil then return end

  local acc = 9.81-- -acceleration[3]

  --You can stop faster forward than in reverse
  if in_reverse == 0 then
    acc = acc * aeb_params.fwd_friction_coeff
    --acc = math.sqrt(acc * acc - acceleration[1] * acceleration[1])
  else
    acc = acc * aeb_params.rev_friction_coeff
  end

  --Calculate TTC
  local ttc = distance / vel_rel
  local time_to_brake = vel_rel / (2 * acc)

  --leeway time depending on speed
  local time_before_braking = ttc - time_to_brake - aeb_params.braking_time_leeway --* math.min(math.max(0.75, vel_rel / 15.0), 2)

  debugDrawer:drawTextAdvanced((my_veh_props.front_pos + my_veh_props.dir * 2):toPoint3F(), String("Time till braking: " .. tostring(time_before_braking)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  --print(time_before_braking)

  timeElapsed3 = timeElapsed3 + dt

  --Sound warning tone if 1.0 seconds away from braking
  if time_before_braking <= 1.0 then -- * math.min(math.max(1, vel_rel / 15.0), 2)  then
    if timeElapsed3 >= 1.0 / aeb_params.parking_fwd_warning_tone_hertz then
      Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')
      timeElapsed3 = 0
  end
  end

  --Maximum Braking
  if time_before_braking <= 0  then
    my_veh:queueLuaCommand("input.event('brake', 1, -1)")
    system_active = true

    i3 = 0
  else
    i3 = i3 + 1

    if i3 >= 15 then
      if system_active then
        my_veh:queueLuaCommand("input.event('brake', 0, -1)")
        system_active = false
      end
    end
  end
end

local function releaseBrakeOnThrottleAfterEmergencyBraking(veh)
  --Release brake if user presses throttle
  if throttle_pos ~= nil then
    if system_active and stopped and throttle_pos > 0 then
      veh:queueLuaCommand("input.event('brake', 0, -1)")
      system_active = false
    end
  end
end


local function update(dt, veh)
  local the_veh_name = veh:getJBeamFilename()
  local veh_speed = vec3(veh:getVelocity()):length()
  local in_reverse = electrics_values["reverse"]
  local gear_selected = electrics_values["gear"]
  local esc_color = electrics_values["dseColor"]

  stopped = veh_speed <= 0.1

  --Release brakes if system was activated before and user presses throttle
  releaseBrakeOnThrottleAfterEmergencyBraking(veh)

  --ESC must be in comfort mode, otherwise it is deactivated
  --only sunburst uses different color for comfort mode
  if (the_veh_name == "sunburst" and esc_color == "98FB00")
    or (the_veh_name ~= "sunburst" and esc_color == "238BE6")
  then
    --Poll front sensors if not in reverse or poll rear sensors when in reverse
    if in_reverse == nil or gear_selected == nil
      or gear_selected == 'P' or gear_selected == 0 then return end

    --If vehicle is traveling >= 144km/h (40 m/s) or stopped deactivate system
    if veh_speed > aeb_params.max_speed or veh_speed <= aeb_params.min_speed then return end

    local other_vehs_data = extra_utils.getNearbyVehiclesInSameLane(veh, aeb_params.vehicle_search_radius, angular_speed, aeb_params.min_distance_from_car, true)

    --returns distance, relative velocity
    local data = getNearestVehicleInPath(dt, veh, other_vehs_data, in_reverse)

    local distance = data[1]
    local vel_rel = data[2]

    performEmergencyBraking(dt, veh, distance, vel_rel, in_reverse)
  end
end

M.update = update

return M
