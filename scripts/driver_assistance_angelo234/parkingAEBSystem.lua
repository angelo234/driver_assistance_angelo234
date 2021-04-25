local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local rev_aeb_on = true

local system_active = false

local static_sensor_id = -1
local prev_min_dist = 9999
local min_dist = 9999

local function toggleSystem()
  rev_aeb_on = not rev_aeb_on
  
  local msg = nil
  
  if rev_aeb_on then
    msg = "ON"
  else
    msg = "OFF"
  end
  
  ui_message("Reverse AEB switched " .. msg)
end

local function staticCastRay(veh_props, sensorPos, same_ray, parking_sensor_params)
  local hit = nil

  local car_half_width = veh_props.bb:getHalfExtents().x - 0.3

  if not same_ray then
    if static_sensor_id >= parking_sensor_params.num_of_sensors - 1 then static_sensor_id = -1 end
    static_sensor_id = static_sensor_id + 1
  end

  local pos = sensorPos + veh_props.dir_right * (car_half_width - car_half_width / ((parking_sensor_params.num_of_sensors - 1) / 2.0) * static_sensor_id)

  local dest = -veh_props.dir * parking_sensor_params.sensor_max_distance + pos

  --use castRayDebug to show lines
  hit = castRayDebug(pos, dest, true, true)

  if hit == nil then return nil end

  return {hit.norm, hit.dist, hit.pt, static_sensor_id}
end

local function processRayCasts(veh_props, static_hit, vehicle_hit)
  --static hit returns {hit.norm, hit.dist, hit.pt, static_sensor_id}
  --vehicle hit returns {other_veh, min_distance, veh_sensor_id}

  local static_dist = 9999
  local vehicle_dist = 9999
  local other_veh = nil

  if static_hit ~= nil then
    local norm = static_hit[1]
    local distance = static_hit[2]

    local new_dir_x = math.sqrt(veh_props.dir.x * veh_props.dir.x 
    + veh_props.dir.y * veh_props.dir.y)
    
    local new_norm_x = math.sqrt(norm.x * norm.x + norm.y * norm.y)

    local dir_xy = vec3(new_dir_x, veh_props.dir.z, 0)
    local norm_xy = vec3(new_norm_x, norm.z, 0)

    dir_xy = dir_xy:normalized()
    norm_xy = norm_xy:normalized()
     
    local angle = math.acos(dir_xy:dot(norm_xy))
    --print(angle * 180.0 / math.pi)
    
    --If surface is < 70 degrees then count it as obstacle
    if angle < 70 * math.pi / 180.0 then
      static_dist = static_hit[2]
    end
  end

  if vehicle_hit[1] ~= nil then
    other_veh = vehicle_hit[1]
    vehicle_dist = vehicle_hit[2]
  end

  local min_dist = math.min(static_dist, vehicle_dist)

  --Vehicle is closest
  --if min_dist ~= 9999 and min_dist == vehicle_dist then

  --end

  return other_veh, min_dist
end


local function getClosestVehicle(other_vehs_data)
  local distance = 9999
  local other_veh = nil

  for _, other_veh_data in pairs(other_vehs_data) do
    local veh = other_veh_data.other_veh
    local this_distance = other_veh_data.distance

    if this_distance <= distance then
      distance = this_distance
      other_veh = veh
    end
  end

  --print(distance)

  return {other_veh, distance}
end


local function pollReverseSensors(dt, my_veh_props, rev_aeb_params)
  local parking_sensor_height = rev_aeb_params.parking_sensor_rel_height

  --Fixes lag of sensorPos
  local sensorPos = my_veh_props.rear_pos + (rev_aeb_params.num_of_sensors / rev_aeb_params.sensors_polled_per_iteration) * my_veh_props.velocity * dt
    + my_veh_props.dir_up * parking_sensor_height + my_veh_props.dir * rev_aeb_params.sensor_offset_forward
  
  local static_hit = staticCastRay(my_veh_props, sensorPos, false, rev_aeb_params)
  --local vehicle_hit = vehicleCastRay(veh:getID(), max_raycast_distance, sensorPos, -carDir, carDirRight, veh_name, false, veh_speed)

  --Get vehicles behind my vehicle
  local other_vehs_data = extra_utils.getNearbyVehicles(my_veh_props, rev_aeb_params.sensor_max_distance, 0, false)

  local vehicle_hit = getClosestVehicle(other_vehs_data)

  local other_veh, min_dist = processRayCasts(my_veh_props, static_hit, vehicle_hit)

  min_dist = min_dist - rev_aeb_params.sensor_offset_forward - 0.15

  return other_veh, min_dist
end

local beeper_timer = 0

local function soundBeepers(dt, dist, parking_lines_params, beeper_params)
  beeper_timer = beeper_timer + dt

  dist = dist + 0.2

  if dist <= parking_lines_params.parking_line_total_len + parking_lines_params.parking_line_offset_long then
    
    --If object is within red line distance, play constant tone
    if dist <= parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_offset_long then
      if beeper_timer >= 1.0 / beeper_params.parking_warning_tone_hertz then
        Engine.Audio.playOnce('AudioGui','art/sound/proximity_tone_50ms_moderate.wav')
        beeper_timer = 0
      end
      
    --Else tone depends on distance
    else
      if beeper_timer >= dist / beeper_params.parking_warning_tone_dist_per_hertz then
        Engine.Audio.playOnce('AudioGui','art/sound/proximity_tone_50ms_moderate.wav')
        beeper_timer = 0
      end
    end
  end
end

local function performEmergencyBraking(dt, veh, distance, speed, system_params, rev_aeb_params)
  --Max braking acceleration = gravity * coefficient of static friction
  local acc = system_params.gravity * system_params.rev_friction_coeff

  --Calculate time to collision (TTC)
  local ttc = distance / speed
  local time_to_brake = speed / (2 * acc)

  local time_before_braking = ttc - time_to_brake

  if time_before_braking <= 0 then
    system_active = true
  end
  
  --If vehicle is below certain speed then deactivate system
  if speed <= rev_aeb_params.min_speed then
    --But if system activated before, then release brakes and apply parking brake
    if system_active then
      --When coming to a stop with system activated, release brakes but apply parking brake in arcade mode :P
      if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then       
        veh:queueLuaCommand("input.event('brake', 1, 2)")
      else
        --Release brake and apply parking brake
        veh:queueLuaCommand("input.event('throttle', 0, 2)")
        veh:queueLuaCommand("input.event('parkingbrake', 1, 2)")   
      end
      system_active = false
    end
    return 
  end    

  --Maximum Braking
  if system_active then
    --Must do differently depending on gearbox mode :/
    if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
      veh:queueLuaCommand("input.event('throttle', 0, 2)")
      veh:queueLuaCommand("input.event('brake', 1, 2)")
    else
      veh:queueLuaCommand("input.event('brake', 0, 2)")
      veh:queueLuaCommand("input.event('throttle', 1, 2)")
    end
    return
  end
end

local function update(dt, veh, system_params, parking_lines_params, rev_aeb_params, beeper_params)
  if not rev_aeb_on then return end
  
  local veh_props = extra_utils.getVehicleProperties(veh)
  
  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then
    prev_min_dist = 9999
    min_dist = 9999
    return 
  end

  for i = 1, rev_aeb_params.sensors_polled_per_iteration do
    if static_sensor_id == rev_aeb_params.num_of_sensors - 1 then
      prev_min_dist = min_dist
      min_dist = 9999
    end
  
    --Get distance and other data of nearest obstacle 
    local other_veh, dist = pollReverseSensors(dt, veh_props, rev_aeb_params)
     
    min_dist = math.min(dist, min_dist)
  end

  --Play beeping sound based on min distance of prev sensor detections to obstacle
  soundBeepers(dt, min_dist, parking_lines_params, beeper_params)
  
  performEmergencyBraking(dt, veh, min_dist, veh_props.speed, system_params, rev_aeb_params)
end

M.toggleSystem = toggleSystem
M.update = update

return M