local M = {}

local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')
local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')

local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local static_sensor_id = -1
local prev_min_dist = 9999
local min_dist = 9999

local system_active = false

local function castRay(sensorPos, dir, max_distance, rightDir, veh_name, same_ray, speed)
  local hit = nil

  local car_half_width = params_per_veh[veh_name].safety_offset_width_sensor + params_per_veh[veh_name].veh_half_width

  if not same_ray then
    if static_sensor_id >= 4 then static_sensor_id = -1 end
    static_sensor_id = static_sensor_id + 1
  end

  local pos = sensorPos + rightDir * (car_half_width - car_half_width / 2.0 * static_sensor_id)

  local dest = dir * max_distance + pos

  hit = castRayDebug(pos, dest, true, true)

  if hit == nil then return nil end

  return {hit.norm, hit.dist, hit.pt, static_sensor_id}
end

local function processRayCasts(static_hit, vehicle_hit)
  --static hit returns {hit.norm, hit.dist, hit.pt, static_sensor_id}
  --vehicle hit returns {other_veh, min_distance, veh_sensor_id}

  local static_dist = 9999
  local vehicle_dist = 9999
  local other_veh = nil

  if static_hit ~= nil then
    local norm = static_hit[1]

    local norm_x = math.abs(norm.x)
    local norm_y = math.abs(norm.y)

    local distance = static_hit[2]

    --If surface is 45 degrees or more then count it
    if norm_x >= 0.5 or norm_y >= 0.5 then
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
    local veh = other_veh_data[1]
    local this_distance = other_veh_data[2]

    if this_distance <= distance then
      distance = this_distance
      other_veh = veh
    end
  end

  --print(distance)

  return {other_veh, distance}
end

local timeElapsed2 = 0

local function soundBeepers(dt, dist)
  timeElapsed2 = timeElapsed2 + dt

  if dist <= parking_lines_params.parking_line_total_len + parking_lines_params.parking_line_offset_long then
    
    --If object is within red line distance, play constant tone
    if dist <= parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_offset_long then
      if timeElapsed2 >= 1.0 / aeb_params.parking_warning_tone_hertz then
        Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')
        timeElapsed2 = 0
      end
      
    --Else tone depends on distance
    else
      if timeElapsed2 >= dist / aeb_params.parking_warning_tone_dist_per_hertz then
        Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')
        timeElapsed2 = 0
      end
    end
  end
end

local function pollReverseSensors(dt, veh)
  local veh_name = veh:getJBeamFilename()

  local my_veh_props = extra_utils.getVehicleProperties(veh)

  --How far to place sensor towards car
  local sensor_offset_forward = 0.2
  local parking_sensor_height = params_per_veh[veh_name].parking_sensor_rel_height

  --Fixes lag of sensorPos
  local sensorPos = my_veh_props.rear_pos + 5 * my_veh_props.velocity * dt
    + my_veh_props.dir_up * parking_sensor_height + my_veh_props.dir * sensor_offset_forward
  
  local max_raycast_distance = 0.5 + parking_lines_params.parking_line_offset_long + parking_lines_params.parking_line_total_len

  local static_hit = castRay(sensorPos, -my_veh_props.dir, max_raycast_distance, my_veh_props.dir_right, veh_name, false, my_veh_props.speed)
  --local vehicle_hit = vehicleCastRay(veh:getID(), max_raycast_distance, sensorPos, -carDir, carDirRight, veh_name, false, veh_speed)

  --Get vehicles in a 10m radius behind my vehicle
  local other_vehs_data = extra_utils.getNearbyVehicles(veh, 10, angular_speed_angelo234, 0, false)
  local vehicle_hit = getClosestVehicle(other_vehs_data)

  local other_veh, min_dist = processRayCasts(static_hit, vehicle_hit)

  min_dist = min_dist - sensor_offset_forward - 0.1

  return other_veh, min_dist
end

local function performEmergencyBraking(dt, my_veh, distance)
  local my_veh_props = extra_utils.getVehicleProperties(my_veh)

  --Maximum Braking
  if system_active then
    my_veh:queueLuaCommand("input.event('brake', 1, -1)")
    return
  end
  
  --Max braking acceleration = gravity * coefficient of static friction
  local acc = aeb_params.gravity * 0.8

  --Calculate TTC
  local ttc = distance / my_veh_props.speed
  local time_to_brake = my_veh_props.speed / (2 * acc)

  --leeway time depending on speed
  local time_before_braking = ttc - time_to_brake

  if time_before_braking <= 0 then
    system_active = true
  end
end

local function update(dt, veh)
  local the_veh_name = veh:getJBeamFilename()
  local veh_speed = vec3(veh:getVelocity()):length()
  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then 
    prev_min_dist = 9999
    min_dist = 9999
    return 
  end
  
  --Get distance and other data of nearest obstacle 
 local other_veh, dist = pollReverseSensors(dt, veh)
   
  min_dist = math.min(dist, min_dist)
  
  --Play beeping sound depending on min distance of prev five sensor detections to obstacle
  soundBeepers(dt, prev_min_dist)
  
  if static_sensor_id == 4 then
    prev_min_dist = min_dist
    min_dist = 9999
  end
  
  --If vehicle is stopped then deactivate system
  if veh_speed <= aeb_params.min_speed then 
    system_active = false
    return 
  end  
  
  performEmergencyBraking(dt, veh, dist)
end

M.update = update

return M
