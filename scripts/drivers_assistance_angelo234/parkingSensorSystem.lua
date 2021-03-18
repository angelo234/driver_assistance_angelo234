local M = {}

local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')
local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')

local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local static_sensor_id = -1

local function castRay(sensorPos, dir, max_distance, rightDir, veh_name, same_ray, speed)
  local hit = nil

  local car_half_width = params_per_veh[veh_name].safety_offset_width_sensor + params_per_veh[veh_name].veh_half_width

  if not same_ray then
    if static_sensor_id >= 2 then static_sensor_id = -1 end
    static_sensor_id = static_sensor_id + 1
  end

  sensorPos = sensorPos + rightDir * (car_half_width - car_half_width * static_sensor_id)

  local dest = dir * max_distance + sensorPos

  hit = castRayDebug(sensorPos, dest, true, true)

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

  return {other_veh, min_dist, vehicle_hit[3]}
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

local function pollReverseSensors(dt, veh)
  local veh_name = veh:getJBeamFilename()

  local veh_vel = vec3(veh.obj:getVelocity())
  local veh_speed = veh_vel:length()

  --How far to place sensor towards car
  local sensor_offset_forward = 0.2

  local sensorPos = vec3(veh:getSpawnWorldOOBBRearPoint())
  local carDir = vec3(veh.obj:getDirectionVector()):normalized()
  local carDirUp = vec3(veh.obj:getDirectionVectorUp()):normalized()
  local carDirRight = carDir:cross(carDirUp):normalized()

  --Fixes lag of sensorPos
  sensorPos = sensorPos + veh_vel * dt

  timeElapsed2 = timeElapsed2 + dt

  local max_raycast_distance = 0.5 + parking_lines_params.parking_line_offset_long + parking_lines_params.parking_line_total_len

  local parking_sensor_height = params_per_veh[veh_name].parking_sensor_rel_height

  sensorPos = sensorPos + carDirUp * parking_sensor_height + carDir * sensor_offset_forward

  local static_hit = castRay(sensorPos, -carDir, max_raycast_distance, carDirRight, veh_name, false, veh_speed)
  --local vehicle_hit = vehicleCastRay(veh:getID(), max_raycast_distance, sensorPos, -carDir, carDirRight, veh_name, false, veh_speed)

  --Get vehicles in a 10m radius behind my vehicle
  local other_vehs_data = extra_utils.getNearbyVehicles(veh, 10, angular_speed, 0, false)
  local vehicle_hit = getClosestVehicle(other_vehs_data)

  local processedRayCast = processRayCasts(static_hit, vehicle_hit)

  local other_veh = processedRayCast[1]
  local min_dist = processedRayCast[2]
  local sensor_id = processedRayCast[3]

  min_dist = min_dist - sensor_offset_forward

  if min_dist <= parking_lines_params.parking_line_total_len + parking_lines_params.parking_line_offset_long then
    if min_dist <= parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_offset_long then
      if timeElapsed2 >= 1.0 / aeb_params.parking_warning_tone_hertz then
        Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')
        timeElapsed2 = 0
      end
    else
      if timeElapsed2 >= min_dist / aeb_params.parking_warning_tone_dist_per_hertz then
        Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')
        timeElapsed2 = 0
      end
    end
  end

  return {other_veh, min_dist, nil, sensor_id}
end

local function update(dt, veh)
  local the_veh_name = veh:getJBeamFilename()
  local veh_speed = vec3(veh:getVelocity()):length()
  local in_reverse = electrics_values["reverse"]
  local gear_selected = electrics_values["gear"]

  --Poll front sensors if not in reverse or poll rear sensors when in reverse
  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then return end

  --If vehicle is traveling >= 144km/h (40 m/s) or stopped deactivate system
  --if veh_speed > aeb_params.max_speed or veh_speed <= aeb_params.min_speed then return end

  pollReverseSensors(dt, veh)
end

M.update = update

return M
