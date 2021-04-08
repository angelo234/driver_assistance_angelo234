local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtilsGE')

local static_sensor_id = -1
local prev_min_dist = 9999
local min_dist = 9999

local system_active = false

local function staticCastRay(veh_props, sensorPos, same_ray, parking_sensor_params)
  local hit = nil

  local car_half_width = parking_sensor_params.safety_offset_width_sensor + veh_props.bb:getHalfExtents().x

  if not same_ray then
    if static_sensor_id >= parking_sensor_params.num_of_sensors - 1 then static_sensor_id = -1 end
    static_sensor_id = static_sensor_id + 1
  end

  local pos = sensorPos + veh_props.dir_right * (car_half_width - car_half_width / ((parking_sensor_params.num_of_sensors - 1) / 2.0) * static_sensor_id)

  local dest = -veh_props.dir * parking_sensor_params.sensor_max_distance + pos

  --use castRayDebug to show lines
  hit = castRay(pos, dest, true, true)

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
    
    --If surface is < 60 degrees then count it as obstacle
    if angle < 60 * math.pi / 180.0 then
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


local function pollReverseSensors(dt, my_veh_props, parking_sensor_params)
  local parking_sensor_height = parking_sensor_params.parking_sensor_rel_height

  --Fixes lag of sensorPos
  local sensorPos = my_veh_props.rear_pos + (parking_sensor_params.num_of_sensors / parking_sensor_params.sensors_polled_per_iteration) * my_veh_props.velocity * dt
    + my_veh_props.dir_up * parking_sensor_height + my_veh_props.dir * parking_sensor_params.sensor_offset_forward
  
  local static_hit = staticCastRay(my_veh_props, sensorPos, false, parking_sensor_params)
  --local vehicle_hit = vehicleCastRay(veh:getID(), max_raycast_distance, sensorPos, -carDir, carDirRight, veh_name, false, veh_speed)

  --Get vehicles in a 7.5m radius behind my vehicle
  local other_vehs_data = extra_utils.getNearbyVehicles(my_veh_props, parking_sensor_params.sensor_max_distance, 0, false)
  local vehicle_hit = getClosestVehicle(other_vehs_data)

  local other_veh, min_dist = processRayCasts(my_veh_props, static_hit, vehicle_hit)

  min_dist = min_dist - parking_sensor_params.sensor_offset_forward - 0.1

  return other_veh, min_dist
end


ve_rev_json_params_angelo234 = nil
ge_rev_aeb_data_angelo234 = "'[9999]'"

local function pollReverseSensorsForVELua(dt)
  if ve_rev_json_params_angelo234 == nil or ve_rev_json_params_angelo234 == 'nil' then return end
  
  local params = jsonDecode(ve_rev_json_params_angelo234)

  local my_veh_id = params[1]
  local parking_sensor_params = params[2]

  local veh = be:getObjectByID(my_veh_id)
  
  if veh == nil then return end

  local veh_props = extra_utils.getVehicleProperties(veh)

  for i = 1, parking_sensor_params.sensors_polled_per_iteration do
    if static_sensor_id == parking_sensor_params.num_of_sensors - 1 then
      prev_min_dist = min_dist
      min_dist = 9999
    end
  
    --Get distance and other data of nearest obstacle 
    local other_veh, dist = pollReverseSensors(dt, veh_props, parking_sensor_params)
     
    min_dist = math.min(dist, min_dist)
  end

  ge_rev_aeb_data_angelo234 = "'" .. jsonEncode({min_dist}) .. "'"
end

local function update(dt)
  if be:getEnabled() then
    pollReverseSensorsForVELua(dt)
  end
end

M.update = update

return M