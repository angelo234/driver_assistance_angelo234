local M = {}

--local p = LuaProfiler("my profiler")

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

--For efficiency
local max = math.max
local min = math.min
local sin = math.sin
local cos = math.cos
local asin = math.asin
local acos = math.acos
local atan2 = math.atan2
local pi = math.pi
local abs = math.abs
local sqrt = math.sqrt
local floor = math.floor
local ceil = math.ceil

local front_static_sensor_id = -1
local front_static_prev_min_dist = 9999
local front_static_min_dist = 9999

local rear_static_sensor_id = -1
local rear_static_prev_min_dist = 9999
local rear_static_min_dist = 9999

local past_wps_props_table = {}

--Returns a table of vehicles and distance to them within a max_dist radius
local function getNearbyVehicles(dt, my_veh_props, max_dist, in_front)

  local other_vehs = {}

  for i = 0, be:getObjectCount() - 1 do
    local other_veh = be:getObject(i)

    if other_veh:getJBeamFilename() ~= "unicycle" then
      local other_veh_props = extra_utils.getVehicleProperties(other_veh)

      if other_veh_props.id ~= my_veh_props.id then
        --Get aproximate distance first between vehicles and return if less than max dist
        local other_bb = other_veh_props.bb

        local front_dist = (my_veh_props.front_pos - other_veh_props.center_pos):length()
        local rear_dist = (my_veh_props.rear_pos - other_veh_props.center_pos):length()

        --If rear distance is larger than front distance, then vehicle is in front
        if front_dist < max_dist and front_dist < rear_dist and in_front then
          local ray_cast_dist = castRayStatic(my_veh_props.front_pos, (other_veh_props.center_pos - my_veh_props.front_pos):normalized(), max_dist)

          --Freepath to vehicle?
          if ray_cast_dist > front_dist then
            local cir_dist = extra_utils.getCircularDistance(my_veh_props, other_veh_props) - (my_veh_props.velocity * dt):length()

            local other_veh_data =
            {
              other_veh = other_veh,
              other_veh_props = other_veh_props,
              distance = cir_dist,
              shortest_dist = front_dist
            }

            table.insert(other_vehs, other_veh_data)
          end

          --If front distance is larger than rear distance, then vehicle is in rear
        elseif rear_dist < max_dist and front_dist > rear_dist and not in_front then
          local ray_cast_dist = castRayStatic(my_veh_props.rear_pos, (other_veh_props.center_pos - my_veh_props.rear_pos):normalized(), max_dist)

          --Freepath to vehicle?
          if ray_cast_dist > rear_dist then
            local dist = extra_utils.getStraightDistance(my_veh_props, other_veh_props, false, true) - (my_veh_props.velocity * dt):length()

            local other_veh_data =
            {
              other_veh = other_veh,
              other_veh_props = other_veh_props,
              distance = dist,
              shortest_dist = rear_dist
            }

            table.insert(other_vehs, other_veh_data)
          end
        end
      end
    end
  end
  return other_vehs
end



--Returns a table of vehicles and distance to them within a max_dist radius and on same road
local function getNearbyVehiclesOnSameRoad(dt, my_veh_props, max_dist, other_vehs_data, only_pos_rel_vel, last_vehs_table)
  local other_vehs_in_my_lane = {}

  local my_veh_wps_props = extra_utils.getWaypointStartEndAdvanced(my_veh_props, my_veh_props, my_veh_props.front_pos, past_wps_props_table[my_veh_props.id])

  past_wps_props_table[my_veh_props.id] = my_veh_wps_props

  if my_veh_wps_props == nil then
    return {}, nil
  end

  for _, other_veh_data in pairs(other_vehs_data) do
    local other_veh_props = other_veh_data.other_veh_props
    local speed_rel = my_veh_props.speed - other_veh_props.speed

    local other_veh_wps_props = extra_utils.getWaypointStartEndAdvanced(my_veh_props, other_veh_props, other_veh_props.front_pos, past_wps_props_table[other_veh_props.id])

    past_wps_props_table[other_veh_props.id] = other_veh_wps_props

    other_veh_data.my_veh_wps_props = my_veh_wps_props
    other_veh_data.other_veh_wps_props = other_veh_wps_props

    --debugDrawer:drawTextAdvanced((other_veh_props.front_pos), String("Free path? " .. tostring(free_path_to_veh)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

    local on_same_road = extra_utils.checkIfOtherCarOnSameRoad(my_veh_props, other_veh_props, my_veh_wps_props)

    if on_same_road then
      --debugDrawer:drawTextAdvanced((other_veh_props.front_pos), String("On same road"),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

      --In same road and my vehicle speed is >= to other
      if on_same_road then
        if only_pos_rel_vel then
          if speed_rel >= 0 then
            --other_veh_data.timer = 0.1
            table.insert(other_vehs_in_my_lane, other_veh_data)
          end
        else
          --other_veh_data.timer = 0.1
          table.insert(other_vehs_in_my_lane, other_veh_data)
        end
      end
    end
  end

  return other_vehs_in_my_lane
end

local function yawSensor(veh_props, init_dir, system_params)
  if electrics_values_angelo234["steering_input"] ~= nil and electrics_values_angelo234["steering_input"] ~= 0 then
    --Sagitta
    local s = 0.05

    --Using steering angle to point sensors
    local avg_radius = (system_params.max_steer_radius + system_params.min_steer_radius) / 2

    local r = avg_radius / electrics_values_angelo234["steering_input"]

    local turning_right = true

    if r < 0 then
      turning_right = false
    end

    r = abs(r)

    --Raycast distance
    local d = 2 * sqrt(s * (-s + 2 * r))

    local angle = 2 * asin(d / (2 * r))

    --Set angle negative if turning left
    if not turning_right then
      angle = -angle
    end

    local dir = init_dir * cos(angle) + veh_props.dir_right * sin(angle)

    return dir
  else
    return init_dir
  end
end

--Used to point sensors to horizontal regardless of pitch of car (e.g. accelearting and braking)
local function pitchSensor(veh_props)
  local height_front = castRayStatic(veh_props.front_pos, -veh_props.dir_up, 2)
  local height_rear = castRayStatic(veh_props.rear_pos, -veh_props.dir_up, 2)

  -- +x = need to lower, -x = need to raise
  local diff = height_front - height_rear

  local result_dir = (veh_props.front_pos - (veh_props.rear_pos + veh_props.dir_up * diff)):normalized()

  return result_dir
end

local function staticCastRay(veh_props, sensor_pos, sensor_dir)
  local dest = sensor_dir * 10 + sensor_pos
  --local dest = veh_props.dir * aeb_params.sensor_max_distance + pos

  --use castRayDebug to show lines
  local hit = castRay(sensor_pos, dest, true, true)

  if hit == nil then return nil end

  return {hit.norm, hit.dist, hit.pt}
end

local function processRayCasts(veh_props, static_hit)
  --static hit returns {hit.norm, hit.dist, hit.pt, static_sensor_id}
  --vehicle hit returns {other_veh, min_distance, veh_sensor_id}

  local static_dist = 9999
  if static_hit ~= nil then
    local norm = static_hit[1]
    local distance = static_hit[2]

    local new_dir_x = sqrt(veh_props.dir.x * veh_props.dir.x
    + veh_props.dir.y * veh_props.dir.y)

    local new_norm_x = sqrt(norm.x * norm.x + norm.y * norm.y)

    local dir_xy = vec3(new_dir_x, veh_props.dir.z, 0)
    local norm_xy = vec3(new_norm_x, norm.z, 0)

    dir_xy = dir_xy:normalized()
    norm_xy = norm_xy:normalized()

    local angle = acos(dir_xy:dot(norm_xy))
    --print(angle * 180.0 / pi)

    --If surface is < 70 degrees then count it as obstacle
    if angle < 70 * pi / 180.0 then
      static_dist = static_hit[2]
    end
  end

  return static_dist
end

local function setFrontSensorID(aeb_params)
  if front_static_sensor_id >= aeb_params.num_of_sensors - 1 then front_static_sensor_id = -1 end

  front_static_sensor_id = front_static_sensor_id + 1
end

local function pollFrontStaticSensors(dt, veh_props, system_params, aeb_params)
  --p:start()

  local car_half_width = veh_props.bb:getHalfExtents().x * 0.75

  setFrontSensorID(aeb_params)

  local sensor_pos = veh_props.front_pos + veh_props.dir_up * aeb_params.parking_sensor_rel_height + veh_props.dir * aeb_params.sensor_offset_forward
  + veh_props.dir_right * (car_half_width - car_half_width / ((aeb_params.num_of_sensors - 1) / 2.0) * front_static_sensor_id)
  + (aeb_params.num_of_sensors / aeb_params.sensors_polled_per_iteration) * veh_props.velocity * dt

  --Set sensor in proper direction
  local sensor_dir = pitchSensor(veh_props)

  --p:add("pitchSensor")

  sensor_dir = yawSensor(veh_props, sensor_dir, system_params)
  --p:add("yawSensor")

  --Do static object raycasting
  local static_hit = staticCastRay(veh_props, sensor_pos, sensor_dir)

  --p:add("staticCastRay")

  local min_dist = processRayCasts(veh_props, static_hit)

  --p:add("processRayCasts")

  --min_dist = min_dist - aeb_params.sensor_offset_forward - 0.2

  --p:finish(true)

  return min_dist
end

local function setRearSensorID(rev_aeb_params)
  if rear_static_sensor_id >= rev_aeb_params.num_of_sensors - 1 then rear_static_sensor_id = -1 end

  rear_static_sensor_id = rear_static_sensor_id + 1
end

local function pollRearStaticSensors(dt, veh_props, rev_aeb_params)
  local parking_sensor_height = rev_aeb_params.parking_sensor_rel_height
  local car_half_width = veh_props.bb:getHalfExtents().x * 0.75

  setRearSensorID(rev_aeb_params)

  local sensorPos = veh_props.rear_pos + veh_props.dir_up * parking_sensor_height + veh_props.dir * rev_aeb_params.sensor_offset_forward
  + veh_props.dir_right * (car_half_width - car_half_width / ((rev_aeb_params.num_of_sensors - 1) / 2.0) * rear_static_sensor_id)
  + veh_props.dir * -rev_aeb_params.sensor_offset_forward
  + (rev_aeb_params.num_of_sensors / rev_aeb_params.sensors_polled_per_iteration) * veh_props.velocity * dt

  local static_hit = staticCastRay(veh_props, sensorPos, -veh_props.dir)
  local min_dist = processRayCasts(veh_props, static_hit)

  min_dist = min_dist - rev_aeb_params.sensor_offset_forward-- - 0.15

  return min_dist
end



local last_vehs_in_same_road_in_front_table = nil

local function pollFrontSensors(dt, veh_props, system_params, aeb_params)
  --p:start()

  --Cast rays for static objects
  for i = 1, aeb_params.sensors_polled_per_iteration do
    if front_static_sensor_id == aeb_params.num_of_sensors - 1 then
      front_static_prev_min_dist = front_static_min_dist
      front_static_min_dist = 9999
    end

    --Get distance and other data of nearest obstacle
    local front_static_dist = pollFrontStaticSensors(dt, veh_props, system_params, aeb_params)

    front_static_min_dist = min(front_static_dist, front_static_min_dist)
  end

  --p:add("cast rays")

  --Get nearby vehicles
  local other_vehs_data = getNearbyVehicles(dt, veh_props, aeb_params.vehicle_search_radius, true)
  --p:add("getNearbyVehicles")


  local other_vehs_same_road_data = getNearbyVehiclesOnSameRoad(dt, veh_props, aeb_params.vehicle_search_radius,
  other_vehs_data, false, last_vehs_in_same_road_in_front_table)
  --p:add("getNearbyVehiclesOnSameRoad")


  last_vehs_in_same_road_in_front_table = other_vehs_same_road_data

  --p:finish(true)

  return {front_static_min_dist, other_vehs_data, other_vehs_same_road_data}
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

local function processRearSensorData(rear_static_min_dist, other_vehs_data, rev_aeb_params)
  local vehicle_hit = getClosestVehicle(other_vehs_data)

  local vehicle_dist = 9999
  local other_veh = nil

  if vehicle_hit[1] ~= nil then
    other_veh = vehicle_hit[1]
    vehicle_dist = vehicle_hit[2]
  end

  local min_dist = math.min(rear_static_min_dist, vehicle_dist)

  --min_dist = min_dist - rev_aeb_params.sensor_offset_forward - 0.15

  return {other_veh, min_dist}
end

local function pollRearSensors(dt, veh_props, system_params, rev_aeb_params)
  --Cast rays for static objects
  for i = 1, rev_aeb_params.sensors_polled_per_iteration do
    if rear_static_sensor_id == rev_aeb_params.num_of_sensors - 1 then
      rear_static_prev_min_dist = rear_static_min_dist
      rear_static_min_dist = 9999
    end

    local rear_static_dist = pollRearStaticSensors(dt, veh_props, rev_aeb_params)

    rear_static_min_dist = min(rear_static_dist, rear_static_min_dist)
  end

  --Get nearby vehicles
  local other_vehs_data = getNearbyVehicles(dt, veh_props, rev_aeb_params.sensor_max_distance, false)

  local data = processRearSensorData(rear_static_min_dist, other_vehs_data, rev_aeb_params)

  return data
end

M.pollFrontSensors = pollFrontSensors
M.pollRearSensors = pollRearSensors

return M