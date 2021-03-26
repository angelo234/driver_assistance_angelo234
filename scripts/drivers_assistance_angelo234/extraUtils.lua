local M = {}

local function toColorI(colorF)
  return ColorI(colorF.r * 255,
    colorF.g * 255,
    colorF.b * 255,
    colorF.a * 255)
end

local function rotateEuler(x, y, z, q)
  q = q or quat()
  q = quatFromEuler(0, z, 0) * q
  q = quatFromEuler(0, 0, x) * q
  q = quatFromEuler(y, 0, 0) * q
  return q
end

local function circleFrom3Pts(a_vec, b_vec, c_vec)
  local a = Point2F(a_vec.x, a_vec.y)
  local b = Point2F(b_vec.x, b_vec.y)
  local c = Point2F(c_vec.x, c_vec.y)

  local yDelta_a = b.y - a.y
  local xDelta_a = b.x - a.x
  local yDelta_b = c.y - b.y
  local xDelta_b = c.x - b.x
  local center = Point2F(0,0)

  local aSlope = yDelta_a / xDelta_a
  local bSlope = yDelta_b / xDelta_b
  center.x = (aSlope * bSlope * (a.y - c.y) + bSlope * (a.x + b.x)
    - aSlope * (b.x + c.x)) / (2.0 * (bSlope - aSlope))
  center.y = -1.0 * (center.x - (a.x + b.x) / 2.0) / aSlope + (a.y + b.y) / 2.0

  local radius = math.sqrt((center.x - a.x) * (center.x - a.x) + (center.y - a.y) * (center.y - a.y))

  return vec3(center.x, center.y, 0), radius
end

local function toNormXYVec(dir)
  local dir_xy = dir
  dir_xy.z = 0
  dir_xy = dir_xy:normalized()

  return dir_xy
end

local function getVehicleProperties(veh)
  local props = {}

  props.name = veh:getJBeamFilename()

  --Direction vectors of vehicle
  props.dir = vec3(veh.obj:getDirectionVector()):normalized()
  props.dir_up = vec3(veh.obj:getDirectionVectorUp()):normalized()
  props.dir_right = props.dir:cross(props.dir_up):normalized()

  --Bounding box of vehicle
  props.bb = veh:getSpawnWorldOOBB()
  props.center_pos = vec3(props.bb:getCenter())
  props.front_pos = props.center_pos + props.dir * veh:getSpawnAABBRadius()
  props.rear_pos = vec3(veh:getSpawnWorldOOBBRearPoint())

  props.velocity = vec3(veh:getVelocity())
  props.speed = props.velocity:length()

  local acceleration = veh_accs_angelo234[veh:getID()]

  if acceleration == nil then
    acceleration = vec3(0,0,0)
  else
    props.acceleration = vec3(acceleration[1], acceleration[2], 9.81 - acceleration[3])
  end

  return props
end

local function getDistanceBetweenWaypoints(wp_a, wp_b, distance_a, distance_b)
  local path = map.getPath(wp_a, wp_b)
  local d = 0

  for i = 1, #path-1 do
    local a,b = path[i],path[i+1]
    a,b = map.getMap().nodes[a].pos, map.getMap().nodes[b].pos
    d = d + (a-b):length()
  end

  return d + (distance_b - distance_a)
end

local function getFuturePosition(veh, time, rel_car_pos)
  local veh_props = getVehicleProperties(veh)
  local acc_vec = -veh_props.acceleration

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(veh_props.dir, vec3(0,0,1)) * acc_vec

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = veh_props.front_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = veh_props.center_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = veh_props.rear_pos + (veh_props.velocity + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getFuturePositionXY(veh, time, rel_car_pos)
  local veh_props = getVehicleProperties(veh)

  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local acc_vec = -veh_props.acceleration
  acc_vec.z = 0

  local dir_xy = toNormXYVec(veh_props.dir)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * acc_vec

  veh_props.front_pos.z = 0
  veh_props.center_pos.z = 0
  veh_props.rear_pos.z = 0

  veh_props.velocity.z = 0

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = veh_props.front_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = veh_props.center_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = veh_props.rear_pos + (veh_props.velocity + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getFuturePositionXYWithAcc(veh, time, acc_vec, rel_car_pos)
  local veh_props = getVehicleProperties(veh)

  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local dir_xy = toNormXYVec(veh_props.dir)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * -acc_vec

  veh_props.front_pos.z = 0
  veh_props.center_pos.z = 0
  veh_props.rear_pos.z = 0

  veh_props.velocity.z = 0

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = veh_props.front_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = veh_props.center_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = veh_props.rear_pos + (veh_props.velocity + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getWaypointPosition(wp)
  if wp ~= nil then
    return vec3(map.getMap().nodes[wp].pos)
  else
    return vec3(-1,-1,-1)
  end
end

--Get start and end waypoints relative to my vehicle nearest to a position
local function getWaypointStartEnd(my_veh, position)
  local my_car_dir = vec3(my_veh.obj:getDirectionVector()):normalized()

  --Relative to my car
  local origin = my_car_dir * -9999

  local start_wp = nil
  local end_wp = nil

  local wp1, wp2, lat_dist_from_wp = map.findClosestRoad(position)

  --Get lane width
  local half_road_width = map.getMap().nodes[wp1].radius
  local my_start_links = map.getMap().nodes[wp1].links
  
  local one_way = false

  for wp, data in pairs(my_start_links) do
    one_way = data.oneWay
    break
  end

  local lane_width = half_road_width

  --For one lane road
  if half_road_width < 3 and one_way then
    lane_width = lane_width * 2
  end


  --For some reason lateral distance from waypoint is offset by 0.5
  lat_dist_from_wp = lat_dist_from_wp - 0.5

  if wp1 == nil or wp2 == nil then
    return start_wp, end_wp
  end

  local wp1_pos = getWaypointPosition(wp1)
  local wp2_pos = getWaypointPosition(wp2)

  --Figure out which waypoints are the start and end waypoint
  if math.abs((origin - wp1_pos):length()) < math.abs((origin - wp2_pos):length()) then
    start_wp = wp1
    end_wp = wp2
  else
    start_wp = wp2
    end_wp = wp1
  end

  return start_wp, end_wp, lat_dist_from_wp, lane_width
end

--Also predicts your future position to find more suitable waypoints
local function getWaypointStartEndAdvanced(my_veh, veh, position)
  local my_veh_props = getVehicleProperties(my_veh)
  local veh_props = getVehicleProperties(veh)

  local start_wp, end_wp, lat_dist_from_wp, lane_width = getWaypointStartEnd(my_veh, position)

  --Check using future positions

  local veh_pos_future = getFuturePosition(veh, 1, "center")
  local future_start_wp, future_end_wp, future_lat_dist_from_wp, future_lane_width = getWaypointStartEnd(my_veh, veh_pos_future)

  local veh_pos_future2 = getFuturePosition(veh, 2, "center")
  local future_start_wp2, future_end_wp2, future_lat_dist_from_wp2, future_lane_width2 = getWaypointStartEnd(my_veh, veh_pos_future2)

  if start_wp == nil then
    return start_wp, end_wp, lat_dist_from_wp, lane_width
  end

  local wps = {}

  table.insert(wps, start_wp)
  table.insert(wps, end_wp)
  
  if future_start_wp ~= nil then
    table.insert(wps, future_start_wp)
    table.insert(wps, future_end_wp)
  end
  
  if future_start_wp2 ~= nil then
    table.insert(wps, future_start_wp2)
    table.insert(wps, future_end_wp2) 
  end
  
  local angle_between_vehs = math.acos(my_veh_props.dir:dot(veh_props.dir))

  if angle_between_vehs > math.pi / 2.0 then
    angle_between_vehs = math.pi
  else
    angle_between_vehs = 0
  end

  local min_wp_angle = {math.pi, nil, nil}

  for i = 1, #wps - 1 do
    local wp1 = wps[i]
    local wp2 = wps[i + 1]
    local wp1_pos = getWaypointPosition(wp1)
    local wp2_pos = getWaypointPosition(wp2)

    --Get direction between our waypoints and one of its linked waypoints
    local wp_dir = (wp2_pos - wp1_pos):normalized()

    --Angle between waypoint dir and car dir
    local angle = math.acos(wp_dir:dot(veh_props.dir))

    angle = math.abs(angle_between_vehs - angle)

    if angle > math.pi / 2.0 then
      angle = math.pi - angle

      if angle < min_wp_angle[1] then
        min_wp_angle[1] = angle
        min_wp_angle[2] = wp2
        min_wp_angle[3] = wp1
      end
    else
      if angle < min_wp_angle[1] then
        min_wp_angle[1] = angle
        min_wp_angle[2] = wp1
        min_wp_angle[3] = wp2
      end
    end
    --debugDrawer:drawSphere((wp2_pos + vec3(0,0,2)):toPoint3F(), 0.5, ColorF(1,1,0,1))
  end

  return min_wp_angle[2], min_wp_angle[3], lat_dist_from_wp, lane_width
end

--returns true for right, false for left
local function getWhichSideCarIsOnAndWaypoints(my_veh, veh)
  local veh_props = getVehicleProperties(veh)

  --Gets waypoints with start and end relative to my vehicle (not other vehicles)
  local start_wp, end_wp, lat_dist_from_wp = getWaypointStartEndAdvanced(my_veh, veh, veh_props.front_pos)

  --If no waypoints exist, don't stop system as failsafe
  if start_wp == nil or end_wp == nil then
    return nil, nil, nil, nil, 0
  end

  local start_pos = getWaypointPosition(start_wp)
  local end_pos = getWaypointPosition(end_wp)

  local wp_mid_pos = (end_pos - start_pos) * 0.5 + start_pos

  --If the waypoints are too far away, then don't use them
  --but don't stop system
  if (wp_mid_pos - veh_props.center_pos):length() > 50 then
    return nil, nil, nil, nil, 0
  end

  local road_line_dir = toNormXYVec(end_pos - start_pos)
  local car_dir_xy = toNormXYVec(veh_props.dir)

  --debugDrawer:drawSphere(start_pos:toPoint3F(), 0.5, ColorF(1,0,0,1))
  --debugDrawer:drawSphere(wp_mid_pos:toPoint3F(), 0.5, ColorF(0,1,0,1))
  --debugDrawer:drawSphere(end_pos:toPoint3F(), 0.5, ColorF(0,0,1,1))

  local color = toColorI(ColorF(1,0,0,0.25))

  --debugDrawer:setSolidTriCulling(false)
  --debugDrawer:drawQuadSolid((start_pos):toPoint3F(), (start_pos + vec3(0,0,1)):toPoint3F(),
  --(end_pos + vec3(0,0,1)):toPoint3F(), (end_pos):toPoint3F(), color)

  --Get angle between car dir and road dir that is between 0 and 360 degrees
  local line_to_left_side_dir = (veh_props.center_pos - veh_props.dir_right * veh_props.bb:getHalfExtents().x * 0.5 - wp_mid_pos):normalized()
  local line_to_right_side_dir = (veh_props.center_pos + veh_props.dir_right * veh_props.bb:getHalfExtents().x * 0.5 - wp_mid_pos):normalized()

  local left_dot = line_to_left_side_dir:dot(road_line_dir)
  local left_det = line_to_left_side_dir.x * road_line_dir.y - line_to_left_side_dir.y * road_line_dir.x

  local right_dot = line_to_right_side_dir:dot(road_line_dir)
  local right_det = line_to_right_side_dir.x * road_line_dir.y - line_to_right_side_dir.y * road_line_dir.x

  local left_angle = math.atan2(left_det, left_dot) * 180.0 / math.pi
  local right_angle = math.atan2(right_det, right_dot) * 180.0 / math.pi

  local side = nil

  --Right side
  if left_angle > 0 and left_angle < 180 then
    side = "right"
    --Left side
  else
    side = "left"
  end

  --Right side
  if right_angle > 0 and right_angle < 180 then
    if side == "left" then
      side = "both"
    end
    --Left side
  else
    if side == "right" then
      side = "both"
    end
  end

  return side, start_wp, end_wp, lat_dist_from_wp
end

local function checkIfOtherCarWithinMyRoadHalfWidth(my_veh, half_road_width, other_veh_props, my_veh_start_wp, my_start_wp_pos, my_end_wp_pos)
  local half_road_width = map.getMap().nodes[my_veh_start_wp].radius

  local xnorm = other_veh_props.center_pos:xnormOnLine(my_start_wp_pos, my_end_wp_pos)
  local my_wp_start_end = my_end_wp_pos - my_start_wp_pos
  local other_veh_pos_on_wp_line = xnorm * my_wp_start_end + my_start_wp_pos
  
  --Get waypoint on my road closest to other vehicle
  local new_start_wp, new_end_wp, new_lat_dist_from_wp, new_lane_width = getWaypointStartEnd(my_veh, other_veh_pos_on_wp_line)
  
  if new_start_wp == nil then
    return true
  end
  
  local new_start_wp_pos = getWaypointPosition(new_start_wp)
  local new_end_wp_pos = getWaypointPosition(new_end_wp)
  
   --Get lateral distance of other car to waypoint
  local xnorm2 = other_veh_props.center_pos:xnormOnLine(new_start_wp_pos, new_end_wp_pos)
  local new_my_wp_start_end = new_end_wp_pos - new_start_wp_pos
  local new_other_veh_pos_on_wp_line = xnorm2 * new_my_wp_start_end + new_start_wp_pos
  
  local lat_dist = (other_veh_props.center_pos - new_other_veh_pos_on_wp_line):length()

  return lat_dist < half_road_width
end

local function checkIfCarsAreInSameLane(my_veh, other_veh)
  local my_veh_props = getVehicleProperties(my_veh)
  local other_veh_props = getVehicleProperties(other_veh)

  local my_veh_side, my_veh_start_wp, my_veh_end_wp, my_lat_dist_from_wp
    = getWhichSideCarIsOnAndWaypoints(my_veh, my_veh)

  local my_start_wp_pos = getWaypointPosition(my_veh_start_wp)
  local my_end_wp_pos = getWaypointPosition(my_veh_end_wp)

  if my_start_wp_pos == vec3(-1,-1,-1) or my_end_wp_pos == vec3(-1,-1,-1) then
    return false, 0, 0, "both", 2
  end
  
  local other_veh_side, other_veh_start_wp, other_veh_end_wp, other_lat_dist_from_wp
    = getWhichSideCarIsOnAndWaypoints(my_veh, other_veh)

  local other_start_wp_pos = getWaypointPosition(other_veh_start_wp)
  local other_end_wp_pos = getWaypointPosition(other_veh_end_wp)

  local half_road_width = map.getMap().nodes[my_veh_start_wp].radius

  local my_start_links = map.getMap().nodes[my_veh_start_wp].links

  local one_way = false

  for wp, data in pairs(my_start_links) do
    one_way = data.oneWay
    break
  end

  --For one lane road
  if half_road_width < 3 and one_way then
    local in_same_lane = checkIfOtherCarWithinMyRoadHalfWidth(
      my_veh, half_road_width, other_veh_props, my_veh_start_wp, my_start_wp_pos, my_end_wp_pos)

    if in_same_lane then
      return true, my_lat_dist_from_wp, other_lat_dist_from_wp, my_veh_side, half_road_width * 2
    end

    --For two lane roads
  else
    --Both on same side of waypoints, but doesn't mean we are actually in same lane.
    --So check if distance of other vehicle's lateral distance from my waypoint
    --is within bounds of the waypoint's road (using radius of waypoint)
    if my_veh_side == other_veh_side or my_veh_side == "both" or other_veh_side == "both" then
      local in_same_lane = checkIfOtherCarWithinMyRoadHalfWidth(
        my_veh, half_road_width, other_veh_props, my_veh_start_wp, my_start_wp_pos, my_end_wp_pos)

      if in_same_lane then
        return true, my_lat_dist_from_wp, other_lat_dist_from_wp, my_veh_side, half_road_width
      end
    end
  end
  
  return false, my_lat_dist_from_wp, other_lat_dist_from_wp, my_veh_side, half_road_width
end

local function getCircularDistance(my_veh_props, other_veh_props, min_distance_from_car)
  local other_bb = other_veh_props.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local shoot_ray_dir = (my_veh_props.front_pos - other_veh_props.center_pos):normalized()

  --Now get exact distance
  local max_distance, min_distance = intersectsRay_OBB(my_veh_props.front_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

  --For some reason, distance is negative
  min_distance = math.abs(min_distance) - min_distance_from_car

  if min_distance < 0 then
    min_distance = 0
  end

  --Convert to circular distance
  local turning_radius = math.abs(my_veh_props.speed / angular_speed_angelo234)

  local angle = math.acos(
    (min_distance * min_distance) / (-2 * turning_radius * turning_radius) + 1)

  local cir_dist = angle * turning_radius

  return cir_dist
end

local function getStraightDistance(my_veh_props, other_veh_props, min_distance_from_car, front)
  local other_bb = other_veh_props.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local ray_pos = nil

  if front then
    ray_pos = my_veh_props.front_pos
  else
    ray_pos = my_veh_props.rear_pos
  end

  local shoot_ray_dir = (ray_pos - other_veh_props.center_pos):normalized()

  --Now get exact distance
  local max_distance, min_distance = intersectsRay_OBB(ray_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

  --For some reason, distance is negative
  min_distance = math.abs(min_distance) - min_distance_from_car

  if min_distance < 0 then
    min_distance = 0
  end

  return min_distance
end

--Returns a table of vehicles and distance to them within a max_dist radius
--only in front of our vehicle, so it discards vehicles behind
local function getNearbyVehicles(my_veh, max_dist, angular_speed_angelo234, min_distance_from_car, in_front)
  local other_vehs = {}

  local my_veh_props = getVehicleProperties(my_veh)

  local vehicles = getAllVehicles()

  for _, other_veh in pairs(vehicles) do
    local veh_id = other_veh:getID()

    if veh_id ~= my_veh:getID() then
      local other_veh_props = getVehicleProperties(other_veh)

      --Get aproximate distance first between vehicles and return if less than max dist
      local other_bb = other_veh_props.bb

      local front_dist = (my_veh_props.front_pos - other_veh_props.center_pos):length()
      local rear_dist = (my_veh_props.rear_pos - other_veh_props.center_pos):length()

      --If rear distance is larger than front distance, then vehicle is in front
      if front_dist < max_dist and front_dist < rear_dist and in_front then
        local cir_dist = getCircularDistance(my_veh_props, other_veh_props, min_distance_from_car)
        table.insert(other_vehs, {other_veh, cir_dist})

        --If front distance is larger than rear distance, then vehicle is in rear
      elseif rear_dist < max_dist and front_dist > rear_dist and not in_front then
        local dist = getStraightDistance(my_veh_props, other_veh_props, min_distance_from_car, false)
        table.insert(other_vehs, {other_veh, dist})
      end
    end
  end

  return other_vehs
end

--Returns a table of vehicles and distance to them within a max_dist radius and in the same lane
local function getNearbyVehiclesInSameLane(my_veh, max_dist, angular_speed_angelo234, min_distance_from_car, in_front)
  local other_vehs_data = getNearbyVehicles(my_veh, max_dist, angular_speed_angelo234, min_distance_from_car, in_front)
  local other_vehs_in_my_lane = {}

  local my_veh_props = getVehicleProperties(my_veh)

  for _, other_veh_data in pairs(other_vehs_data) do
    local other_veh = other_veh_data[1]
    local this_distance = other_veh_data[2]

    local other_veh_props = getVehicleProperties(other_veh)
    local speed_rel = my_veh_props.speed - other_veh_props.speed
    local same_lane, my_lat_dist_from_wp, other_lat_dist_from_wp, my_veh_side, half_road_width = checkIfCarsAreInSameLane(my_veh, other_veh)

    --print(same_lane)

    other_veh_data[3] = my_lat_dist_from_wp
    other_veh_data[4] = other_lat_dist_from_wp
    other_veh_data[5] = my_veh_side
    other_veh_data[6] = half_road_width

    --In same lane and my vehicle speed is >= to other
    if same_lane and speed_rel >= 0 then
      table.insert(other_vehs_in_my_lane, other_veh_data)
    end
  end

  return other_vehs_in_my_lane
end

M.toColorI = toColorI
M.rotateEuler = rotateEuler
M.circleFrom3Pts = circleFrom3Pts
M.toNormXYVec = toNormXYVec
M.getVehicleProperties = getVehicleProperties
M.getDistanceBetweenWaypoints = getDistanceBetweenWaypoints
M.getFuturePosition = getFuturePosition
M.getFuturePositionXY = getFuturePositionXY
M.getFuturePositionXYWithAcc = getFuturePositionXYWithAcc
M.getWaypointPosition = getWaypointPosition
M.getWaypointStartEnd = getWaypointStartEnd
M.getWaypointStartEndAdvanced = getWaypointStartEndAdvanced
M.getWhichSideCarIsOnAndWaypoints = getWhichSideCarIsOnAndWaypoints
M.checkIfCarsAreInSameLane = checkIfCarsAreInSameLane
M.getNearbyVehicles = getNearbyVehicles
M.getNearbyVehiclesInSameLane = getNearbyVehiclesInSameLane

return M
