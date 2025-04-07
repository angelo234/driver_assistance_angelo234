local M = {}

require("lua/common/luaProfiler")

--For efficiency
local max = math.max
local min = math.min
local sin = math.sin
local asin = math.asin
local acos = math.acos
local atan2 = math.atan2
local pi = math.pi
local abs = math.abs
local sqrt = math.sqrt
local floor = math.floor
local ceil = math.ceil

local map_nodes = map.getMap().nodes
local findClosestRoad = map.findClosestRoad

local function getPart(partName)
  local vehData = core_vehicle_manager.getPlayerVehicleData()
  if not vehData then return nil end
  return vehData.vdata.activePartsData[partName]
end

local vehs_props_reusing = {}

local function getVehicleProperties(veh)
  if not vehs_props_reusing[veh:getID()] then
    vehs_props_reusing[veh:getID()] = {
      name = nil,
      id = nil,
      dir = vec3(),
      dir_up = vec3(),
      dir_right = nil,
      bb = nil,
      center_pos = vec3(),
      front_pos = nil,
      rear_pos = vec3(),
      velocity = vec3(),
      speed = nil,
      acceleration = vec3()
    }
  end

  local props = vehs_props_reusing[veh:getID()]

  props.name = veh:getJBeamFilename()
  props.id = veh:getID()

  --Direction vectors of vehicle
  props.dir:set(veh.obj:getDirectionVector())
  props.dir_up:set(veh.obj:getDirectionVectorUp())
  props.dir_right = props.dir:cross(props.dir_up)

  --Bounding box of vehicle
  props.bb = veh:getSpawnWorldOOBB()
  props.center_pos:set(props.bb:getCenter())
  props.front_pos = props.center_pos + props.dir * veh:getSpawnAABBRadius()
  props.rear_pos:set(veh:getSpawnWorldOOBBRearPoint())

  props.velocity:set(veh:getVelocity())
  props.speed = props.velocity:length()

  local acceleration = veh_accs_angelo234[veh:getID()]

  if acceleration == nil then
    props.acceleration:set(0,0,0)
  else
    props.acceleration:set(acceleration[1], acceleration[2], 9.81 - acceleration[3])
  end

  return props
end

local function getPathLen(path, startIdx, stopIdx)
  if not path then return end
  startIdx = startIdx or 1
  stopIdx = stopIdx or #path
  local pathLen = 0
  for i = startIdx+1, stopIdx do
    pathLen = pathLen + (map_nodes[path[i]].pos - map_nodes[path[i-1]].pos):length()
  end

  return pathLen
end

--Check if waypoint is on the same road as me (not lane)
local function checkIfWaypointsWithinMyCar(veh_props, wps_props)
  local xnorm = veh_props.center_pos:xnormOnLine(wps_props.start_wp_pos, wps_props.end_wp_pos)
  local wp_dir = wps_props.end_wp_pos - wps_props.start_wp_pos
  local veh_pos_on_wp_line = xnorm * wp_dir + wps_props.start_wp_pos

  local lat_dist = (veh_props.center_pos - veh_pos_on_wp_line):length()

  --debugDrawer:drawTextAdvanced((wps_props.end_wp_pos), String("Lateral Distance: " .. tostring(lat_dist)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  return lat_dist < wps_props.wp_radius
end

local function toNormXYVec(dir)
  return dir:z0():normalized()
end

local function getFuturePosition(veh_props, time, rel_car_pos)
  local acc_vec = vec3(-veh_props.acceleration)

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

local function getFuturePositionXY(veh_props, time, rel_car_pos)
  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local acc_vec = vec3(-veh_props.acceleration):z0()

  local dir_xy = toNormXYVec(veh_props.dir)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * acc_vec

  local front_pos_xy = veh_props.front_pos:z0()
  local center_pos_xy = veh_props.center_pos:z0()
  local rear_pos_xy = veh_props.rear_pos:z0()

  local velocity_xy = veh_props.velocity:z0()

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos_xy + (velocity_xy + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getFuturePositionXYWithAcc(veh_props, time, acc_vec, rel_car_pos)
  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local dir_xy = toNormXYVec(veh_props.dir)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * -acc_vec

  local front_pos_xy = veh_props.front_pos:z0()
  local center_pos_xy = veh_props.center_pos:z0()
  local rear_pos_xy = veh_props.rear_pos:z0()

  local velocity_xy = veh_props.velocity:z0()

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos_xy + (velocity_xy + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getWaypointPosition(wp)
  if wp ~= nil then
    return vec3(map_nodes[wp].pos)
  else
    return vec3(-1,-1,-1)
  end
end

local function getLaneWidth(wps_info)
  local max_lane_width = 3.75
  local lane_width = 0
  local lanes = 0

  if wps_info.one_way then
    if wps_info.wp_radius * 2 < max_lane_width then
      --For one lane road
      lane_width = wps_info.wp_radius * 2.0
      lanes = 1

    elseif wps_info.wp_radius * 2 < max_lane_width * 2 then
      --For two lane road
      lane_width = wps_info.wp_radius
      lanes = 2

    elseif wps_info.wp_radius * 2 < max_lane_width * 3 then
      --For three lane road
      lane_width = wps_info.wp_radius * 2.0 / 3.0
      lanes = 3

    elseif wps_info.wp_radius * 2 < max_lane_width * 4 then
      --For four lane road
      lane_width = wps_info.wp_radius * 2.0 / 4.0
      lanes = 4

    end
  else
    lane_width = wps_info.wp_radius
    lanes = 2
  end

  return lane_width, lanes
end

--Gets whether we are to the left or right of waypoints
local function getWhichSideOfWaypointsCarIsOn(veh_props, start_pos, end_pos)
  local road_line_dir = toNormXYVec(end_pos - start_pos)
  local road_line_dir_perp_right = vec3(road_line_dir.y, -road_line_dir.x)
  local car_dir_xy = toNormXYVec(veh_props.dir)

  local wp_mid_pos = (end_pos - start_pos) * 0.5 + start_pos

  local line_to_center_dir = (veh_props.center_pos - wp_mid_pos):normalized()

  local center_angle = acos(line_to_center_dir:dot(road_line_dir_perp_right))

  --Determine side of road car is on
   --Right side
  if center_angle < pi / 2.0 then
    return "right"
    --Left side
  else
    return "left"
  end
end

local function getWaypointsProperties(veh_props, start_wp, end_wp, start_wp_pos, end_wp_pos, lat_dist_from_wp)
  local wps_props = {}

  --Get lane width
  local wp_radius = map_nodes[start_wp].radius
  local my_start_links = map_nodes[start_wp].links

  local one_way = false

  for wp, data in pairs(my_start_links) do
    one_way = data.oneWay
    break
  end

  wps_props.start_wp = start_wp
  wps_props.end_wp = end_wp
  wps_props.start_wp_pos = start_wp_pos
  wps_props.end_wp_pos = end_wp_pos
  wps_props.one_way = one_way
  wps_props.wp_radius = wp_radius
  wps_props.lat_dist_from_wp = lat_dist_from_wp

  local lane_width, num_of_lanes = getLaneWidth(wps_props)

  wps_props.lane_width = lane_width
  wps_props.num_of_lanes = num_of_lanes

  local my_veh_side_of_wp = getWhichSideOfWaypointsCarIsOn(veh_props, start_wp_pos, end_wp_pos)

  if my_veh_side_of_wp == "left" then
    wps_props.lat_dist_from_wp = -wps_props.lat_dist_from_wp
  end

  wps_props.veh_side_of_wp = my_veh_side_of_wp

  return wps_props
end

--Get start and end waypoints relative to my vehicle nearest to a position
local function getWaypointStartEnd(my_veh_props, veh_props, position)
  local start_wp = nil
  local end_wp = nil

  local start_wp_pos = nil
  local end_wp_pos = nil

  local wp1, wp2, lat_dist_from_wp = findClosestRoad(position)

  if wp1 == nil or wp2 == nil then
    return nil
  end

  local wp1_pos = getWaypointPosition(wp1)
  local wp2_pos = getWaypointPosition(wp2)

  --Relative to my car
  local origin = my_veh_props.dir * -9999

  --Figure out which waypoints are the start and end waypoint
  if abs((origin - wp1_pos):length()) < abs((origin - wp2_pos):length()) then
    start_wp = wp1
    end_wp = wp2

    start_wp_pos = wp1_pos
    end_wp_pos = wp2_pos
  else
    start_wp = wp2
    end_wp = wp1

    start_wp_pos = wp2_pos
    end_wp_pos = wp1_pos
  end

  local wps_props = getWaypointsProperties(veh_props, start_wp, end_wp, start_wp_pos, end_wp_pos, lat_dist_from_wp)

  return wps_props
end

local wps_reusing = {}

--Uses past wps to find most suitable waypoints to use
local function getWaypointStartEndAdvanced(my_veh_props, veh_props, position, past_wps_props)
  for k in pairs(wps_reusing) do
    wps_reusing[k] = nil
  end

  local wps_props = getWaypointStartEnd(my_veh_props, veh_props, position)

  table.insert(wps_reusing, wps_props)

  if past_wps_props then
    table.insert(wps_reusing, past_wps_props)
  end

  local angle_between_vehs = acos(my_veh_props.dir:dot(veh_props.dir))

  if angle_between_vehs > pi / 2.0 then
    angle_between_vehs = pi
  else
    angle_between_vehs = 0
  end

  local min_wp_props_angle = {pi, nil}

  for _, curr_wps_props in pairs(wps_reusing) do
    --Get direction between our waypoints and one of its linked waypoints
    local wp_dir = (curr_wps_props.end_wp_pos - curr_wps_props.start_wp_pos):normalized()

    --Angle between waypoint dir and car dir
    local angle = acos(wp_dir:dot(veh_props.dir))

    --print(angle * 180.0 / pi)

    angle = abs(angle_between_vehs - angle)

    if angle > pi / 2.0 then
      angle = pi - angle
    end

    if angle < min_wp_props_angle[1] then
      min_wp_props_angle[1] = angle
      min_wp_props_angle[2] = curr_wps_props
    end

    --debugDrawer:drawSphere((curr_wps_props.start_wp_pos + vec3(0,0,2)), 0.5, ColorF(1,1,0,1))
    --debugDrawer:drawTextAdvanced((wp1_pos + vec3(0,0,3)), String("ID: " .. tostring(i)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

    --debugDrawer:drawSphere((curr_wps_props.end_wp_pos + vec3(0,0,2)), 0.5, ColorF(1,1,0,1))
    --debugDrawer:drawTextAdvanced((wp2_pos + vec3(0,0,3)), String("ID: " .. tostring(i + 1)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  end

  return min_wp_props_angle[2]
end

--Check if other car is on the same road as me (not lane)
local function checkIfOtherCarOnSameRoad(my_veh_props, other_veh_props, wps_props)
  --Calculate distance to get from my waypoint to other vehicle's waypoint using graphpath
  --and if that distance is equal or less than shortest distance * 1.05 between two then
  --vehicle is in path

  local other_wps_props_in_other_dir = getWaypointStartEnd(other_veh_props, other_veh_props, other_veh_props.center_pos)

  local path = map.getPath(wps_props.start_wp, other_wps_props_in_other_dir.start_wp, 0, 100)
  local path_len = getPathLen(path)

  --debugDrawer:drawTextAdvanced((other_veh_props.front_pos), String(path_len),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  return path_len <= (wps_props.start_wp_pos - other_wps_props_in_other_dir.start_wp_pos):length() * 1.05
end

local function getCircularDistance(my_veh_props, other_veh_props)
  local other_bb = other_veh_props.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local shoot_ray_dir = (other_veh_props.center_pos - my_veh_props.front_pos):normalized()

  local min_distance, max_distance = intersectsRay_OBB(my_veh_props.front_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

  if min_distance < 0 then
    min_distance = 0
  end

  --Convert to circular distance
  local turning_radius = abs(my_veh_props.speed / angular_speed_angelo234)

  local angle = acos(
    (min_distance * min_distance) / (-2 * turning_radius * turning_radius) + 1)

  local cir_dist = angle * turning_radius

  return cir_dist
end

local function getStraightDistance(my_veh_props, other_veh_props, front, in_my_vehs_straight_path)
  local my_bb = my_veh_props.bb

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

  local min_distance = 9999
  local shoot_ray_dir = nil

  if in_my_vehs_straight_path then
    if front then
      shoot_ray_dir = my_veh_props.dir
    else
      shoot_ray_dir = -my_veh_props.dir
    end

    local min_distance1, max_distance1 = intersectsRay_OBB(ray_pos + my_veh_props.dir_right * my_bb:getHalfExtents().x * 0.75, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)
    local min_distance2, max_distance2 = intersectsRay_OBB(ray_pos - my_veh_props.dir_right * my_bb:getHalfExtents().x * 0.75, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)
    local min_distance3, max_distance3 = intersectsRay_OBB(ray_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

    min_distance = math.min(min_distance1, min_distance2, min_distance3)
  else
    shoot_ray_dir = (other_veh_props.center_pos - ray_pos):normalized()

    local min_distance1, max_distance1 = intersectsRay_OBB(ray_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

    min_distance = min_distance1
  end

  if min_distance < -0.1 then
    min_distance = 9999
  end

  return min_distance
end

local function onClientPostStartMission(levelpath)
  map_nodes = map.getMap().nodes
  findClosestRoad = map.findClosestRoad
end

M.getPart = getPart
M.getVehicleProperties = getVehicleProperties
M.toNormXYVec = toNormXYVec
M.getFuturePosition = getFuturePosition
M.getFuturePositionXY = getFuturePositionXY
M.getFuturePositionXYWithAcc = getFuturePositionXYWithAcc
M.getWaypointPosition = getWaypointPosition
M.getLaneWidth = getLaneWidth
M.getWhichSideOfWaypointsCarIsOn = getWhichSideOfWaypointsCarIsOn
M.getWaypointStartEnd = getWaypointStartEnd
M.getWaypointStartEndAdvanced = getWaypointStartEndAdvanced
M.checkIfOtherCarOnSameRoad = checkIfOtherCarOnSameRoad
M.getCircularDistance = getCircularDistance
M.getStraightDistance = getStraightDistance
M.onClientPostStartMission = onClientPostStartMission

return M