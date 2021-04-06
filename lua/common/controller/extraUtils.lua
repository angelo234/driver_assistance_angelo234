local M = {}

local prev_mapmgr_objects = deepcopy(mapmgr.objects)

local function rotateEuler(x, y, z, q)
  q = q or quat()
  q = quatFromEuler(0, z, 0) * q
  q = quatFromEuler(0, 0, x) * q
  q = quatFromEuler(y, 0, 0) * q
  return q
end

local function toNormXYVec(dir)
  return dir:z0():normalized()
end

--Check if waypoint is on the same road as me (not lane)
local function checkIfWaypointsWithinMyCar(veh_id, wps_props)
  local veh = mapmgr.objects[veh_id]

  local front_pos = vec3(obj:getObjectFrontPosition(veh_id)) + veh.dirVec * 0.5
  local center_pos = front_pos - veh.dirVec * obj:getObjectInitialLength(veh_id) * 0.5

  local xnorm = center_pos:xnormOnLine(wps_props.start_wp_pos, wps_props.end_wp_pos)
  local wp_dir = wps_props.end_wp_pos - wps_props.start_wp_pos
  local veh_pos_on_wp_line = xnorm * wp_dir + wps_props.start_wp_pos
  
  local lat_dist = (center_pos - veh_pos_on_wp_line):length()

  return lat_dist < wps_props.wp_radius
end

local function getFuturePosition(veh_id, time, rel_car_pos)
  local veh = mapmgr.objects[veh_id]
  
  local acc_vec = vec3()
  
  if veh.acc ~= nil then
    acc_vec = veh.acc
  end
  
  local front_pos = vec3(obj:getObjectFrontPosition(veh_id)) + veh.dirVec * 0.5
  local center_pos = front_pos - veh.dirVec * obj:getObjectInitialLength(veh_id) * 0.5
  local rear_pos = front_pos - veh.dirVec * obj:getObjectInitialLength(veh_id)
  
  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos + (veh.vel + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos + (veh.vel + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos + (veh.vel + acc_vec * time) * time
  end

  return veh_pos_future
end


local function getFuturePositionXY(veh_id, time, rel_car_pos)
  local veh = mapmgr.objects[veh_id]
  
  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  
  local acc_vec_xy = vec3()
  
  if veh.acc ~= nil then
    acc_vec_xy = veh.acc:z0()
  end

  local front_pos_xy = (vec3(obj:getObjectFrontPosition(veh_id)) + veh.dirVec * 0.5):z0()
  local center_pos_xy = (front_pos_xy - veh.dirVec * obj:getObjectInitialLength(veh_id) * 0.5):z0()
  local rear_pos_xy = (front_pos_xy - veh.dirVec * obj:getObjectInitialLength(veh_id)):z0()

  local velocity_xy = veh.vel:z0()

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos_xy + (velocity_xy + acc_vec_xy * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos_xy + (velocity_xy + acc_vec_xy * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos_xy + (velocity_xy + acc_vec_xy * time) * time
  end

  return veh_pos_future
end



local function getFuturePositionXYWithAcc(veh_id, time, the_acc_vec, rel_car_pos)
  local veh = mapmgr.objects[veh_id]
  
  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local dir_xy = toNormXYVec(veh.dirVec)

  --Convert from local space into world space acceleration vector
  the_acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * -the_acc_vec

  local front_pos_xy = (vec3(obj:getObjectFrontPosition(veh_id)) + veh.dirVec * 0.5):z0()
  local center_pos_xy = (front_pos_xy - veh.dirVec * obj:getObjectInitialLength(veh_id) * 0.5):z0()
  local rear_pos_xy = (front_pos_xy - veh.dirVec * obj:getObjectInitialLength(veh_id)):z0()

  local velocity_xy = veh.vel:z0()

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos_xy + (velocity_xy + the_acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos_xy + (velocity_xy + the_acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos_xy + (velocity_xy + the_acc_vec * time) * time
  end

  return veh_pos_future
end

local function getWaypointPosition(wp)
  if wp ~= nil then
    return vec3(mapmgr.mapData.positions[wp])
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

local function getWaypointsProperties(start_wp, end_wp, lat_dist_from_wp)
  local wps_props = {}
  
  local start_wp_pos = getWaypointPosition(start_wp)
  local end_wp_pos = getWaypointPosition(end_wp)

  --Get lane width
  local wp_radius = mapmgr.mapData.radius.start_wp
  --local my_start_links = map.getMap().nodes[start_wp].links
  
  local one_way = false

  --for wp, data in pairs(my_start_links) do
    --one_way = data.oneWay
    --break
  --end

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
  
  return wps_props
end


--Get start and end waypoints relative to my vehicle nearest to a position
local function getWaypointStartEnd(position)
  
  local start_wp = nil
  local end_wp = nil

  local wp1, wp2, lat_dist_from_wp = mapmgr.findClosestRoad(position)

  --For some reason lateral distance from waypoint is offset by 0.5
  lat_dist_from_wp = lat_dist_from_wp - 0.5

  if wp1 == nil or wp2 == nil then
    return nil
  end

  local wp1_pos = getWaypointPosition(wp1)
  local wp2_pos = getWaypointPosition(wp2)

  --Relative to my car
  local origin = vec3(obj:getDirectionVector()) * -9999

  --Figure out which waypoints are the start and end waypoint
  if math.abs((origin - wp1_pos):length()) < math.abs((origin - wp2_pos):length()) then
    start_wp = wp1
    end_wp = wp2
  else
    start_wp = wp2
    end_wp = wp1
  end

  local wps_props = getWaypointsProperties(start_wp, end_wp, lat_dist_from_wp)

  return wps_props
end


--Also predicts your future position to find more suitable waypoints
local function getWaypointStartEndAdvanced(veh_id, position)
  local veh = mapmgr.objects[veh_id]
  
  local wps = {}

  local wps_props = getWaypointStartEnd(position)

  --Check using future positions

  local veh_pos_past = getFuturePosition(veh_id, -1, "center")
  local past_wps_props = getWaypointStartEnd(veh_pos_past)

  local veh_pos_future = getFuturePosition(veh_id, 1, "center")
  local future_wps_props = getWaypointStartEnd(veh_pos_future)

  --local veh_pos_future2 = getFuturePosition(veh_id, 2, "center")
  --local future_wps_props2 = getWaypointStartEnd(veh_pos_future2)

  if wps_props == nil then
    return nil
  end

  table.insert(wps, wps_props.start_wp)
  table.insert(wps, wps_props.end_wp)
  
  if past_wps_props ~= nil then
    local within = checkIfWaypointsWithinMyCar(past_wps_props)
  
    if within then
      table.insert(wps, past_wps_props.start_wp)
      table.insert(wps, past_wps_props.end_wp)
    end
  end
  
  if future_wps_props ~= nil then
    local within = checkIfWaypointsWithinMyCar(future_wps_props)
  
    if within then
      table.insert(wps, future_wps_props.start_wp)
      table.insert(wps, future_wps_props.end_wp)
    end
  end
  
  --if future_wps_props2 ~= nil then
    --table.insert(wps, future_wps_props2.start_wp)
    --table.insert(wps, future_wps_props2.end_wp) 
  --end
  
  --Remove duplicates
  local hash = {}
  local res = {}
  
  for _, val in ipairs(wps) do
    if not hash[val] then
      res[#res + 1] = val
      hash[val] = true
    end
  end
  
  wps = res
  
  --Sort waypoints relative to my vehicle
  local back_origin = my_.dir * -9999
  
  local new_wps = {}
  
  --Calculate distance from origin displaced far away from back of vehicle to waypoints
  for _, wp in ipairs(wps) do
    local wp_pos = getWaypointPosition(wp)
    local dist = (wp_pos - back_origin):length()
  
    table.insert(new_wps, {dist, wp})
  end
  
  wps = {}

  local len = #new_wps

  for j = 2, len do
    local val = new_wps[j]
    local i = j - 1
    while i > 0 and new_wps[i][1] > val[1] do
      new_wps[i + 1] = new_wps[i]
      i = i - 1
    end
    new_wps[i + 1] = val  
  end
  
  for _, val in ipairs(new_wps) do
    table.insert(wps, val[2])
  end
  

  local angle_between_vehs = math.acos(vec3(obj:getDirectionVector()):dot(veh.dirVec))

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
    local angle = math.acos(wp_dir:dot(veh.dirVec))

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
    --debugDrawer:drawSphere((wp1_pos + vec3(0,0,2)):toPoint3F(), 0.5, ColorF(1,1,0,1))
    --debugDrawer:drawTextAdvanced((wp1_pos + vec3(0,0,3)):toPoint3F(), String("ID: " .. tostring(i)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
    
    --debugDrawer:drawSphere((wp2_pos + vec3(0,0,2)):toPoint3F(), 0.5, ColorF(1,1,0,1))
    --debugDrawer:drawTextAdvanced((wp2_pos + vec3(0,0,3)):toPoint3F(), String("ID: " .. tostring(i + 1)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  end

  local new_wps_props = getWaypointsProperties(min_wp_angle[2], min_wp_angle[3], wps_props.lat_dist_from_wp)

  return new_wps_props
end

--[[

--Gets whether we are to the left or right of waypoints
local function getWhichSideOfWaypointsCarIsOn(, start_pos, end_pos)
  local road_line_dir = toNormXYVec(end_pos - start_pos)
  local car_dir_xy = toNormXYVec(.dir)

  local wp_mid_pos = (end_pos - start_pos) * 0.5 + start_pos

  --debugDrawer:drawSphere(start_pos:toPoint3F(), 0.5, ColorF(1,0,0,1))
  --debugDrawer:drawSphere(wp_mid_pos:toPoint3F(), 0.5, ColorF(0,1,0,1))
  --debugDrawer:drawSphere(end_pos:toPoint3F(), 0.5, ColorF(0,0,1,1))

  --debugDrawer:setSolidTriCulling(false)
  --debugDrawer:drawQuadSolid((start_pos):toPoint3F(), (start_pos + vec3(0,0,1)):toPoint3F(),
  --(end_pos + vec3(0,0,1)):toPoint3F(), (end_pos):toPoint3F(), ColorF(1,0,0,0.25))

  --Get angle between car dir and road dir that is between 0 and 360 degrees
  local line_to_left_side_dir = (.center_pos - .dir_right * .bb:getHalfExtents().x * 0.5 - wp_mid_pos):normalized()
  local line_to_right_side_dir = (.center_pos + .dir_right * .bb:getHalfExtents().x * 0.5 - wp_mid_pos):normalized()
  local line_to_center_dir = (.center_pos - wp_mid_pos):normalized()

  local left_dot = line_to_left_side_dir:dot(road_line_dir)
  local left_det = line_to_left_side_dir.x * road_line_dir.y - line_to_left_side_dir.y * road_line_dir.x

  local right_dot = line_to_right_side_dir:dot(road_line_dir)
  local right_det = line_to_right_side_dir.x * road_line_dir.y - line_to_right_side_dir.y * road_line_dir.x

  local center_dot = line_to_center_dir:dot(road_line_dir)
  local center_det = line_to_center_dir.x * road_line_dir.y - line_to_center_dir.y * road_line_dir.x

  local left_angle = math.atan2(left_det, left_dot) * 180.0 / math.pi
  local right_angle = math.atan2(right_det, right_dot) * 180.0 / math.pi
  local center_angle = math.atan2(center_det, center_dot) * 180.0 / math.pi

  local side = nil
  local in_wp_middle = false

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
      in_wp_middle = true
    end
    --Left side
  else
    if side == "right" then
      in_wp_middle = true
    end
  end
  
   --Right side
  if center_angle > 0 and center_angle < 180 then
    side = "right"
    --Left side
  else
    side = "left"
  end
  
  return side, in_wp_middle
end

local function getLaneNum(, wps_props, side_of_wps)
  local lane_num = -1

  if wps_props.num_of_lanes == 1 then
    return 0
  end

  local even_lanes = wps_props.num_of_lanes % 2 == 0

  local lanes_from_wp = 0
  --Waypoint is in middle of lane lines when even
  --Waypoint is in middle of lane when odd
  if even_lanes then
    lanes_from_wp = math.floor(wps_props.lat_dist_from_wp / wps_props.lane_width)
    
    if side_of_wps == "right" then
      lane_num = wps_props.num_of_lanes / 2.0 + lanes_from_wp
    else
      lane_num = wps_props.num_of_lanes / 2.0 - lanes_from_wp - 1
    end
  else
    lanes_from_wp = math.floor(wps_props.lat_dist_from_wp / wps_props.lane_width + 0.5)
    
    if side_of_wps == "right" then
      lane_num = wps_props.num_of_lanes / 2.0 + lanes_from_wp - 0.5
    else
      lane_num = wps_props.num_of_lanes / 2.0 - lanes_from_wp - 1 + 0.5
    end
  end

  --Cap between 0 and wps_props.num_of_lanes - 1
  lane_num = math.min(wps_props.num_of_lanes - 1, math.max(lane_num, 0))

  return lane_num
end

--Check if other car is on the same road as me (not lane)
local function checkIfOtherCarOnSameRoad(my_, other_, wps_props)
  local xnorm = other_.center_pos:xnormOnLine(wps_props.start_wp_pos, wps_props.end_wp_pos)
  local my_wp_dir = wps_props.end_wp_pos - wps_props.start_wp_pos
  local other_veh_pos_on_wp_line = xnorm * my_wp_dir + wps_props.start_wp_pos
  
  --Get waypoint on my road closest to other vehicle
  local new_wps_props = getWaypointStartEnd(my_, other_veh_pos_on_wp_line)
  
  if new_wps_props.start_wp == nil then
    return true
  end
  
   --Get lateral distance of other car to waypoint
  local xnorm2 = other_.center_pos:xnormOnLine(new_wps_props.start_wp_pos, new_wps_props.end_wp_pos)
  local new_my_wp_start_end = new_wps_props.end_wp_pos - new_wps_props.start_wp_pos
  local new_other_veh_pos_on_wp_line = xnorm2 * new_my_wp_start_end + new_wps_props.start_wp_pos
  
  local lat_dist = (other_.center_pos - new_other_veh_pos_on_wp_line):length()

  --print(lat_dist)

  return lat_dist < wps_props.wp_radius
end

local function getCircularDistance(my_, other_, min_distance_from_car)
  local other_bb = other_.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local shoot_ray_dir = (my_.front_pos - other_.center_pos):normalized()

  --Now get exact distance
  local max_distance, min_distance = intersectsRay_OBB(my_.front_pos, shoot_ray_dir, other_.center_pos, other_x, other_y, other_z)

  --For some reason, distance is negative
  min_distance = math.abs(min_distance) - min_distance_from_car

  if min_distance < 0 then
    min_distance = 0
  end

  --Convert to circular distance
  local turning_radius = math.abs(my_.speed / angular_speed_angelo234)

  local angle = math.acos(
    (min_distance * min_distance) / (-2 * turning_radius * turning_radius) + 1)

  local cir_dist = angle * turning_radius

  return cir_dist
end

local function getStraightDistance(my_, other_, min_distance_from_car, front)
  local other_bb = other_.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local ray_pos = nil

  if front then
    ray_pos = my_.front_pos
  else
    ray_pos = my_.rear_pos
  end

  local shoot_ray_dir = (ray_pos - other_.center_pos):normalized()

  --Now get exact distance
  local max_distance, min_distance = intersectsRay_OBB(ray_pos, shoot_ray_dir, other_.center_pos, other_x, other_y, other_z)

  --For some reason, distance is negative
  min_distance = math.abs(min_distance) - min_distance_from_car

  if min_distance < 0 then
    min_distance = 0
  end

  return min_distance
end

--Returns a table of vehicles and distance to them within a max_dist radius
--only in front of our vehicle, so it discards vehicles behind
local function getNearbyVehicles(my_, max_dist, min_distance_from_car, in_front)
  local other_vehs = {}

  local vehicles = getAllVehicles()

  for _, other_veh in pairs(vehicles) do
    local other_ = getVehicleProperties(other_veh)

    if other_.id ~= my_.id then
      --Get aproximate distance first between vehicles and return if less than max dist
      local other_bb = other_.bb

      local front_dist = (my_.front_pos - other_.center_pos):length()
      local rear_dist = (my_.rear_pos - other_.center_pos):length()

      --If rear distance is larger than front distance, then vehicle is in front
      if front_dist < max_dist and front_dist < rear_dist and in_front then
        local cir_dist = getCircularDistance(my_, other_, min_distance_from_car)
        
        local other_veh_data = 
        {
          other_veh = other_veh, 
          distance = cir_dist
        }
        
        table.insert(other_vehs, other_veh_data)
              
        --If front distance is larger than rear distance, then vehicle is in rear
      elseif rear_dist < max_dist and front_dist > rear_dist and not in_front then
        local dist = getStraightDistance(my_, other_, min_distance_from_car, false)
        
        local other_veh_data = 
        {
          other_veh = other_veh, 
          distance = dist
        }
        
        table.insert(other_vehs, other_veh_data)
      end
    end
  end

  return other_vehs
end

--Returns a table of vehicles and distance to them within a max_dist radius and in the same lane
local function getNearbyVehiclesInSameLane(my_, max_dist, min_distance_from_car, in_front)
  local my_veh_wps_props = getWaypointStartEndAdvanced(my_, my_, my_.front_pos)
  
  if my_veh_wps_props == nil then
    return {}
  end
  
  local my_veh_side_of_wp, my_in_wp_middle = getWhichSideOfWaypointsCarIsOn(my_, my_veh_wps_props.start_wp_pos, my_veh_wps_props.end_wp_pos)
  local my_veh_lane_num = getLaneNum(my_, my_veh_wps_props, my_veh_side_of_wp)
  
  --debugDrawer:drawTextAdvanced((my_.front_pos):toPoint3F(), String("Lane Num: " .. tostring(my_veh_lane_num)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  --print("My Lane: " .. my_veh_lane_num)
  
  my_veh_wps_props.side_of_wp = my_veh_side_of_wp
  my_veh_wps_props.in_wp_middle = my_in_wp_middle
  
  local other_vehs_data = getNearbyVehicles(my_, max_dist, min_distance_from_car, in_front)
  local other_vehs_in_my_lane = {}

  for _, other_veh_data in pairs(other_vehs_data) do
    local other_ = getVehicleProperties(other_veh_data.other_veh)
    local speed_rel = my_.speed - other_.speed
    
    local other_veh_wps_props = getWaypointStartEndAdvanced(my_, other_, other_.front_pos)
    
    local other_veh_side_of_wp, other_in_wp_middle = getWhichSideOfWaypointsCarIsOn(other_, other_veh_wps_props.start_wp_pos, other_veh_wps_props.end_wp_pos)
    local other_veh_lane_num = getLaneNum(other_, other_veh_wps_props, other_veh_side_of_wp)
    
    --debugDrawer:drawTextAdvanced((other_.front_pos):toPoint3F(), String("Lane Num: " .. tostring(other_veh_lane_num)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
    --print("Other Lane: " .. other_veh_lane_num)
    
    other_veh_wps_props.side_of_wp = other_veh_side_of_wp
    other_veh_wps_props.in_wp_middle = other_in_wp_middle

    other_veh_data.my_veh_wps_props = my_veh_wps_props
    other_veh_data.other_veh_wps_props = other_veh_wps_props

    local on_same_road = checkIfOtherCarOnSameRoad(my_, other_, my_veh_wps_props)

    --debugDrawer:drawTextAdvanced((other_.front_pos):toPoint3F(), String("Same Road: " .. tostring(on_same_road)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

    --In same lane or either vehicle in middle of road and my vehicle speed is >= to other
    if (my_in_wp_middle or other_in_wp_middle or (my_veh_lane_num == other_veh_lane_num and on_same_road)) and speed_rel >= 0 then
      table.insert(other_vehs_in_my_lane, other_veh_data)
    end
  end

  return other_vehs_in_my_lane
end

]]--

local function init(jbeamData)
  if mapmgr.mapData == nil then
    mapmgr.requestMap()
  end
end

local smoothing_table = {}

local function updateGFX(dt)
  --update stuff
  
  --all this to get acceleration vectors from vehicles
  
  if tableSize(mapmgr.objects) ~= tableSize(smoothing_table) then
    smoothing_table = {}
    
    for id, _ in pairs(mapmgr.objects) do
      smoothing_table[id] = {}
    
      smoothing_table[id].x = newExponentialSmoothing(30)
      smoothing_table[id].y = newExponentialSmoothing(30)
      smoothing_table[id].z = newExponentialSmoothing(30)
    end
  end
  
  if tableSize(smoothing_table) > 0 then
    --acceleration
    for id, _ in pairs(mapmgr.objects) do
      if mapmgr.objects[id] and prev_mapmgr_objects[id] then
        local curr_vel = mapmgr.objects[id].vel
        local prev_vel = prev_mapmgr_objects[id].vel
      
        local acc = (curr_vel - prev_vel) / dt
        
        if smoothing_table[id] then
          local new_acc = vec3(smoothing_table[id].x:get(acc.x), smoothing_table[id].y:get(acc.y), smoothing_table[id].z:get(acc.z))

          mapmgr.objects[id].acc = new_acc
        else
          smoothing_table = {}
        end
      end
    end
  end

  prev_mapmgr_objects = deepcopy(mapmgr.objects)
end



M.rotateEuler = rotateEuler
M.toNormXYVec = toNormXYVec
M.getFuturePosition = getFuturePosition
M.getFuturePositionXY = getFuturePositionXY
M.getFuturePositionXYWithAcc = getFuturePositionXYWithAcc
M.getWaypointPosition = getWaypointPosition
M.getLaneWidth = getLaneWidth
M.getWaypointStartEnd = getWaypointStartEnd
M.getWaypointStartEndAdvanced = getWaypointStartEndAdvanced
M.getWhichSideOfWaypointsCarIsOn = getWhichSideOfWaypointsCarIsOn
M.getNearbyVehicles = getNearbyVehicles
M.getNearbyVehiclesInSameLane = getNearbyVehiclesInSameLane
M.init = init
M.updateGFX = updateGFX

return M
