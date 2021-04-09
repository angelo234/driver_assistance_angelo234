require("lua/common/luaProfiler")

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtilsGE')

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

    if my_veh_side == "right" then
       my_lat_dist_from_wp = my_lat_dist_from_wp + my_speed_in_wp_perp_dir * ttc
    
      if my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
      or my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
      then
        overlap_at_ttc = true
      end    
    else
       my_lat_dist_from_wp = my_lat_dist_from_wp - my_speed_in_wp_perp_dir * ttc
    
      if my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
      or my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
      then
        overlap_at_ttc = true
      end  
    end

    --Collision may be possible
    if overlap_at_ttc then
      debugDrawer:drawSphere((other_veh_props.center_pos):toPoint3F(), 1, ColorF(1,0,0,1))
    
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

--Global variables as IO between Vehicle and GameEngine Lua
ve_json_params_angelo234 = nil
ge_aeb_data_angelo234 = "'[9999,0]'"

--Executing functions here because you get extreme lag for some reason
--executing them in Vehicle Lua. Will then send results back
local function getNearestVehicleInPathForVELua(dt)
  if ve_json_params_angelo234 == nil or ve_json_params_angelo234 == 'nil' then 
    ge_aeb_data_angelo234 = "'[9999,0]'"
    return 
  end
  
  local params = jsonDecode(ve_json_params_angelo234)

  local my_veh_id = params[1]
  local aeb_params = params[2]
  local min_distance_from_car = params[3]

  local veh = be:getObjectByID(my_veh_id)
  
  if veh == nil then return end
  
  local veh_props = extra_utils.getVehicleProperties(veh)
  
  local data = extra_utils.getNearbyVehiclesInSameLane(veh_props, aeb_params.vehicle_search_radius, min_distance_from_car, true)

  --Determine if a collision will actually occur and return the distance and relative velocity 
  --to the vehicle that I'm planning to collide with
  local distance, vel_rel = getNearestVehicleInPath(dt, veh_props, data, aeb_params.lateral_acc_to_avoid_collision)

  --Takes 1 frame to actually send data so account for that
  distance = distance - vel_rel * dt 

  ge_aeb_data_angelo234 = "'" .. jsonEncode({distance, vel_rel}) .. "'"
end

local function update(dt)
  if be:getEnabled() then
    getNearestVehicleInPathForVELua(dt)
  end
end

M.update = update

return M