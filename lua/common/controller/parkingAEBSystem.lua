local M = {}

local system_params = nil
local parking_lines_params = nil
local rev_aeb_params = nil
local beeper_params = nil

local veh_name = nil

local car_dir = vec3(obj:getDirectionVector())
local car_dir_up = vec3(obj:getDirectionVectorUp())
local car_dir_right = vec3(obj:getDirectionVectorRight())
local front_pos = vec3(obj:getFrontPosition()) + car_dir * 0.5
local rear_pos = front_pos + -car_dir * obj:getInitialLength() 
local speed = vec3(obj:getVelocity()):length()
local acc_vec = quatFromDir(car_dir, vec3(0,0,1)) * vec3(sensors.gx2, sensors.gy2, 9.81 + sensors.gz2)

local static_sensor_id = -1
local prev_min_dist = 9999
local min_dist = 9999

local system_active = false

local function staticCastRay(sensorPos, same_ray)
  local hit = nil
  
  local car_half_width = rev_aeb_params.safety_offset_width_sensor + obj:getInitialWidth() / 2.0

  if not same_ray then
    if static_sensor_id >= rev_aeb_params.num_of_sensors - 1 then static_sensor_id = -1 end
    static_sensor_id = static_sensor_id + 1
  end

  local pos = sensorPos + car_dir_right * (car_half_width - car_half_width / ((rev_aeb_params.num_of_sensors - 1) / 2.0) * static_sensor_id)

  --use castRayDebug to show lines
  --hit = castRay(pos, dest, true, true)
  --local param_arr = {origin, dest, false, true, index, true} 
  --obj:queueGameEngineLua("be:getPlayerVehicle(0):queueLuaCommand('raycastdata = ' .. aeb_angelo234_castRay('" ..jsonEncode(param_arr) .. "'))")
  --local data = jsonDecode(raycastdata)

  local dist = obj:castRayStatic(pos:toFloat3(), (-car_dir):toFloat3(), rev_aeb_params.sensor_max_distance)

  if dist == rev_aeb_params.sensor_max_distance then
    return 9999
  end

  return dist

  --if hit == nil then return nil end

  --return {hit.norm, hit.dist, hit.pt, static_sensor_id}
end

local function processRayCasts(static_hit, vehicle_hit)
  --static hit returns {hit.norm, hit.dist, hit.pt, static_sensor_id}
  --vehicle hit returns {other_veh, min_distance, veh_sensor_id}

  local static_dist = 9999
  local vehicle_dist = 9999
  local other_veh = nil
  
  static_dist = static_hit
  
  --[[
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
  ]]--
  
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

local beeper_timer = 0

local function soundBeepers(dt, dist)
  beeper_timer = beeper_timer + dt

  if dist <= parking_lines_params.parking_line_total_len + parking_lines_params.parking_line_offset_long then
    
    --If object is within red line distance, play constant tone
    if dist <= parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_offset_long then
      if beeper_timer >= 1.0 / beeper_params.parking_warning_tone_hertz then
        obj:queueGameEngineLua("Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')")
        beeper_timer = 0
      end
      
    --Else tone depends on distance
    else
      if beeper_timer >= dist / beeper_params.parking_warning_tone_dist_per_hertz then
        obj:queueGameEngineLua("Engine.Audio.playOnce('AudioGui','core/art/sound/proximity_tone_50ms.wav')")
        beeper_timer = 0
      end
    end
  end
end

local function pollReverseSensors(dt)
  --Fixes lag of sensorPos
  local sensorPos = rear_pos --+ (rev_aeb_params.num_of_sensors / rev_aeb_params.sensors_polled_per_iteration) * vec3(obj:getVelocity()) * dt 
    + car_dir_up * rev_aeb_params.parking_sensor_rel_height + car_dir * rev_aeb_params.sensor_offset_forward
  
  local static_hit = staticCastRay(sensorPos, false)

  --Get vehicles in a 7.5m radius behind my vehicle
  local other_vehs_data = controller.getController("extraUtils").getNearbyVehicles(rev_aeb_params.sensor_max_distance, 0, false)
  local vehicle_hit = getClosestVehicle(other_vehs_data)

  local other_veh, min_dist = processRayCasts(static_hit, vehicle_hit)

  min_dist = min_dist - rev_aeb_params.sensor_offset_forward - 0.1

  return other_veh, min_dist
end

local function performEmergencyBraking(dt, distance)
  --If vehicle is below certain speed then deactivate system
  if speed <= rev_aeb_params.min_speed then
    --But if system activated before, then release brakes and apply parking brake
    if system_active then
      --Release brake and apply parking brake
      --Must do differently depending on gearbox mode :/
      if controller.mainController.onSerialize().previousGearboxBehavior == "realistic" then      
        input.event('brake', 0, 2)
      else
        input.event('throttle', 0, 2)
      end
      
      input.event('parkingbrake', 1, 2)
      system_active = false
    end
    return 
  end    

  --Maximum Braking
  if system_active then
    --Must do differently depending on gearbox mode :/
    if controller.mainController.onSerialize().previousGearboxBehavior == "realistic" then
      input.event('brake', 1, 2)
    else
      input.event('throttle', 1, 2)
    end
    return
  end
  
  --Max braking acceleration = gravity * coefficient of static friction
  local acc = system_params.gravity * system_params.rev_friction_coeff

  --Calculate time to collision (TTC)
  local ttc = distance / speed
  local time_to_brake = speed / (2 * acc)

  --leeway time depending on speed
  local time_before_braking = ttc - time_to_brake

  if time_before_braking <= 0 then
    system_active = true
  end
end

local function init(jbeamData)
  veh_name = v.config.model
  
  local default_param_file_dir = 'vehicles/common/parameters'
  local param_file_dir = 'vehicles/' .. veh_name .. '/parameters'
  
  if FS:fileExists(param_file_dir) then
    --load parameter lua file dependent on vehicle
    system_params = require(param_file_dir)
  else
    --use default parameters if they don't exist for current vehicle
    system_params = require(default_param_file_dir)
  end
  
  parking_lines_params = system_params.rev_cam_params.parking_lines_params
  rev_aeb_params = system_params.rev_aeb_params
  beeper_params = system_params.beeper_params
end

local function updateGFX(dt)
  --update stuff
  car_dir:set(obj:getDirectionVector())
  car_dir_up:set(obj:getDirectionVectorUp())
  car_dir_right:set(obj:getDirectionVectorRight())
  front_pos:set(vec3(obj:getFrontPosition()) + car_dir * 0.5)
  rear_pos:set(front_pos + -car_dir * obj:getInitialLength())
  speed = vec3(obj:getVelocity()):length()

  local in_reverse = electrics.values.reverse
  local gear_selected = electrics.values.gear

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then 
    prev_min_dist = 9999
    min_dist = 9999
    return 
  end
  
  --Play beeping sound based on min distance of prev sensor detections to obstacle
  soundBeepers(dt, prev_min_dist)

  for i = 1, rev_aeb_params.sensors_polled_per_iteration do
    if static_sensor_id == rev_aeb_params.num_of_sensors - 1 then
      prev_min_dist = min_dist
      min_dist = 9999
    end
  
    --Get distance and other data of nearest obstacle 
    local other_veh, dist = pollReverseSensors(dt)
     
    min_dist = math.min(dist, min_dist)
  end

  --if not aeb_enabled then return end 

  --print(min_dist)
  
  performEmergencyBraking(dt, min_dist)
  
  local extra_utils = controller.getController("extraUtils")
  
  local future_pos = extra_utils.getFuturePosition(obj:getID(), 1, "front")
  
  --obj.debugDrawProxy:drawSphere(0.25, future_pos:toFloat3(), color(255,0,0,255))
  
  for id, vals in pairs(mapmgr.objects) do
    local front_pos = vec3(obj:getObjectFrontPosition(id))
  
    --obj.debugDrawProxy:drawSphere(0.25, front_pos:toFloat3(), color(255,0,0,255))
  end
end

M.init = init
M.updateGFX = updateGFX

return M
