local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local rev_aeb_on = true

local system_active = false

local function getSystemOnOff()
  return rev_aeb_on
end

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
  hit = castRay(pos, dest, true, true)

  if hit == nil then return nil end

  return {hit.norm, hit.dist, hit.pt, static_sensor_id}
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
  
  --Max braking acceleration = gravity * coefficient of static friction
  local acc = system_params.gravity * system_params.rev_friction_coeff

  --Calculate time to collision (TTC)
  local ttc = distance / speed
  local time_to_brake = speed / (2 * acc)

  local time_before_braking = ttc - time_to_brake

  if time_before_braking <= 0 then
    system_active = true
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

local function processData(rear_sensor_data, rev_aeb_params)
  local vehicle_hit = getClosestVehicle(rear_sensor_data[2])
  
  local vehicle_dist = 9999
  local other_veh = nil

  if vehicle_hit[1] ~= nil then
    other_veh = vehicle_hit[1]
    vehicle_dist = vehicle_hit[2]
  end

  local min_dist = math.min(rear_sensor_data[1], vehicle_dist)

  --Vehicle is closest
  --if min_dist ~= 9999 and min_dist == vehicle_dist then

  --end

  min_dist = min_dist - rev_aeb_params.sensor_offset_forward - 0.15

  return other_veh, min_dist
end

local function update(dt, veh, system_params, parking_lines_params, rev_aeb_params, beeper_params, rear_sensor_data)
  if not rev_aeb_on then return end
  
  local veh_props = extra_utils.getVehicleProperties(veh)
  
  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then
    return 
  end

  local other_veh, min_dist = processData(rear_sensor_data, rev_aeb_params)

  --Play beeping sound based on min distance of prev sensor detections to obstacle
  soundBeepers(dt, min_dist, parking_lines_params, beeper_params)
  
  performEmergencyBraking(dt, veh, min_dist, veh_props.speed, system_params, rev_aeb_params)
end

M.getSystemOnOff = getSystemOnOff
M.toggleSystem = toggleSystem
M.update = update

return M