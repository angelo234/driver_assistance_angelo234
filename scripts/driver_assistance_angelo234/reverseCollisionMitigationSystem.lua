local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local system_active = false

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

local function update(dt, veh, system_params, parking_lines_params, rev_aeb_params, beeper_params, rear_sensor_data)
  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then
    return 
  end

  local veh_props = extra_utils.getVehicleProperties(veh)

  local other_veh = rear_sensor_data[1]
  local min_dist = rear_sensor_data[2]

  if extra_utils.checkIfPartExists("reverse_aeb_angelo234") then
    performEmergencyBraking(dt, veh, min_dist, veh_props.speed, system_params, rev_aeb_params)
  end
  
  if extra_utils.checkIfPartExists("reverse_collision_warning_angelo234") then
    --Play beeping sound based on min distance of prev sensor detections to obstacle
    soundBeepers(dt, min_dist, parking_lines_params, beeper_params)
  end

end

M.update = update

return M