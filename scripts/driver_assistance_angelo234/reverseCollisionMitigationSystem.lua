local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local system_state = "ready"

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

local function performEmergencyBraking(dt, veh, veh_props, distance, speed, system_params, rev_aeb_params)
  if veh_props.speed <= rev_aeb_params.min_speed then return end

  --Max braking acceleration = gravity * coefficient of static friction
  local acc = system_params.gravity * system_params.rev_friction_coeff

  --Calculate time to collision (TTC)
  local ttc = distance / speed
  local time_to_brake = speed / (2 * acc)

  local time_before_braking = ttc - time_to_brake

  if time_before_braking <= 0 then
    system_state = "braking"
  end

  --Maximum Braking
  if system_state == "braking" then
    --Must do differently depending on gearbox mode :/
    if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      veh:queueLuaCommand("electrics.values.brakeOverride = 1")
    else
      veh:queueLuaCommand("electrics.values.brakeOverride = nil")
      veh:queueLuaCommand("electrics.values.throttleOverride = 1")
    end
    return
  end
end

local function holdBrakes(veh, veh_props, rev_aeb_params)
  --If system activated before and below certain speed then release brakes and apply parking brake
  if veh_props.speed <= rev_aeb_params.min_speed and system_state == "braking" then
    --When coming to a stop with system activated, release brakes but apply parking brake in arcade mode :P
    if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
      veh:queueLuaCommand("electrics.values.brakeOverride = 1")
    else
      --Release brake and apply parking brake
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      veh:queueLuaCommand("input.event('parkingbrake', 1, 2)")
    end

    system_state = "holding"
  end

  --If vehicle held by brake after AEB and user modulates throttle or brake pedal then release brakes
  if system_state == "holding" and (input_throttle_angelo234 > 0 or input_brake_angelo234 > 0) then
    if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
      veh:queueLuaCommand("electrics.values.brakeOverride = nil")
    else
      veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
    end
    veh:queueLuaCommand("electrics.values.throttleOverride = nil")

    system_state = "ready"
  end

  return system_state == "holding"
end

local function update(dt, veh, system_params, parking_lines_params, rev_aeb_params, beeper_params, rear_sensor_data)
  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then
    if system_state ~= "ready" then
      veh:queueLuaCommand("electrics.values.brakeOverride = nil")
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      system_state = "ready"
    end

    return
  end

  local veh_props = extra_utils.getVehicleProperties(veh)

  local other_veh = rear_sensor_data[1]
  local min_dist = rear_sensor_data[2] - 0.1

  if extra_utils.getPart("reverse_collision_warning_angelo234") then
    --Play beeping sound based on min distance of prev sensor detections to obstacle
    soundBeepers(dt, min_dist, parking_lines_params, beeper_params)
  end

  if holdBrakes(veh, veh_props, rev_aeb_params) then return end

  if extra_utils.getPart("reverse_aeb_angelo234") then
    performEmergencyBraking(dt, veh, veh_props, min_dist, veh_props.speed, system_params, rev_aeb_params)
  end

end

M.update = update

return M