local M = {}

local control_systems = require("controlSystems") -- for newPIDStandard
local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

--PID for getting vehicle up to desired speed when no car in front
--Input = desired velocity, Output = pedal position
local speed_pid = newPIDStandard(0.2, 2, 0.0, 0, 1, 1, 1, 0, 2)
local speed_smooth = newTemporalSmoothing(200, 200)

--PID for setting car to following distance
--Input = desired distance, Output = pedal position
local dist_pid = newPIDStandard(0.06, 2, 1, -0.3, 0.3)
local dist_smooth = newTemporalSmoothing(200, 200)

local target_acceleration = 2.5

local target_speed = 13.8
local ramped_target_speed = 0

local following_time = 1

local following_mode = false

local function onToggled(acc_on)
  if acc_on then
    local display_speed = target_speed

    local units = settings.getValue("uiUnitLength")
    local the_unit = "m/s"

    if units == "metric" then
      the_unit = "kph"
      display_speed = display_speed * 3.6
    elseif units == "imperial" then
      the_unit = "mph"
      display_speed = display_speed * 2.24
    end

    display_speed = math.floor(display_speed / 5 + 0.5) * 5

    ui_message("Adaptive Cruise Control switched ON (" .. display_speed .. " " .. the_unit .. ")")

  else
    speed_pid:reset()
    speed_smooth:reset()
    dist_pid:reset()
    dist_smooth:reset()

    local veh = be:getPlayerVehicle(0)

    veh:queueLuaCommand("electrics.values.throttleOverride = nil")
    veh:queueLuaCommand("electrics.values.brakeOverride = nil")

    ramped_target_speed = 0

    ui_message("Adaptive Cruise Control switched OFF")
  end
end

local function setACCSpeed()
  local my_veh = be:getPlayerVehicle(0)
  local veh_speed = vec3(my_veh:getVelocity()):length()

  if veh_speed <= 8.3 then
    ui_message("ERROR Adaptive Cruise Control must be set above 30 km/h (20 mph)")
    return
  end

  target_speed = veh_speed
  ramped_target_speed = veh_speed

  --Display speed in user's units

  local units = settings.getValue("uiUnitLength")
  local the_unit = "m/s"

  if units == "metric" then
    the_unit = "kph"
    veh_speed = veh_speed * 3.6
  elseif units == "imperial" then
    the_unit = "mph"
    veh_speed = veh_speed * 2.24
  end

  veh_speed = math.floor(veh_speed / 5 + 0.5) * 5

  ui_message("Adaptive Cruise Control speed set to " .. tostring(veh_speed) .. " " .. the_unit)
end

local function changeACCSpeed(amt)
  local my_veh = be:getPlayerVehicle(0)
  local veh_speed = vec3(my_veh:getVelocity()):length()

  --amt is 1 or -1 so increment/decrement by user's units

  local display_speed = target_speed

  local units = settings.getValue("uiUnitLength")
  local the_unit = "m/s"

  if units == "metric" then
    the_unit = "kph"
    target_speed = target_speed + amt / 3.6

    display_speed = target_speed * 3.6
  elseif units == "imperial" then
    the_unit = "mph"
    target_speed = target_speed + amt / 2.24

    display_speed = target_speed * 2.24
  end


  --Minimum of 30 km/h or 20 mph

  if math.floor(target_speed) < 8.34 then
    target_speed = 8.34
    if units == "metric" then
      display_speed = 30

    elseif units == "imperial" then
      display_speed = 20
    end
  end

  ramped_target_speed = veh_speed

  display_speed = math.floor(display_speed / 5 + 0.5) * 5

  ui_message("Adaptive Cruise Control speed set to " .. tostring(display_speed) .. " " .. the_unit)
end

local function changeACCFollowingDistance(amt)
  following_time = following_time + amt

  --0.5 ~ 3 seconds
  if following_time <= 0 then
    following_time = 0.5
  elseif following_time > 3 then
    following_time = 3
  end

  ui_message("Adaptive Cruise Control following distance set to " .. tostring(following_time) .. "s")
end

local function getVehicleAheadInLane(dt, my_veh_props, data_table)
  local distance = 9999
  local other_veh_vel = 0
  local curr_veh_in_path = nil

  --Analyze the trajectory of other vehicles with my trajectory
  --to see if collision imminent
  for _, data in pairs(data_table) do
    --Other vehicle properties
    local other_veh_props = extra_utils.getVehicleProperties(data.other_veh)

    --Vehicle must be facing in same dir as my vehicle
    if math.acos(my_veh_props.dir:dot(other_veh_props.dir)) < math.pi / 2.0 then

      local this_rel_vel = (my_veh_props.velocity - other_veh_props.velocity):length()

      --Capping to 5 seconds to prevent too much error in predicting position
      local ttc = math.min(data.distance / this_rel_vel, 5)

      if data.my_veh_wps_props ~= nil and data.other_veh_wps_props ~= nil then

        local my_lat_dist_from_wp = data.my_veh_wps_props.lat_dist_from_wp
        local other_lat_dist_from_wp = data.other_veh_wps_props.lat_dist_from_wp

        if my_lat_dist_from_wp - my_veh_props.bb:getHalfExtents().x < other_lat_dist_from_wp + other_veh_props.bb:getHalfExtents().x
        and my_lat_dist_from_wp + my_veh_props.bb:getHalfExtents().x > other_lat_dist_from_wp - other_veh_props.bb:getHalfExtents().x
        then
          --This may be the car to adapt to

          --If this distance is less than current min distance
          --then this is new min distance
          if data.distance <= distance then
            distance = data.distance
            other_veh_vel = other_veh_props.velocity

            curr_veh_in_path = data.other_veh
          end
        else
          --debugDrawer:drawTextAdvanced((other_veh_props.front_pos), String("Delta distance: " .. math.abs(my_lat_dist_from_wp - other_lat_dist_from_wp)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
        end
      end
    end
  end

  if curr_veh_in_path then
    --debugDrawer:drawSphere((vec3(curr_veh_in_path:getPosition())), 1, ColorF(1,0,0,1))
  end

  return distance, other_veh_vel
end

local function accelerateVehicle(dt, veh, output)
  if output > 0 then
    --Accelerate
    veh:queueLuaCommand("electrics.values.throttleOverride = " .. output)
    veh:queueLuaCommand("electrics.values.brakeOverride = 0")
  else
    --Brake
    veh:queueLuaCommand("electrics.values.brakeOverride = " .. -output)
    veh:queueLuaCommand("electrics.values.throttleOverride = 0")
  end
end

--Maintaining distance to vehicle using PID controller
local function maintainDistanceFromVehicleAhead(dt, veh, veh_props, aeb_params, distance, following_distance, only_braking)
  local output = dist_pid:get(-(distance), -following_distance, dt)

  output = dist_smooth:getUncapped(output, dt)

  if (only_braking and output < 0) or not only_braking then
    accelerateVehicle(dt, veh, output)
  end

  return output
end

local function maintainSetSpeed(dt, veh, veh_props, aeb_params)
  --Code from BeamNG's cruiseControl.lua

  --ramp up/down our target speed with our desired target acceleration to avoid integral wind-up
  if ramped_target_speed ~= target_speed then
    local upper_limit = target_speed > ramped_target_speed and target_speed or ramped_target_speed
    local lower_limit = target_speed < ramped_target_speed and target_speed or ramped_target_speed
    ramped_target_speed = clamp(ramped_target_speed + fsign(target_speed - ramped_target_speed) * target_acceleration * dt, lower_limit, upper_limit)
  end

  local currentSpeed = veh_props.speed
  local output = speed_pid:get(currentSpeed, ramped_target_speed, dt)
  output = speed_smooth:getUncapped(output, dt)

  accelerateVehicle(dt, veh, output)

  --print("Desired Acceleration: " .. output)
end

local function update(dt, veh, system_params, aeb_params, front_sensor_data)
  local veh_props = extra_utils.getVehicleProperties(veh)

  local in_reverse = electrics_values_angelo234["reverse"]
  local gear_selected = electrics_values_angelo234["gear"]
  local esc_color = electrics_values_angelo234["dseColor"]

  --ESC must be in comfort mode, otherwise it is deactivated
  --sunburst uses different color for comfort mode
  --if (veh_name == "sunburst" and esc_color == "98FB00")
   --or (veh_name ~= "sunburst" and esc_color == "238BE6")
  --then

  --Deactivate system based on any of these variables
  if in_reverse == nil or in_reverse == 1 or gear_selected == nil
    or gear_selected == 'P' then
    scripts_driver__assistance__angelo234_extension.setACCSystemOn(false)
    return
  end

  --If user presses any pedals, then deactivate system

  if input_throttle_angelo234 > 0 or input_clutch_angelo234 > 0 then
    veh:queueLuaCommand("electrics.values.throttleOverride = " .. input_throttle_angelo234)
  end

  --Turn off ACC system if braking
  if input_brake_angelo234 > 0 then
    veh:queueLuaCommand("electrics.values.brakeOverride = " .. input_brake_angelo234)
    scripts_driver__assistance__angelo234_extension.setACCSystemOn(false)
  end

  if input_throttle_angelo234 > 0 or input_brake_angelo234 > 0 or input_clutch_angelo234 > 0 then return end

  local distance = 9999
  local other_veh_vel = 0

  --If table is empty then return
  if next(front_sensor_data[3]) ~= nil then
    --Determine if a collision will actually occur and return the distance and relative velocity
    --to the vehicle that I'm planning to collide with
    distance, other_veh_vel = getVehicleAheadInLane(dt, veh_props, front_sensor_data[3])
  end

  --5 meter of leeway
  local following_distance = math.max(veh_props.speed * following_time, 5)

  local output = maintainDistanceFromVehicleAhead(dt, veh, veh_props, aeb_params, distance, following_distance, true)

  if output < 0 then
    ramped_target_speed = veh_props.speed
    speed_pid:reset()
    speed_smooth:reset()
  else
    if veh_props.speed > target_speed then
      maintainSetSpeed(dt, veh, veh_props, aeb_params)
    else
      if distance < following_distance * 1.5 then
        maintainDistanceFromVehicleAhead(dt, veh, veh_props, aeb_params, distance, following_distance, false)

        ramped_target_speed = veh_props.speed
        speed_pid:reset()
        speed_smooth:reset()
      else
        maintainSetSpeed(dt, veh, veh_props, aeb_params)
      end
    end
  end
end

M.onToggled = onToggled
M.setACCSpeed = setACCSpeed
M.changeACCSpeed = changeACCSpeed
M.changeACCFollowingDistance = changeACCFollowingDistance
M.update = update

return M