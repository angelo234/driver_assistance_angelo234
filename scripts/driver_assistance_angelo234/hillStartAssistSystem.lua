local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local hold_angle = 4
local brake_pedal_threshold = 0.1

local activated = false
local off_brake_timer = 0

local function checkToActivateSystem(veh, gear_selected, yaw)
  if input_brake_angelo234 >= brake_pedal_threshold
  and vec3(veh:getVelocity()):length() < 0.05 then
    if ((yaw > hold_angle and (gear_selected == "D" or gear_selected:find('S') or gear_selected == "1"))
    or yaw < -hold_angle and (gear_selected == "R" or gear_selected == "-1"))
    and input_throttle_angelo234 == 0
    and input_parkingbrake_angelo234 == 0 then
      veh:queueLuaCommand("electrics.values.brakeOverride = 1")
      off_brake_timer = 0
      activated = true

      ui_message("Hill Start Assist Activated", 3)
    end
  end
end

local function checkToDeactivateSystem(dt, veh, gear_selected, yaw)
  local deactivate = false

  if (yaw > hold_angle and (gear_selected ~= "D" and (not gear_selected:find('S')) and gear_selected ~= "1"))
  or yaw < -hold_angle and (gear_selected ~= "R" and gear_selected ~= "-1")
  or input_throttle_angelo234 > 0
  or input_parkingbrake_angelo234 > 0 then
    deactivate = true
  end

  --Start timer when player gets off brakes
  if input_brake_angelo234 == 0 then
    if off_brake_timer >= 3 then
      deactivate = true
    end

    off_brake_timer = off_brake_timer + dt
  else
    off_brake_timer = 0
  end

  if deactivate then
    veh:queueLuaCommand("electrics.values.brakeOverride = nil")
    activated = false
    ui_message("Hill Start Assist Deactivated", 3)
  end

end

local function update(dt, veh)
  local gear_selected = tostring(electrics_values_angelo234["gear"])
  local yaw = math.asin(rotation_angelo234.y) * (180 / math.pi)

  if not activated then
    checkToActivateSystem(veh, gear_selected, yaw)
  else
    checkToDeactivateSystem(dt, veh, gear_selected, yaw)
  end
end

M.update = update

return M