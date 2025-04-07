-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

--File with all parameters for system
local system_params = nil
local parking_lines_params = nil
local rev_cam_params = nil

local C = {}
C.__index = C

local car_pos = nil
local cam_pos = nil
local cam_dir = nil
local cam_dir_left = nil
local cam_dir_right = nil
local cam_dir_up = nil

local veh_half_width = 0
local line_height_rel_cam = -1

local function rotateEuler(x, y, z, q)
  q = q or quat()
  q = quatFromEuler(0, z, 0) * q
  q = quatFromEuler(0, 0, x) * q
  q = quatFromEuler(y, 0, 0) * q
  return q
end

local function initProperties()
  local veh = be:getPlayerVehicle(0)
  if veh == nil then return end

  local veh_name = veh:getJBeamFilename()

  local default_param_file_dir = 'vehicles/common/parameters'
  local param_file_dir = 'vehicles/' .. veh_name .. '/parameters'

  if FS:fileExists(param_file_dir .. ".lua") then
    --load parameter lua file dependent on vehicle
    system_params = require(param_file_dir)
  else
    --use default parameters if they don't exist for current vehicle
    system_params = require(default_param_file_dir)
  end

  rev_cam_params = system_params.rev_cam_params
  parking_lines_params = rev_cam_params.parking_lines_params
end

function C:init()
  self.disabledByDefault = true
  self.register = true
  self:onVehicleCameraConfigChanged()

  initProperties()
end

function C:onVehicleCameraConfigChanged()
end

function C:onCameraChanged(focused)
  if focused then
    initProperties()
  end
end

function C:reset()
  self.camRot = vec3()
end

--cam_dir is just negative dir of car
local function drawLeftTrajectoryLine(line_start, line_end, line_start_width, line_end_width)
  local offset_height_vec = cam_dir_up * line_height_rel_cam --line offset height

  local vec_offset_center = cam_dir_left * veh_half_width --line offset lat
    + offset_height_vec + cam_dir * rev_cam_params.cam_to_wheel_len --line offset long

  local vec_offset_center_width = cam_dir_left * (veh_half_width + parking_lines_params.line_width)  --line offset lat + width
    + offset_height_vec + cam_dir * rev_cam_params.cam_to_wheel_len --line offset long

  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center + line_start),
    (cam_pos + vec_offset_center_width + line_start_width),
    (cam_pos + vec_offset_center_width + line_end_width),
    (cam_pos + vec_offset_center + line_end),
    ColorF(1,1,1,1),
    false)
end

--cam_dir is just negative dir of car
local function drawRightTrajectoryLine(line_start, line_end, line_start_width, line_end_width)
  local offset_height_vec = cam_dir_up * line_height_rel_cam --line offset height

  local vec_offset_center = cam_dir_right * veh_half_width --line offset lat
    + offset_height_vec + cam_dir * rev_cam_params.cam_to_wheel_len --line offset long

  local vec_offset_center_width = cam_dir_right * (veh_half_width + parking_lines_params.line_width)  --line offset lat + width
    + offset_height_vec + cam_dir * rev_cam_params.cam_to_wheel_len --line offset long

    --debugDrawer:drawSphere((cam_pos + vec_offset_center + line_start), 0.05, ColorF(1,0,0,1))
    --debugDrawer:drawSphere((cam_pos + vec_offset_center_width + line_start_width), 0.05, ColorF(0,1,0,1))

  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_width + line_start_width),
    (cam_pos + vec_offset_center + line_start),
    (cam_pos + vec_offset_center + line_end),
    (cam_pos + vec_offset_center_width + line_end_width),
    ColorF(1,1,1,1),
    false)
end

local function drawTrajectoryLines(left_turning_radius, right_turning_radius)

  local is_turning_left = left_turning_radius > 0

  --If negative turning radius, make is positive and then set angle to negative
  local left_start_angle = math.atan2(parking_lines_params.parking_line_offset_long - rev_cam_params.cam_to_wheel_len, math.abs(left_turning_radius))
  local right_start_angle = math.atan2(parking_lines_params.parking_line_offset_long - rev_cam_params.cam_to_wheel_len, math.abs(right_turning_radius))

  if is_turning_left == false then
    left_start_angle = -left_start_angle
    right_start_angle = -right_start_angle
  end

  local ind_line_length = parking_lines_params.parking_line_total_len / parking_lines_params.num_of_lines

  --In radians
  local ind_sector_angle_left = ind_line_length / left_turning_radius
  local ind_sector_angle_right = ind_line_length / right_turning_radius

  local the_line_width = parking_lines_params.line_width

  local offset_all_pos_left = cam_dir_left * left_turning_radius
  local offset_all_pos_right = cam_dir_left * right_turning_radius
  local offset_all_pos_width_large = nil
  local offset_all_pos_width_small = nil

  --Negative radius = turning back to left
  --Positive radius = turning back to right
  if left_turning_radius < 0 then
    the_line_width = -the_line_width

    offset_all_pos_width_large = cam_dir_left * (left_turning_radius + the_line_width)
    offset_all_pos_width_small = cam_dir_left * (right_turning_radius - the_line_width)
  else
    offset_all_pos_width_large = cam_dir_left * (right_turning_radius + the_line_width)
    offset_all_pos_width_small = cam_dir_left * (left_turning_radius - the_line_width)
  end

  local i = 0

  for i = 0, parking_lines_params.num_of_lines - 1 do
    local left_vec1 = math.cos(i * ind_sector_angle_left + left_start_angle) * cam_dir_right + math.sin(i * ind_sector_angle_left + left_start_angle) * cam_dir
    local left_vec2 = math.cos((i + 1) * ind_sector_angle_left + left_start_angle) * cam_dir_right + math.sin((i + 1) * ind_sector_angle_left + left_start_angle) * cam_dir

    local right_vec1 = math.cos(i * ind_sector_angle_right + right_start_angle) * cam_dir_right + math.sin(i * ind_sector_angle_right + right_start_angle) * cam_dir
    local right_vec2 = math.cos((i + 1) * ind_sector_angle_right + right_start_angle) * cam_dir_right + math.sin((i + 1) * ind_sector_angle_right + right_start_angle) * cam_dir

    --Left line
    local offset_pos1 = left_vec1 * left_turning_radius + offset_all_pos_left
    local offset_pos2 = left_vec2 * left_turning_radius + offset_all_pos_left

    --Right line
    local offset_pos3 = right_vec1 * right_turning_radius + offset_all_pos_right
    local offset_pos4 = right_vec2 * right_turning_radius + offset_all_pos_right

    --Turning back to the right
    if left_turning_radius < 0 then
      --For larger line (left)
      local offset_pos5 = left_vec1 * (left_turning_radius + the_line_width) + offset_all_pos_width_large
      local offset_pos6 = left_vec2 * (left_turning_radius + the_line_width) + offset_all_pos_width_large

      --For smaller line (right)
      local offset_pos7 = right_vec1 * (right_turning_radius - the_line_width) + offset_all_pos_width_small
      local offset_pos8 = right_vec2 * (right_turning_radius - the_line_width) + offset_all_pos_width_small

      drawLeftTrajectoryLine(offset_pos1, offset_pos2, offset_pos5, offset_pos6)
      drawRightTrajectoryLine(offset_pos3, offset_pos4, offset_pos7, offset_pos8)

      --Turning back to left
    else
      --For larger line (right)
      local offset_pos5 = right_vec1 * (right_turning_radius + the_line_width) + offset_all_pos_width_large
      local offset_pos6 = right_vec2 * (right_turning_radius + the_line_width) + offset_all_pos_width_large

      --For smaller line (left)
      local offset_pos7 = left_vec1 * (left_turning_radius - the_line_width) + offset_all_pos_width_small
      local offset_pos8 = left_vec2 * (left_turning_radius - the_line_width) + offset_all_pos_width_small

      drawLeftTrajectoryLine(offset_pos1, offset_pos2, offset_pos7, offset_pos8)
      drawRightTrajectoryLine(offset_pos3, offset_pos4, offset_pos5, offset_pos6)
    end
  end
end

local function drawParkingLines()

  local offset_height_vec = cam_dir_up * line_height_rel_cam


  local vec_offset_center_left = cam_dir_left * veh_half_width
    + offset_height_vec + cam_dir * parking_lines_params.parking_line_offset_long

  local vec_offset_center_width_left = cam_dir_left * (veh_half_width + parking_lines_params.line_width)
    + offset_height_vec + cam_dir * parking_lines_params.parking_line_offset_long

  local vec_offset_center_right = cam_dir_right * veh_half_width
    + offset_height_vec + cam_dir * parking_lines_params.parking_line_offset_long

  local vec_offset_center_width_right = cam_dir_right * (veh_half_width + parking_lines_params.line_width)
    + offset_height_vec + cam_dir * parking_lines_params.parking_line_offset_long

  --Vectors to draw perpendicular/interval lines
  local vec_offset_center_left_perp = vec_offset_center_left + cam_dir * (parking_lines_params.parking_line_red_len - parking_lines_params.line_width)
  local vec_offset_center_left_width_perp = vec_offset_center_left + cam_dir * (parking_lines_params.parking_line_red_len)
  local vec_offset_center_left_inward_perp = vec_offset_center_left_perp + cam_dir_right * parking_lines_params.perp_line_length
  local vec_offset_center_left_width_inward_perp = vec_offset_center_left_width_perp + cam_dir_right * parking_lines_params.perp_line_length

  local vec_offset_center_right_perp = vec_offset_center_right + cam_dir * (parking_lines_params.parking_line_red_len - parking_lines_params.line_width)
  local vec_offset_center_right_width_perp = vec_offset_center_right + cam_dir * (parking_lines_params.parking_line_red_len)
  local vec_offset_center_right_inward_perp = vec_offset_center_right_perp + cam_dir_left * parking_lines_params.perp_line_length
  local vec_offset_center_right_width_inward_perp = vec_offset_center_right_width_perp + cam_dir_left * parking_lines_params.perp_line_length

  --This prevents a bug where the red line intersects the trajectory lines
  debugDrawer:drawLine(vec3(), vec3(), ColorF(0,0,0,0))

  --Left red lines
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_left),
    (cam_pos + vec_offset_center_width_left),
    (cam_pos + vec_offset_center_width_left + cam_dir * parking_lines_params.parking_line_red_len),
    (cam_pos + vec_offset_center_left + cam_dir * parking_lines_params.parking_line_red_len),
    ColorF(1,0,0,1),
    false)

  --Right red lines
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_width_right),
    (cam_pos + vec_offset_center_right),
    (cam_pos + vec_offset_center_right + cam_dir * parking_lines_params.parking_line_red_len),
    (cam_pos + vec_offset_center_width_right + cam_dir * parking_lines_params.parking_line_red_len),
    ColorF(1,0,0,1),
    false)

  --Perpendicular left red line
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_left_perp),
    (cam_pos + vec_offset_center_left_width_perp),
    (cam_pos + vec_offset_center_left_width_inward_perp),
    (cam_pos + vec_offset_center_left_inward_perp),
    ColorF(1,0,0,1),
    false)

  --Perpendicular right red line
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_right_width_perp),
    (cam_pos + vec_offset_center_right_perp),
    (cam_pos + vec_offset_center_right_inward_perp),
    (cam_pos + vec_offset_center_right_width_inward_perp),
    ColorF(1,0,0,1),
    false)

  --Left yellow lines
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_left + cam_dir * parking_lines_params.parking_line_red_len),
    (cam_pos + vec_offset_center_width_left + cam_dir * parking_lines_params.parking_line_red_len),
    (cam_pos + vec_offset_center_width_left + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    (cam_pos + vec_offset_center_left + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    ColorF(1,1,0,1),
    false)

  --Right yellow lines
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_width_right + cam_dir * parking_lines_params.parking_line_red_len),
    (cam_pos + vec_offset_center_right + cam_dir * parking_lines_params.parking_line_red_len),
    (cam_pos + vec_offset_center_right + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    (cam_pos + vec_offset_center_width_right + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    ColorF(1,1,0,1),
    false)

  --Perpendicular left yellow line
  local yellow_offset_perp_vec = cam_dir * parking_lines_params.parking_line_yellow_len

  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_left_perp + yellow_offset_perp_vec),
    (cam_pos + vec_offset_center_left_width_perp + yellow_offset_perp_vec),
    (cam_pos + vec_offset_center_left_width_inward_perp + yellow_offset_perp_vec),
    (cam_pos + vec_offset_center_left_inward_perp + yellow_offset_perp_vec),
    ColorF(1,1,0,1),
    false)

  --Perpendicular right yellow line
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_right_width_perp + yellow_offset_perp_vec),
    (cam_pos + vec_offset_center_right_perp + yellow_offset_perp_vec),
    (cam_pos + vec_offset_center_right_inward_perp + yellow_offset_perp_vec),
    (cam_pos + vec_offset_center_right_width_inward_perp + yellow_offset_perp_vec),
    ColorF(1,1,0,1),
    false)


  --Left green lines
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_left + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    (cam_pos + vec_offset_center_width_left + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    (cam_pos + vec_offset_center_width_left + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)),
    (cam_pos + vec_offset_center_left + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)),
    ColorF(0,1,0,1),
    false)

  --Right green lines
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_width_right + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    (cam_pos + vec_offset_center_right + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)),
    (cam_pos + vec_offset_center_right + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)),
    (cam_pos + vec_offset_center_width_right + cam_dir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)),
    ColorF(0,1,0,1),
    false)

  --Perpendicular left green line
  local green_offset_perp_vec = cam_dir * (parking_lines_params.parking_line_green_len + parking_lines_params.parking_line_yellow_len)

  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_left_perp + green_offset_perp_vec),
    (cam_pos + vec_offset_center_left_width_perp + green_offset_perp_vec),
    (cam_pos + vec_offset_center_left_width_inward_perp + green_offset_perp_vec),
    (cam_pos + vec_offset_center_left_inward_perp + green_offset_perp_vec),
    ColorF(0,1,0,1),
    false)

  --Perpendicular right green line
  debugDrawer:drawQuadSolid(
    (cam_pos + vec_offset_center_right_width_perp + green_offset_perp_vec),
    (cam_pos + vec_offset_center_right_perp + green_offset_perp_vec),
    (cam_pos + vec_offset_center_right_inward_perp + green_offset_perp_vec),
    (cam_pos + vec_offset_center_right_width_inward_perp + green_offset_perp_vec),
    ColorF(0,1,0,1),
    false)
end

local function calculateTurningRadii(veh)
  local rear_left_turning_radius = 0
  local rear_right_turning_radius = 0

  local steering_input = electrics_values_angelo234["steering_input"]

  if steering_input == nil then return 0,0 end

  if steering_input == 0 then
    rear_left_turning_radius = 999
    rear_right_turning_radius = 999

  else
    if steering_input > 0 then
      rear_left_turning_radius = system_params.min_steer_radius / steering_input
      rear_right_turning_radius = system_params.max_steer_radius / steering_input
    else
      rear_left_turning_radius = system_params.max_steer_radius / steering_input
      rear_right_turning_radius = system_params.min_steer_radius / steering_input
    end
  end

  return -rear_left_turning_radius, -rear_right_turning_radius
end

local function checkVehicleSupported(id)
  local backup_cam_part = extra_utils.getPart("backup_cam_angelo234")

  --Does backup cam part exists?
  if backup_cam_part then
    local cam_traj_lines_on = extra_utils.getPart("trajectory_lines_angelo234") ~= nil
    local cam_park_lines_on = extra_utils.getPart("parking_lines_angelo234") ~= nil

    return true, cam_traj_lines_on, cam_park_lines_on
  end

  return false, false, false
end

function C:update(data)
  --Check if vehicle supported
  local is_supported, cam_traj_lines_on, cam_park_lines_on = checkVehicleSupported(data.veh:getID())

  if electrics_values_angelo234 == nil then return end

  local gear_selected = electrics_values_angelo234["gear"]
  local in_reverse = electrics_values_angelo234["reverse"]

  if gear_selected == nil or in_reverse == nil then return end

  --If vehicle doesn't have a backup camera or not in reverse and neutral, set camera to previous camera mode
  if not is_supported
    or (gear_selected ~= 'N' and gear_selected ~= 'R' and gear_selected ~= 0 and gear_selected ~= -1) then
    local prev_cam = scripts_driver__assistance__angelo234_extension.prev_camera_mode
    core_camera.setByName(0, prev_cam)
    return
  end

  car_pos = data.pos
  local ref  = vec3(data.veh:getNodePosition(self.refNodes.ref))
  local left = vec3(data.veh:getNodePosition(self.refNodes.left))
  local back = vec3(data.veh:getNodePosition(self.refNodes.back))
  cam_dir = (back - ref):normalized()
  cam_dir_left = (ref - left):normalized()
  cam_dir_right = -cam_dir_left
  cam_dir_up = cam_dir:cross(cam_dir_left):normalized()

  local qdir = quatFromDir(cam_dir)
  local rotated_up = qdir * vec3(0, 0, 1)

  qdir = rotateEuler(0, math.rad(rev_cam_params.cam_down_angle), math.atan2(rotated_up:dot(cam_dir_right), rotated_up:dot(cam_dir_up)), qdir)

  cam_pos = vec3(data.veh:getSpawnWorldOOBBRearPoint()) + cam_dir_up * rev_cam_params.rel_cam_height

  local bb = data.veh:getSpawnWorldOOBB()

  local cam_pos_to_center_vec = vec3(bb:getCenter()) - cam_pos

  line_height_rel_cam = -(bb:getHalfExtents().z - cam_pos_to_center_vec:dot(cam_dir_up));

  --Get turning radius of both rear wheels (they are different)
  local left_turning_radius, right_turning_radius = calculateTurningRadii(data.veh)

  --accounts for the line raised from ground, by setting lines out not as much
  veh_half_width = data.veh:getSpawnWorldOOBB():getHalfExtents().x

  --debugDrawer:drawSphere((vec3(data.veh:getSpawnWorldOOBBRearPoint()) + cam_dir_up * rev_cam_params.rel_cam_height), 0.05, ColorF(1,0,0,1))
  --debugDrawer:setSolidTriCulling(false)

  --Draw trajectory lines
  if cam_traj_lines_on then
    drawTrajectoryLines(-left_turning_radius, -right_turning_radius)
  end

  --Draw parking lines
  if cam_park_lines_on then
    drawParkingLines()
  end

  -- application
  data.res.pos = cam_pos
  data.res.rot = qdir
  data.res.fov = rev_cam_params.cam_fov

  return true
end

function C:setRefNodes(centerNodeID, leftNodeID, backNodeID)
  self.refNodes = self.refNodes or {}
  self.refNodes.ref = centerNodeID
  self.refNodes.left = leftNodeID
  self.refNodes.back = backNodeID
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end
