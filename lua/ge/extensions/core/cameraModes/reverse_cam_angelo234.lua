-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local extra_utils = require('scripts/drivers_assistance_angelo234/extraUtils')
--File with all parameters for system
local system_params = require('scripts/drivers_assistance_angelo234/vehicleSystemParameters')

local parking_lines_params = system_params.parking_lines_params
local params_per_veh = system_params.params_per_veh
local aeb_params = system_params.aeb_params

local first_update = true
local veh_name = nil
local is_supported = false
local cam_traj_lines_on = false
local cam_park_lines_on = false

local C = {}
C.__index = C

function C:init()
	self.disabledByDefault = true
	self.register = true
	self:onVehicleCameraConfigChanged()
end

function C:onVehicleCameraConfigChanged()
end

function C:onCameraChanged(focused)
	if focused then
		be:executeJS('HookManager.trigger("onCameraNameChanged", ' .. jsonEncode({ name = "driver"}) .. ')')
	end
end

function C:reset()
	self.camRot = vec3()
end

--camDir is just negative dir of car
local function drawLeftTrajectoryLine(camPos, camDir, camLeft, camRight, camUp, line_start, line_end, line_start_width, line_end_width)
	local offset_height_vec = camUp * params_per_veh[veh_name].line_height_rel_cam --line offset height

	local vec_offset_center = camLeft * params_per_veh[veh_name].veh_half_width --line offset lat
		+ offset_height_vec + camDir * params_per_veh[veh_name].cam_to_wheel_len --line offset long

	local vec_offset_center_width = camLeft * params_per_veh[veh_name].veh_half_width_line_width --line offset lat + width
		+ offset_height_vec + camDir * params_per_veh[veh_name].cam_to_wheel_len --line offset long

	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center + line_start):toPoint3F(),
	(camPos + vec_offset_center_width + line_start_width):toPoint3F(),
	(camPos + vec_offset_center_width + line_end_width):toPoint3F(),
	(camPos + vec_offset_center + line_end):toPoint3F(),
	extra_utils.toColorI(ColorF(1,1,1,1)))
end

--camDir is just negative dir of car
local function drawRightTrajectoryLine(camPos, camDir, camLeft, camRight, camUp, line_start, line_end, line_start_width, line_end_width)
	local offset_height_vec = camUp * params_per_veh[veh_name].line_height_rel_cam --line offset height

	local vec_offset_center = camRight * params_per_veh[veh_name].veh_half_width --line offset lat
		+ offset_height_vec + camDir * params_per_veh[veh_name].cam_to_wheel_len --line offset long
	
	local vec_offset_center_width = camRight * params_per_veh[veh_name].veh_half_width_line_width --line offset lat + width
		+ offset_height_vec + camDir * params_per_veh[veh_name].cam_to_wheel_len --line offset long

	--debugDrawer:drawSphere((camPos + vec_offset_center + line_start):toPoint3F(), 0.05, ColorF(1,0,0,1))
	--debugDrawer:drawSphere((camPos + vec_offset_center_width + line_start_width):toPoint3F(), 0.05, ColorF(0,1,0,1))

	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_width + line_start_width):toPoint3F(),
	(camPos + vec_offset_center + line_start):toPoint3F(),	
	(camPos + vec_offset_center + line_end):toPoint3F(),
	(camPos + vec_offset_center_width + line_end_width):toPoint3F(),
	extra_utils.toColorI(ColorF(1,1,1,1)))
end

local function drawTrajectoryLines(camPos, dir, camLeft, camRight, camUp, left_turning_radius, right_turning_radius)
	
	local is_turning_left = left_turning_radius > 0
	
	--If negative turning radius, make is positive and then set angle to negative
	local left_start_angle = math.atan2(parking_lines_params.parking_line_offset_long - params_per_veh[veh_name].cam_to_wheel_len, math.abs(left_turning_radius))
	local right_start_angle = math.atan2(parking_lines_params.parking_line_offset_long - params_per_veh[veh_name].cam_to_wheel_len, math.abs(right_turning_radius))
	
	if is_turning_left == false then
		left_start_angle = -left_start_angle
		right_start_angle = -right_start_angle
	end
	
	local ind_line_length = parking_lines_params.parking_line_total_len / parking_lines_params.num_of_lines
	
	--In radians
	local ind_sector_angle_left = ind_line_length / left_turning_radius
	local ind_sector_angle_right = ind_line_length / right_turning_radius

	local the_line_width = parking_lines_params.line_width
	
	local offset_all_pos_left = camLeft * left_turning_radius
	local offset_all_pos_right = camLeft * right_turning_radius
	local offset_all_pos_width_large = nil
	local offset_all_pos_width_small = nil
	
	--Negative radius = turning back to left
	--Positive radius = turning back to right
	if left_turning_radius < 0 then
		the_line_width = -the_line_width
		
		offset_all_pos_width_large = camLeft * (left_turning_radius + the_line_width)
		offset_all_pos_width_small = camLeft * (right_turning_radius - the_line_width)
	else
		offset_all_pos_width_large = camLeft * (right_turning_radius + the_line_width)
		offset_all_pos_width_small = camLeft * (left_turning_radius - the_line_width)
	end
	
	local i = 0
	
	for i = 0, parking_lines_params.num_of_lines - 1 do
		local left_vec1 = math.cos(i * ind_sector_angle_left + left_start_angle) * camRight + math.sin(i * ind_sector_angle_left + left_start_angle) * dir
		local left_vec2 = math.cos((i + 1) * ind_sector_angle_left + left_start_angle) * camRight + math.sin((i + 1) * ind_sector_angle_left + left_start_angle) * dir
		
		local right_vec1 = math.cos(i * ind_sector_angle_right + right_start_angle) * camRight + math.sin(i * ind_sector_angle_right + right_start_angle) * dir
		local right_vec2 = math.cos((i + 1) * ind_sector_angle_right + right_start_angle) * camRight + math.sin((i + 1) * ind_sector_angle_right + right_start_angle) * dir

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

			drawLeftTrajectoryLine(camPos, dir, camLeft, camRight, camUp, offset_pos1, offset_pos2, offset_pos5, offset_pos6)
			drawRightTrajectoryLine(camPos, dir, camLeft, camRight, camUp, offset_pos3, offset_pos4, offset_pos7, offset_pos8)
		
		--Turning back to left
		else
			--For larger line (right)
			local offset_pos5 = right_vec1 * (right_turning_radius + the_line_width) + offset_all_pos_width_large
			local offset_pos6 = right_vec2 * (right_turning_radius + the_line_width) + offset_all_pos_width_large	

			--For smaller line (left)
			local offset_pos7 = left_vec1 * (left_turning_radius - the_line_width) + offset_all_pos_width_small
			local offset_pos8 = left_vec2 * (left_turning_radius - the_line_width) + offset_all_pos_width_small

			drawLeftTrajectoryLine(camPos, dir, camLeft, camRight, camUp, offset_pos1, offset_pos2, offset_pos7, offset_pos8)
			drawRightTrajectoryLine(camPos, dir, camLeft, camRight, camUp, offset_pos3, offset_pos4, offset_pos5, offset_pos6)
		end	
	end
end

local function drawParkingLines(camPos, camDir, camLeft, camRight, camUp)
	local offset_height_vec = camUp * params_per_veh[veh_name].line_height_rel_cam
  
	local vec_offset_center_left = camLeft * params_per_veh[veh_name].veh_half_width
		+ offset_height_vec + camDir * parking_lines_params.parking_line_offset_long

	local vec_offset_center_width_left = camLeft * params_per_veh[veh_name].veh_half_width_line_width
		+ offset_height_vec + camDir * parking_lines_params.parking_line_offset_long

	local vec_offset_center_right = camRight * params_per_veh[veh_name].veh_half_width
		+ offset_height_vec + camDir * parking_lines_params.parking_line_offset_long
	
	local vec_offset_center_width_right = camRight * params_per_veh[veh_name].veh_half_width_line_width
		+ offset_height_vec + camDir * parking_lines_params.parking_line_offset_long

  --Vectors to draw perpendicular/interval lines
	local vec_offset_center_left_perp = vec_offset_center_left + camDir * (parking_lines_params.parking_line_red_len - parking_lines_params.line_width)
	local vec_offset_center_left_width_perp = vec_offset_center_left + camDir * (parking_lines_params.parking_line_red_len)
	local vec_offset_center_left_inward_perp = vec_offset_center_left_perp + camRight * parking_lines_params.perp_line_length
	local vec_offset_center_left_width_inward_perp = vec_offset_center_left_width_perp + camRight * parking_lines_params.perp_line_length

	local vec_offset_center_right_perp = vec_offset_center_right + camDir * (parking_lines_params.parking_line_red_len - parking_lines_params.line_width)
	local vec_offset_center_right_width_perp = vec_offset_center_right + camDir * (parking_lines_params.parking_line_red_len)
	local vec_offset_center_right_inward_perp = vec_offset_center_right_perp + camLeft * parking_lines_params.perp_line_length
	local vec_offset_center_right_width_inward_perp = vec_offset_center_right_width_perp + camLeft * parking_lines_params.perp_line_length

	--This prevents a bug where the red line intersects the trajectory lines
	debugDrawer:drawLine(vec3():toPoint3F(), vec3():toPoint3F(), ColorF(0,0,0,0))

	--Left red lines
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_left):toPoint3F(),	
	(camPos + vec_offset_center_width_left):toPoint3F(),
	(camPos + vec_offset_center_width_left + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),
	(camPos + vec_offset_center_left + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),
	extra_utils.toColorI(ColorF(1,0,0,1)))

	--Right red lines
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_width_right):toPoint3F(),
	(camPos + vec_offset_center_right):toPoint3F(),	
	(camPos + vec_offset_center_right + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),
	(camPos + vec_offset_center_width_right + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),
	extra_utils.toColorI(ColorF(1,0,0,1)))
	
	--Perpendicular left red line
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_left_perp):toPoint3F(),	
	(camPos + vec_offset_center_left_width_perp):toPoint3F(),
	(camPos + vec_offset_center_left_width_inward_perp):toPoint3F(),
	(camPos + vec_offset_center_left_inward_perp):toPoint3F(),
	extra_utils.toColorI(ColorF(1,0,0,1)))
	
	--Perpendicular right red line
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_right_width_perp):toPoint3F(),
	(camPos + vec_offset_center_right_perp):toPoint3F(),	
	(camPos + vec_offset_center_right_inward_perp):toPoint3F(),
	(camPos + vec_offset_center_right_width_inward_perp):toPoint3F(),
	extra_utils.toColorI(ColorF(1,0,0,1)))
	
	--Left yellow lines
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_left + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),	
	(camPos + vec_offset_center_width_left + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),
	(camPos + vec_offset_center_width_left + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),
	(camPos + vec_offset_center_left + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),
	extra_utils.toColorI(ColorF(1,1,0,1)))
	
	--Right yellow lines
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_width_right + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),
	(camPos + vec_offset_center_right + camDir * parking_lines_params.parking_line_red_len):toPoint3F(),	
	(camPos + vec_offset_center_right + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),
	(camPos + vec_offset_center_width_right + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),
	extra_utils.toColorI(ColorF(1,1,0,1)))
	
	--Perpendicular left yellow line
	local yellow_offset_perp_vec = camDir * parking_lines_params.parking_line_yellow_len
	
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_left_perp + yellow_offset_perp_vec):toPoint3F(),	
	(camPos + vec_offset_center_left_width_perp + yellow_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_left_width_inward_perp + yellow_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_left_inward_perp + yellow_offset_perp_vec):toPoint3F(),
	extra_utils.toColorI(ColorF(1,1,0,1)))
	
	--Perpendicular right yellow line
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_right_width_perp + yellow_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_right_perp + yellow_offset_perp_vec):toPoint3F(),	
	(camPos + vec_offset_center_right_inward_perp + yellow_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_right_width_inward_perp + yellow_offset_perp_vec):toPoint3F(),
	extra_utils.toColorI(ColorF(1,1,0,1)))
	
	
	--Left green lines
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_left + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),	
	(camPos + vec_offset_center_width_left + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),
	(camPos + vec_offset_center_width_left + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)):toPoint3F(),
	(camPos + vec_offset_center_left + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)):toPoint3F(),
	extra_utils.toColorI(ColorF(0,1,0,1)))
	
	--Right green lines
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_width_right + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),
	(camPos + vec_offset_center_right + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len)):toPoint3F(),	
	(camPos + vec_offset_center_right + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)):toPoint3F(),
	(camPos + vec_offset_center_width_right + camDir * (parking_lines_params.parking_line_red_len + parking_lines_params.parking_line_yellow_len + parking_lines_params.parking_line_green_len)):toPoint3F(),
	extra_utils.toColorI(ColorF(0,1,0,1)))
	
	--Perpendicular left green line
	local green_offset_perp_vec = camDir * (parking_lines_params.parking_line_green_len + parking_lines_params.parking_line_yellow_len)
	
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_left_perp + green_offset_perp_vec):toPoint3F(),	
	(camPos + vec_offset_center_left_width_perp + green_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_left_width_inward_perp + green_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_left_inward_perp + green_offset_perp_vec):toPoint3F(),
	extra_utils.toColorI(ColorF(0,1,0,1)))
	
	--Perpendicular right green line
	debugDrawer:drawQuadSolid(
	(camPos + vec_offset_center_right_width_perp + green_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_right_perp + green_offset_perp_vec):toPoint3F(),	
	(camPos + vec_offset_center_right_inward_perp + green_offset_perp_vec):toPoint3F(),
	(camPos + vec_offset_center_right_width_inward_perp + green_offset_perp_vec):toPoint3F(),
	extra_utils.toColorI(ColorF(0,1,0,1)))
end

local function calculateTurningRadii(veh)
	local rear_left_turning_radius = 0
	local rear_right_turning_radius = 0
	
	veh:queueLuaCommand("obj:queueGameEngineLua('steering_input = ' .. electrics.values.steering_input )")

	if steering_input == nil then return {0,0} end

	if steering_input == 0 then
		rear_left_turning_radius = 999
		rear_right_turning_radius = 999
		
	else
		if steering_input > 0 then
			rear_left_turning_radius = params_per_veh[veh_name].min_steer_radius / steering_input
			rear_right_turning_radius = params_per_veh[veh_name].max_steer_radius / steering_input 	
		else
			rear_left_turning_radius = params_per_veh[veh_name].max_steer_radius / steering_input
			rear_right_turning_radius = params_per_veh[veh_name].min_steer_radius / steering_input 
		end	
	end

	return {-rear_left_turning_radius, -rear_right_turning_radius}
end

local function checkVehicleSupported(id)
	is_supported = false
	cam_traj_lines_on = false
	cam_park_lines_on = false
	
	local the_veh_name = be:getObjectByID(id):getJBeamFilename()
	
	for curr_veh_name, params in pairs(params_per_veh) do
		if the_veh_name == curr_veh_name then
			for _, system in pairs(params.systems) do
				if system == "reverse_cam" then
					is_supported = true
					veh_name = the_veh_name				
				elseif system == "cam_traj_lines" then
					cam_traj_lines_on = true
				
				elseif system == "cam_park_lines" then
					cam_park_lines_on = true
				end
			end
		
			if is_supported then 
			  --check if Driving and Safety Electronics (DSE) part is the regular one (not race one)
        local dse_part_selected = extensions.core_vehicle_manager.getVehicleData(id)
        .chosenParts[params_per_veh[the_veh_name].dse_part_name]
  
        is_supported = dse_part_selected == params_per_veh[the_veh_name].dse_part_name			
			end
		end
	end
end

function C:onVehicleSwitched(oldID, newID, player)
	checkVehicleSupported(newID)
end

function C:onVehicleSpawned(vid)
  checkVehicleSupported(vid)
end

function C:update(data)
  --Check if vehicle supported on first update
  if first_update then
    checkVehicleSupported(data.veh:getID())
    first_update = false
  end
  
  if electrics_values_angelo234 == nil then return end

  local in_reverse = electrics_values_angelo234["reverse"]

	--If vehicle doesn't have a backup camera or not in reverse, set camera to previous camera mode
	if in_reverse == nil or is_supported == false or in_reverse == 0 then
	  local prev_cam = scripts_drivers__assistance__angelo234_extension.prev_camera_mode
		core_camera.setByName(0, prev_cam)
		return
	end
	
	local carPos = data.pos
	local ref  = vec3(data.veh:getNodePosition(self.refNodes.ref))
	local left = vec3(data.veh:getNodePosition(self.refNodes.left))
	local back = vec3(data.veh:getNodePosition(self.refNodes.back))
	local dir = (ref - back):normalized()
	local camLeft = (ref - left):normalized()
	local camUp = -(dir:cross(camLeft):normalized())

	local qdir = quatFromDir(dir)
	local rotatedUp = qdir * vec3(0, 0, 1)

	qdir = extra_utils.rotateEuler(math.rad(180),math.rad(params_per_veh[veh_name].cam_down_angle),math.atan2(rotatedUp:dot(camLeft), rotatedUp:dot(camUp)),qdir)
	local camPos = vec3(data.veh:getSpawnWorldOOBBRearPoint()) + camUp * params_per_veh[veh_name].rel_cam_height

	--Get turning radius of both rear wheels (they are different)
	local radii = calculateTurningRadii(data.veh)

	local left_turning_radius = radii[1]
	local right_turning_radius = radii[2]

	--debugDrawer:drawSphere((vec3(data.veh:getSpawnWorldOOBBRearPoint()) + camUp * params_per_veh[veh_name].rel_cam_height):toPoint3F(), 0.05, ColorF(1,0,0,1))
  debugDrawer:setSolidTriCulling(true)

	--Draw parking lines
	if cam_park_lines_on then
		drawParkingLines(vec3(data.veh:getSpawnWorldOOBBRearPoint()), -dir, camLeft, -camLeft, camUp)
	end
	
	--Draw trajectory lines
	if cam_traj_lines_on then
		drawTrajectoryLines(vec3(data.veh:getSpawnWorldOOBBRearPoint()), -dir, camLeft, -camLeft, camUp, -left_turning_radius, -right_turning_radius)
	end

	-- application
	data.res.pos = camPos
	data.res.rot = qdir
	data.res.fov = params_per_veh[veh_name].cam_fov

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
