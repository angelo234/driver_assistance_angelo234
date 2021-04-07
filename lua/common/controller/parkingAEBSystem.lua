local M = {}

local system_params = nil
local parking_lines_params = nil
local rev_aeb_params = nil
local beeper_params = nil

local veh_name = nil

local speed = vec3(obj:getVelocity()):length()

local system_active = false

local beeper_timer = 0

local function soundBeepers(dt, dist)
  beeper_timer = beeper_timer + dt

  dist = dist + 0.2

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
  if v.config.model then
    veh_name = v.config.model
  else
    veh_name = v.config.partConfigFilename:match("/(%S+)/")
  end 

  local default_param_file_dir = 'vehicles/common/parameters'
  local param_file_dir = 'vehicles/' .. veh_name .. '/parameters'
  
  if FS:fileExists(param_file_dir .. ".lua") then
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
  --Don't update if player not in vehicle
  
  if mapmgr.objects[obj:getID()] then
    if not mapmgr.objects[obj:getID()].active then
      return
    end
  end
  
  --update stuff
  speed = vec3(obj:getVelocity()):length()

  local in_reverse = electrics.values.reverse
  local gear_selected = electrics.values.gear

  if in_reverse == nil or gear_selected == nil or in_reverse == 0 then 
    return 
  end
  
  local param_arr = 
  {
    obj:getID(), 
    rev_aeb_params
  }
   
  --Set params for AEB system
  obj:queueGameEngineLua("ve_rev_json_params_angelo234 = '" .. jsonEncode(param_arr) .. "'")
  obj:queueGameEngineLua("be:getPlayerVehicle(0):queueLuaCommand('rev_aeb_data_angelo234 = ' .. ge_rev_aeb_data_angelo234)")

  if rev_aeb_data_angelo234 == nil then return end
  
  local aeb_data = jsonDecode(rev_aeb_data_angelo234)
  
  local distance = aeb_data[1]
  
  --Play beeping sound based on min distance of prev sensor detections to obstacle
  soundBeepers(dt, distance)
  
  --[[

  for i = 1, rev_aeb_params.sensors_polled_per_iteration do
    if static_sensor_id == rev_aeb_params.num_of_sensors - 1 then
      prev_min_dist = min_dist
      min_dist = 9999
    end
  
    --Get distance and other data of nearest obstacle 
    local other_veh, dist = pollReverseSensors(dt)
     
    min_dist = math.min(dist, min_dist)
  end
  ]]--

  


  --if not aeb_enabled then return end 

  --print(min_dist)
  
  performEmergencyBraking(dt, distance)
end

M.init = init
M.updateGFX = updateGFX

return M
