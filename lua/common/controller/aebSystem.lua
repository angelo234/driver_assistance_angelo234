local M = {}

--require("lua/common/luaProfiler")

local system_params = nil
local aeb_params = nil
local beeper_params = nil

local veh_name = nil

local velocity = vec3(obj:getVelocity())
local speed = velocity:length()

local system_active = false

local beeper_timer = 0

local function soundBeepers(dt, time_before_braking, vel_rel)
  beeper_timer = beeper_timer + dt

  --Sound warning tone if 1.0 * (0.5 + vel_rel / 40.0) seconds away from braking
  if time_before_braking <= 1.0 * (math.min(0.5 + vel_rel / 30.0, 1.0)) then
    --
    if beeper_timer >= 1.0 / beeper_params.fwd_warning_tone_hertz then
      obj:queueGameEngineLua("Engine.Audio.playOnce('AudioGui','art/sound/proximity_tone_50ms_loud.wav')")
      beeper_timer = 0
    end
  end
end

local release_brake_confidence_level = 0

local function performEmergencyBraking(dt, time_before_braking)
  --If throttle pedal is about half pressed then perform braking
  --But if throttle is highly requested then override braking
  if electrics.values.throttle > 0.4 then
    if system_active then
      input.event('brake', 0, 2)
      system_active = false 
    end
    return
  end
  
  --Stop car completely if below certain speed regardless of sensor information
  if system_active and speed < aeb_params.brake_till_stop_speed then
    input.event('brake', 1, 2)
    return
  end

  --debugDrawer:drawTextAdvanced((my_veh_props.front_pos + my_veh_props.dir * 2):toPoint3F(), String("Time till braking: " .. tostring(time_before_braking)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  --Maximum Braking
  if time_before_braking <= 0 then
    input.event('throttle', 0, 2)
    input.event('brake', 1, 2)
    system_active = true

    release_brake_confidence_level = 0
  else    
    release_brake_confidence_level = release_brake_confidence_level + 1

    --Only release brakes if confident
    if release_brake_confidence_level >= 15 then
      if system_active then
        input.event('brake', 0, 2)
        system_active = false
      end
    end
  end
end

local function calculateTimeBeforeBraking(distance, vel_rel)
  --Max braking acceleration = gravity * coefficient of static friction
  --local acc = math.min(-veh_accs_angelo234[veh_props.id][3], system_params.gravity) * params_per_veh[veh_props.name].fwd_friction_coeff

  local acc = math.min(10, system_params.gravity) * system_params.fwd_friction_coeff

  --Calculate TTC
  local ttc = distance / vel_rel
  local time_to_brake = vel_rel / (2 * acc)

  --leeway time depending on speed
  local time_before_braking = ttc - time_to_brake - aeb_params.braking_time_leeway
  
  return time_before_braking
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
  
  aeb_params = system_params.fwd_aeb_params
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
  velocity:set(obj:getVelocity())
  speed = velocity:length()

  local in_reverse = electrics.values.reverse
  local gear_selected = electrics.values.gear
  local esc_color = electrics.values.dseColor

  --ESC must be in comfort mode, otherwise it is deactivated
  --sunburst uses different color for comfort mode
  --if (veh_name == "sunburst" and esc_color == "98FB00")
   --or (veh_name ~= "sunburst" and esc_color == "238BE6")
  --then
  
  --Deactivate system based on any of these variables
  if in_reverse == nil or in_reverse == 1 or gear_selected == nil
    or gear_selected == 'P' or gear_selected == 0 then
    if system_active then
      input.event('brake', 0, 2)   
      system_active = false 
    end
    obj:queueGameEngineLua("ve_json_params_angelo234 = 'nil'")
    
    return
  end
      
  if speed <= aeb_params.min_speed then
    --When coming to a stop with system activated, release brakes but apply parking brake
    if system_active then
      --Release brake and apply parking brake
      input.event('brake', 0, 2)
      input.event('parkingbrake', 1, 2)
      system_active = false        
    end
    
    obj:queueGameEngineLua("ve_json_params_angelo234 = 'nil'")
    
    return   
  end

  local param_arr = 
  {
    obj:getID(), 
    aeb_params, 
    (speed / 20.0 + 0.1) --* aeb_params.min_distance_from_car
  }

  --Set params for AEB system
  obj:queueGameEngineLua("ve_json_params_angelo234 = '" .. jsonEncode(param_arr) .. "'")
  
  --Get AEB system data
  obj:queueGameEngineLua("be:getPlayerVehicle(0):queueLuaCommand('aeb_data_angelo234 = ' .. ge_aeb_data_angelo234)")

  if aeb_data_angelo234 == nil then return end
  
  local aeb_data = jsonDecode(aeb_data_angelo234)
  
  local distance = aeb_data[1]
  local vel_rel = aeb_data[2]

  local time_before_braking = calculateTimeBeforeBraking(distance, vel_rel)

  --At low speeds don't sound beepers
  if speed > 5 then
    soundBeepers(dt, time_before_braking, vel_rel)
  end
  
  --Use distance, relative velocity, and max acceleration to determine when to apply emergency braking
  performEmergencyBraking(dt, time_before_braking)
end

M.init = init
M.update = updateGFX

return M
