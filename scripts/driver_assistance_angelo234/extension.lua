-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.curr_camera_mode = "orbit"
M.prev_camera_mode = "orbit"

--Used for what camera to switch the player to when the player gets out of reverse gear using reverse camera
local function onCameraModeChanged(new_camera_mode)
  if new_camera_mode ~= M.curr_camera_mode then
    M.prev_camera_mode = M.curr_camera_mode
    M.curr_camera_mode = new_camera_mode
  end
end

--Complicated way to load in camera (but it works quite seamlessly)
local function doLuaReload()
  --Determine if Lua loaded for first time or was loaded another time
  --if loaded for first time, then reload Lua to load in the reverse camera properly
  --if not, then don't need to do anything
  --And if log header changes from "Log started" to "Log rotated" then don't bother reloading
  --since by that time, the reverse camera probably was already loaded in
  
  local log_file_header_file = "prev_log_file_header_angelo234.txt"
  local curr_log_file_header = readFile("beamng.log"):match("^.-\r")
  local curr_log_status = curr_log_file_header:match("Log.-ed")
  
  --If log header doesn't state "Log started" then don't do anything
  if curr_log_status ~= "Log started" then
    return
  end
  
  if not FS:fileExists(log_file_header_file) then
    writeFile(log_file_header_file, curr_log_file_header)
    Lua:requestReload()
  else
    local prev_log_file_header = readFile(log_file_header_file)
    
    --Means different game instance so reload lua again
    if prev_log_file_header ~= curr_log_file_header then
      writeFile(log_file_header_file, curr_log_file_header)
      Lua:requestReload()    
    end 
  end
end

local first_update = true

local function onUpdate(dt)
  --Do Lua reload after first Lua initialization to load in the reverse camera
  if first_update then
    doLuaReload()   
    first_update = false
  end
  
  local my_veh = be:getPlayerVehicle(0)
  my_veh:queueLuaCommand('obj:queueGameEngineLua("electrics_values_angelo234 = (\'" .. jsonEncode(electrics.values) .. "\')")')
  
  if electrics_values_angelo234 == nil or #electrics_values_angelo234 == 0 then return end
  
  electrics_values_angelo234 = jsonDecode(electrics_values_angelo234)
end

M.onCameraModeChanged = onCameraModeChanged
M.onUpdate = onUpdate

return M
