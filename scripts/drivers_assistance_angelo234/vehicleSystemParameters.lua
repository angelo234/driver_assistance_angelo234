local M = {}

M.parking_lines_params = {
  num_of_lines = 10,
  line_width = 0.05,
  line_length = 2.95,
  perp_line_length = 0.2,

  parking_line_offset_long = 0.2,
  parking_line_red_len = 0.2,
  parking_line_yellow_len = 0.6,
  parking_line_green_len = 1.0,
  parking_line_total_len = 0.2 + 0.6 + 1.0
}

M.params_per_veh = {
  ["etk800"] = {
    cam_fov = 120,
    cam_down_angle = 35,
    max_steer_radius = 4.3,
    min_steer_radius = 3.1,
    rel_cam_height = 0.25,
    cam_to_wheel_len = -1,
    veh_half_width = 0.6,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.6 + 0.05,
    parking_sensor_rel_height = -0.5,
    safety_offset_width_sensor = 0.25,
    systems = {"aeb", "parking_sensors", "reverse_cam", "cam_traj_lines", "cam_park_lines"}
  },
  ["etkc"] = {
    cam_fov = 120,
    cam_down_angle = 35,
    max_steer_radius = 4.5,
    min_steer_radius = 3.3,
    rel_cam_height = 0.25,
    cam_to_wheel_len = -1,
    veh_half_width = 0.6,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.6 + 0.05,
    parking_sensor_rel_height = -0.55,
    safety_offset_width_sensor = 0.35,
    systems = {"aeb", "parking_sensors", "reverse_cam", "cam_traj_lines", "cam_park_lines"}
  },
  ["sunburst"] = {
    cam_fov = 120,
    cam_down_angle = 35,
    max_steer_radius = 4.5,
    min_steer_radius = 3.3,
    rel_cam_height = 0.25,
    cam_to_wheel_len = -1,
    veh_half_width = 0.6,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.6 + 0.05,
    parking_sensor_rel_height = -0.55,
    safety_offset_width_sensor = 0.35,
    systems = {"aeb", "parking_sensors", "reverse_cam", "cam_traj_lines", "cam_park_lines"}
  },
  ["sbr"] = {
    cam_fov = 120,
    cam_down_angle = 35,
    max_steer_radius = 4.5,
    min_steer_radius = 3.3,
    rel_cam_height = 0.25,
    cam_to_wheel_len = -1,
    veh_half_width = 0.6,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.6 + 0.05,
    parking_sensor_rel_height = -0.55,
    safety_offset_width_sensor = 0.2,
    systems = {"aeb", "parking_sensors", "reverse_cam", "cam_traj_lines", "cam_park_lines"}
  },
  ["vivace"] = {
    cam_fov = 120,
    cam_down_angle = 35,
    max_steer_radius = 4.5,
    min_steer_radius = 3.3,
    rel_cam_height = 0.25,
    cam_to_wheel_len = -1,
    veh_half_width = 0.6,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.6 + 0.05,
    parking_sensor_rel_height = -0.55,
    safety_offset_width_sensor = 0.35,
    systems = {"aeb", "parking_sensors", "reverse_cam", "cam_traj_lines", "cam_park_lines"}
  },
  ["van"] = {
    cam_fov = 100,
    cam_down_angle = 45,
    max_steer_radius = 3.6,
    min_steer_radius = 2.4,
    rel_cam_height = 1.5,
    cam_to_wheel_len = -1,
    veh_half_width = 0.8,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.8 + 0.05,
    parking_sensor_rel_height = -1.4,
    safety_offset_width_sensor = 0.35,
    systems = {"reverse_cam"}
  },
  ["citybus"] = {
    cam_fov = 100,
    cam_down_angle = 50,
    max_steer_radius = 3.6,
    min_steer_radius = 2.4,
    rel_cam_height = 1.3,
    cam_to_wheel_len = -1,
    veh_half_width = 0.9,
    line_height_rel_cam = -0.3,
    veh_half_width_line_width = 0.9 + 0.05,
    parking_sensor_rel_height = -0.55,
    safety_offset_width_sensor = 0.35,
    systems = {"reverse_cam"}
  }
}

M.aeb_params = {
  gravity = 9.81,

  min_speed = 0.5, -- 3
  max_speed = 40,
  brake_till_stop_speed = 5,
  braking_time_leeway = 0.075,

  vehicle_search_radius = 100,
  min_distance_from_car = 0.5,

  parking_fwd_warning_tone_hertz = 7,
  parking_warning_tone_hertz = 20,
  parking_warning_tone_dist_per_hertz = 2.75,

  fwd_friction_coeff = 0.95,
  rev_friction_coeff = 0.65,

  lateral_acc_to_avoid_collision = 0.3,

  num_of_sensors = 3
}

return M
