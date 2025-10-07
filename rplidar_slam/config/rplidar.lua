include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- RPLidar S2 optimized settings
TRAJECTORY_BUILDER_2D.min_range = 0.05  -- S2 has better close-range performance
TRAJECTORY_BUILDER_2D.max_range = 30.0  -- S2 extended range up to 30m
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0  -- Adjusted for longer range
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- Increased for larger environments
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(25.)  -- Increased angular search

-- Enhanced settings for S2's higher accuracy
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120  -- More data per submap for better accuracy
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  -- Finer motion filtering
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Loop closure optimization for larger environments
POSE_GRAPH.optimization_problem.huber_scale = 5e1  -- Adjusted for S2's accuracy
POSE_GRAPH.optimize_every_n_nodes = 40  -- More frequent optimization for better accuracy
POSE_GRAPH.constraint_builder.min_score = 0.62  -- Slightly relaxed for larger environments
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options 