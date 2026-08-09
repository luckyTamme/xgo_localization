-- Cartographer 2D for the XGO2 with an STL-19P LiDAR, driven by an external
-- odometry prior.
--
-- Inputs:  scan (LaserScan), odom (nav_msgs/Odometry from the localisation node)
-- Outputs: map -> odom on /tf, plus submaps and the occupancy grid.
--
-- Frame ownership is the important part. published_frame = "odom" together with
-- provide_odom_frame = false means cartographer emits only map -> odom and never
-- touches odom -> base_link, which the localisation node owns. That matches how
-- rtabmap behaves, so the two backends are drop-in alternatives.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
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
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_tracked_pose = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- Single-threaded pose-graph optimisation keeps runs deterministic. Multi-
-- threaded Ceres varies floating-point and constraint ordering, which flips
-- marginal loop closures between otherwise identical runs. There is no accuracy
-- cost; it only slows the background solve, which is negligible for a room-sized
-- 2D map. Raise it (default 4) only if a large live map needs faster optimising.
MAP_BUILDER.num_background_threads = 1

-- The platform's IMU reports orientation only; angular velocity and linear
-- acceleration are absent, and those are the two fields cartographer's 2D IMU
-- path actually consumes. The odometry prior already carries heading.
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Keep inserting scans while stationary. The motion filter otherwise waits for
-- the pose to move, so a standing robot contributes one scan every 5 s and the
-- map looks starved. Safe here because the odometry prior pins the pose at rest,
-- so each time-triggered insert lands at the correct frozen pose.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
