-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "bumpy7/base_footprint",
  published_frame = "bumpy7/base_footprint",
  odom_frame = "bumpy7/odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 50e-3,
  trajectory_publish_period_sec = 50e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ========================================
-- NOISE REDUCTION & RANGE SETTINGS
-- ========================================

-- Filter out close and far noisy points
TRAJECTORY_BUILDER_2D.min_range = 0.15  -- Ignore points closer than 15cm (reduces floor/ceiling noise)
TRAJECTORY_BUILDER_2D.max_range = 6.0   -- Ignore points beyond 6m (reduces far-field noise)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.5

TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ========================================
-- VOXEL FILTERING (Primary Noise Reduction)
-- ========================================

-- Aggressive voxel filtering - groups nearby points into single voxels
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.08  -- 8cm voxels (increase to 0.10 for more filtering)

-- Adaptive voxel filter - removes sparse/isolated points
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter = {
  max_length = 0.9,           -- Maximum voxel size
  min_num_points = 150,       -- Minimum points in voxel to keep it (higher = more filtering)
  max_range = 50.,
}

-- ========================================
-- SCAN MATCHING (Real-time pose estimation)
-- ========================================

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Real-time correlative scan matcher settings
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher (fine-tuning pose with optimization)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options = {
  use_nonmonotonic_steps = false,
  max_num_iterations = 20,
  num_threads = 1,
}

-- ========================================
-- MOTION FILTER
-- ========================================

-- Only process scans when robot has moved enough (reduces noise from stationary scans)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2  -- 20cm movement threshold
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- ~1 degree rotation threshold

-- ========================================
-- SUBMAP SETTINGS
-- ========================================

-- Number of scans to accumulate per submap (smaller = less CPU/memory)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- Submap grid options
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5cm grid resolution

-- ========================================
-- SCAN ACCUMULATION
-- ========================================

-- For single laser scan (not point cloud)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ========================================
-- POSE GRAPH OPTIMIZATION
-- ========================================

-- Optimize global pose graph less frequently (reduces CPU)
POSE_GRAPH.optimize_every_n_nodes = 50

-- Constraint builder settings (loop closure detection)
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

-- Sample fewer constraints to reduce CPU load
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1

-- Fast correlative scan matcher for loop closure
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for loop closure refinement
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options = {
  use_nonmonotonic_steps = true,
  max_num_iterations = 10,
  num_threads = 1,
}

-- ========================================
-- OPTIMIZATION PROBLEM
-- ========================================

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5

-- Odometry is not used, reduce its weight
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.

-- Local SLAM pose weight
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

-- ========================================
-- CONSTRAINT WEIGHTS
-- ========================================

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- Max constraint count to limit memory usage
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.log_matches = true

-- ========================================
-- ADDITIONAL OPTIMIZATIONS
-- ========================================

-- Matcher settings for better noise handling
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

-- Global sampling ratio (reduce for less CPU)
POSE_GRAPH.global_sampling_ratio = 0.003

return options
