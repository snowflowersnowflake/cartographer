> 提供一个官方文档的链接：chrome-extension://cdonnmffkdaoajfknoeeecmchibpmkmg/assets/pdf/web/viewer.html?file=https%3A%2F%2Fbuildmedia.readthedocs.org%2Fmedia%2Fpdf%2Fgoogle-cartographer-ros%2Flatest%2Fgoogle-cartographer-ros.pdf
文档中p33的Lua configuration reference documentation有对设置的简介
```lua
MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
}
```
```lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0.,
  max_range = 30.,
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 50.,
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  imu_gravity_time_constant = 10.,

  submaps = {
    num_range_data = 90,
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
    },
  },
}
```
* use_trajectory_builder_2d：为使用2d slam 。在进行2d slam 时，carto的默认配置是使用imu的，所以如果没有imu就要TRAJECTORY_BUILDER_2D.use_imu_data = false，否则会一直等待imu的数据而进行不下去。而3d 的 slam 必须使用imu，所以就没有这个参数配置。
