> 提供一个官方文档的链接：chrome-extension://cdonnmffkdaoajfknoeeecmchibpmkmg/assets/pdf/web/viewer.html?file=https%3A%2F%2Fbuildmedia.readthedocs.org%2Fmedia%2Fpdf%2Fgoogle-cartographer-ros%2Flatest%2Fgoogle-cartographer-ros.pdf
>文档中p33的Lua configuration reference documentation有对部分设置的简介

```lua
--map_builder.lua
MAP_BUILDER = {
  use_trajectory_builder_2d = false, -- 使用2dslam
  use_trajectory_builder_3d = false, -- 使用3dslam
  num_background_threads = 4, --（核数（双核四线程=2 ））
  pose_graph = POSE_GRAPH, --后端优化的具体参数配置
}
```
```lua
--map_builder_server.lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true, --使用imu数据
  min_range = 0.,   
  max_range = 30., --选择带通滤波器过滤范围上下界
  min_z = -0.8,
  max_z = 2., --专门针对于3dslam中的z轴进行点云切割
  missing_data_ray_length = 5., --和ray_trace_range类似，该值通常要大于max_range
  num_accumulated_range_data = 1, --将n个UDP数据包的点云(per-UDP-packet point clouds)合成一个大的点云
  voxel_filter_size = 0.025, #体素滤波器大小，过小则意味着更多的计算量，过大则意味着部分数据可能丢失。该滤波器在裁剪数据后会立即应用到范围数据上

  adaptive_voxel_filter = { --自适应体素滤波器的详细参数。该滤波器计算一个稀疏点云用于进行匹配
    max_length = 0.5, --限制最大长度作为优化约束
    min_num_points = 200, --一个体素cube中包含点云的最小量
    max_range = 50., 
  },

  loop_closure_adaptive_voxel_filter = { --该自适应滤波器计算一个稀疏点云用于回环检测
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false, --是否用correlative scan matcher给Ceres生成一个良好的优化起点
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

  imu_gravity_time_constant = 10., --IMU的重力加速度常数

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

* use_trajectory_builder_2d：为使用2d slam 。在进行2d slam时，carto的默认配置是使用imu的，所以如果没有imu就要TRAJECTORY_BUILDER_2D.use_imu_data = false，否则会一直等待imu的数据而进行不下去。而3d slam必须使用imu，所以就没有这个参数配置。

* TRAJECTORY_BUILDER_nD.min(max)_range: 测距传感器(例如:激光雷达)提供多个方向的深度信息。然而，有些测量结果与SLAM无关(传感器部分被灰尘覆盖或测量到噪声数据,或者一些远距离测量的数据可能来自一些不被希望纳入的噪音源或者反射等等)。为了解决这些问题，Cartographer首先应用一个bandpass filter(带通滤波器) ，并保持滤波器范围值（bandwidth）在一定的范围之间，其最小值和最大值应根据机器人和传感器的规格选择。



```
refer:
-- min_z, max_z, min_range, and max_range refer to the pointcloud points.
-- we only keep points that are within the "min_z < pointcloud_point_height < max_z" range
-- we only keep points, whose distance are within the "min_range < pointcloud_point_distance < max_range" range.
-- all pointcloud points that fall outside of these ranges are discarded.
-- if min_range is < 1.1, it detects the support beams at the back of the robot as obstacles
TRAJECTORY_BUILDER_2D.min_range = 1.1
-- max_range is similar in function to obstacle_range (if you prefer to think of it that way).  Any obstacle that it
-- detects that is less than max_range (and bigger than min_range) will be added to the map.
-- missing_data_ray_length is similar in function to ray_trace_range.  
-- ray_trace_range should ALWAYS be bigger than max_range.
-- Why?  You don't want the max_range to be able to add obstacles to your map that you can't raytrace away.
-- Suppose raytrace is 5m and obstaclerange is 10m.  Suppose a person walked in front of the robot and out of range 
-- of its sensors.  At some point, the person would be at 3m.  Then at 4m away, then 5m, then 6m, etc.
-- When they are at 3m, they will be detected as an obstacle.  At the next time step, they will be at 4m.  We will 
-- raytrace away the obstacle at 3m, and place one at 4m.  The same thing happens at 4m->5m.  Now consider what happens
-- at 6m -> 7m.  An obstacle was placed at 6m.  We now detect the person at 7m, and place an obstacle there.  But 
-- because our raytrace range is limited to 5m, we cannot raytrace away the obstacle at 6m,  It stays there forever.
-- This can leave phantom obstacles in the map.  
-- THE SOLUTION IS TO ALWAYS HAVE THE RAYTRACE_RANGE BE LARGER THAN THE OBSTACLE_RANGE  
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 160.0
```

* voxel_filter_size: 范围数据通常是从机器人上的单个点以多个角度测量的，因此闭合的表面（例如道路）会提供很多点云。但远处的物体被激光击中的机会更少，而提供的点云则更少。为了减轻点云处理的计算量，我们通常需要对点云进行降采样。但是，简单的随机采样会导致低密度区域中点变得更少，而高密度区域仍然具有比所需数量更多的点。为了解决该问题，cartographer使用体素滤波器过滤数据，并且仅保留每个体素（cube）的质心。较小的体素大小将导致更密集的数据表示，从而导致更多的计算量。较大的体素大小会导致数据丢失，但会更快。在3dslam中分别应用高通和低通滤波器。

```lua
MAX_3D_RANGE = 60.

TRAJECTORY_BUILDER_3D = {
  min_range = 1.,
  max_range = MAX_3D_RANGE,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.15,

  high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 15.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = MAX_3D_RANGE,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 1.,
    occupied_space_weight_1 = 6.,
    translation_weight = 5.,
    rotation_weight = 4e2,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.1,
    max_angle_radians = 0.004,
  },

  imu_gravity_time_constant = 10.,
  rotational_histogram_size = 120,

  submaps = {
    high_resolution = 0.10,
    high_resolution_max_range = 20.,
    low_resolution = 0.45,
    num_range_data = 160,
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
      num_free_space_voxels = 2,
    },
  },
}
```
