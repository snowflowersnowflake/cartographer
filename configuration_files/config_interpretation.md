> 官方文档文档中p33的Lua configuration reference documentation有对部分设置的简介：chrome-extension://cdonnmffkdaoajfknoeeecmchibpmkmg/assets/pdf/web/viewer.html?file=https%3A%2F%2Fbuildmedia.readthedocs.org%2Fmedia%2Fpdf%2Fgoogle-cartographer-ros%2Flatest%2Fgoogle-cartographer-ros.pdf


```lua
--map_builder.lua
MAP_BUILDER = {
  use_trajectory_builder_2d = false, -- 使用2dslam
  use_trajectory_builder_3d = false, -- 使用3dslam
  num_background_threads = 4, --（核数（双核四线程=2 ））
  pose_graph = POSE_GRAPH, --后端优化的具体参数配置
}
```
* use_trajectory_builder_2d：为使用2d slam 。在进行2d slam时，carto的默认配置是使用imu的，所以如果没有imu就要TRAJECTORY_BUILDER_2D.use_imu_data = false，否则会一直等待imu的数据而进行不下去。而3d slam必须使用imu，所以就没有这个参数配置。

```lua
--map_builder_server.lua
MAP_BUILDER_SERVER = {
  map_builder = MAP_BUILDER,
  num_event_threads = 4,
  num_grpc_threads = 4,
  server_address = "0.0.0.0:50051",
  uplink_server_address = "",
  upload_batch_size = 100,
  enable_ssl_encryption = false,
}
```

```lua
--trajectory_builder.lua
TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D, --根据 2d/3d SLAM选择对应的轨迹构建模式
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,
  pure_localization = false, --纯定位模式，如果只用cartographer进行定位可以开启   
}
```
* pure_localization应用于重定位教程：https://github.com/zhang-datou/cartographer_ros

```lua
--trajectory_builder_2d.lua
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
    linear_search_window = 0.1, --最小化线性搜索窗口，用于搜索最有可能性的scan alignment 
    angular_search_window = math.rad(20.), --最小化角度搜索窗口，用于搜索最有可能性的scan alignment 
    translation_delta_cost_weight = 1e-1, --该处的权重应用到score的每一部分
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {  --解决最小二乘问题进行匹配
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40., 
    ceres_solver_options = {
      use_nonmonotonic_steps = false, --使用非单调的step，该定义在ceres官网中有解释
      max_num_iterations = 20, --优化器的最大迭代次数
      num_threads = 1,
    },
  },

  motion_filter = { --2d中用来判断是否对子图进行更新的一个类
    max_time_seconds = 5., --基于时间插入范围数据的阈值
    max_distance_meters = 0.2, --基于直线运动插入距离数据的阈值
    max_angle_radians = math.rad(1.), --基于旋转轴数据插入范围数据的阈值
  },

  imu_gravity_time_constant = 10., --IMU的重力加速度常数

  submaps = { --设置子图
    num_range_data = 90,--添加新子图（submap）之前的范围数据数。每个子图将得到两倍数量的范围数据插入
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05, --分辨率
    },
    range_data_inserter = { --和resilution一起存储指定分辨率概率栅格
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = { --关于grid的占用
        insert_free_space = true, --如果“False”那么自由空间（free space）不会改变占用网格的概率
        hit_probability = 0.55, --命中的概率变化(该数值将转换为几率，因此必须大于0.5)
        miss_probability = 0.49, --错过的概率变化(同上，因此必须小于0.5)
      },
    },
  },
}
```

* TRAJECTORY_BUILDER_nD.min(max)_range: 测距传感器(例如:激光雷达)提供多个方向的深度信息。然而，有些测量结果与SLAM无关(传感器部分被灰尘覆盖或测量到噪声数据,或者一些远距离测量的数据可能来自一些不被希望纳入的噪音源或者反射等等)。为了解决这些问题，Cartographer首先应用一个bandpass filter(带通滤波器) ，并保持滤波器范围值（bandwidth）在一定的范围之间，其最小值和最大值应根据机器人和传感器的规格选择。


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


* ceres_scan_matcher: 需要注意的是，在前端的SLAM定位过程中，cartographer3D并没有像2D那样先使用scan matching确定初值，然后进行ceres优化，而是直接使用IMU提供的角速度进行积分，获得一个旋转初值，用于优化姿态，而如果有里程计，会给出一个平移的初值，如果没有里程计，该初值为0。因此前端定位直接是ceres优化。

* motion_filter: MotionFilter是cartographer中用来判断是否对子图进行更新的一个类，具体应用在local_trajectory_build_2d和local_trajectory_build_2d中。首先这里对MotionFilter的使用场景进行推导，例如，在local_trajectory_build_2d中，AddRangeData函数调用AddAccumulatedRangeData函数来扫描匹配并插入扫描数据，AddAccumulatedRangeData函数在扫描匹配完成后，会调用InsertIntoSubmap函数将扫描数据插入子图，而InsertIntoSubmap函数第一步就会调用motion_filter_.IsSimilar(time, pose_estimate)来判断当前扫描是否能插入子图中



```lua
pose_graph.lua

POSE_GRAPH = { --位姿图，很重要的一个类
  optimize_every_n_nodes = 90, --实时闭环，如果为正值则n个节点闭环一次(num_nodes_since_last_loop_closure > optimize_every_n_nodes ==> run_loop_closure)
  constraint_builder = { --用于生成优化约束的选项
    sampling_ratio = 0.3,  --如果添加的约束与潜在约束(potential constraints)的比例低于这个数字，就会添加约束
    max_constraint_distance = 15.,--pose被判断为归属一个子图的距离阈值
    min_score = 0.55, -- 置信度（score）低于该阈值则不考虑匹配，低的score表明scan和map看起来不相似
    global_localization_min_score = 0.6, --低于此阈值的全局定位不被信任
    loop_closure_translation_weight = 1.1e4,  --用于平移分量回环约束优化中的权重
    loop_closure_rotation_weight = 1e5,  --用于旋转分量回环约束优化中的权重
    log_matches = true, --设置为true则记录可用于调试的回环约束信息
    fast_correlative_scan_matcher = { --内联scan_matcher的设置
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20., --每个代价函子（functor, 范畴间的映射）的比例系数
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8, --用于预计算的栅格（grid）的数量
      full_resolution_depth = 3, --要使用的全分辨率栅格的数量，其他栅格的分辨率只有一半
      min_rotational_score = 0.77, --旋转匹配的最小置信度
      min_low_resolution_score = 0.55, --仅用于3d中低分辨栅格的置信度
      linear_xy_search_window = 5., --xy平面上的线性搜索窗口
      linear_z_search_window = 1., --z上的线性搜索窗口
      angular_search_window = math.rad(15.), --角窗口
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2, --用于平移分量的无回环优化问题中的权重
  matcher_rotation_weight = 1.6e3, --用于旋转分量的无回环优化问题中的权重
  optimization_problem = {
    huber_scale = 1e1, --Huber损失函数的比例因子
    acceleration_weight = 1.1e2, --IMU加速项的比例因子
    rotation_weight = 1.6e4, --IMU旋转项的比例因子
    local_slam_pose_translation_weight = 1e5, --局部位姿下连续节点间平移变换的比例因子
    local_slam_pose_rotation_weight = 1e5, --局部位姿下连续节点间旋转变换的比例因子
    odometry_translation_weight = 1e5, --里程计上连续节点间平移变换的比例因子
    odometry_rotation_weight = 1e5, --里程计上连续节点间旋转变换的比例因子
    fixed_frame_pose_translation_weight = 1e1, -- FixedFramePose平移的比例因子 （FixedFrame是全局显示区域依托的坐标系）
    fixed_frame_pose_rotation_weight = 1e2,  -- FixedFramePose旋转的比例因子
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false, -- 设置为True则每次优化都会记录Ceres求解器信息（summary）
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200, --优化问题中对于最终优化的迭代次数
  global_sampling_ratio = 0.003, --在全局定位中对单条轨迹节点采样的频率
  log_residual_histograms = true, --是否输出pose残差的直方图
  global_constraint_search_after_n_seconds = 10., --如果该时间内没有在两个轨迹之间添加全局约束，则全局执行闭环搜索，而不是局部执行
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}

```
