# Cartographer
* 목표 : Cartographer 아키텍쳐 이해

![](https://raw.githubusercontent.com/cartographer-project/cartographer/master/docs/source/high_level_system_overview.png)

* 그림 읽는 방법
  * 그림 해석해보기

* [Cartographer-ROS 문서](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)
* [Cartographer-ROS github](https://github.com/cartographer-project/cartographer_ros)

[![Video](http://img.youtube.com/vi/L51S2RVu-zc/0.jpg)](http://www.youtube.com/watch?v=L51S2RVu-zc)

----


## 구성
1. Input
2. Local SLAM
3. Global SLAM


## 1. Input
* LiDAR 센서 : 거리 측정
  * Noise로 인해 SLAM에 영향을 줄 수 있음.
* filer 적용 : min ~ max 사이 값만 사용  
```bash
TRAJECTORY_BUILDER_nD.min_range
TRAJECTORY_BUILDER_nD.max_range
```

* 16 채널 혹은 32 채널 LiDAR의 경우 한꺼번에 많은 데이터를 전송
```bahs
TRAJECTORY_BUILDER_nD.num_accumulated_range_data
```

* LiDAR 위치는 고정되어 있지만 FOV 즉 다양한 각도에서 측정. 가까운 지점은 자주 hit이 일어나고 먼거리는 hit이 늦게 일어남. 이를 위해서 point clouds에 대한 subsampling이 필요.
```bash
TRAJECTORY_BUILDER_nD.voxel_filter_size
```

* fixed-size voxel filter 통과 후에 adaptive voxel filter 적용. 최적화된 voxel size를 결정하기 위해서.
```bash
TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length
TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points
```

* IMU 센서사용시 중력 방향 정확성, 회전에 대한 
```bash
TRAJECTORY_BUILDER_2D.use_imu_data
TRAJECTORY_BUILDER_nD.imu_gravity_time_constant
```

## 2. Local SLAM
* 2가지 scan matching 전략
  1. CeresScanMatcher
  2. RealTimeCorrelativeScanMatcher

* CeresSCanMatcher에서 input에 대한 weight 전략 
```bash
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.translation_weight
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.rotation_weight
```

* 2개 scan 사이에 motion 발생시 최적화 
```bash
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads
```

* RealTimeCorrelativeScanMatcher
```bash
TRAJECTORY_BUILDER_nD.use_online_correlative_scan_matching
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.linear_search_window
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.angular_search_window
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.translation_delta_cost_weight
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.rotation_delta_cost_weight
```

* motion filter 설정
```bash
TRAJECTORY_BUILDER_nD.motion_filter.max_time_seconds
TRAJECTORY_BUILDER_nD.motion_filter.max_distance_meters
TRAJECTORY_BUILDER_nD.motion_filter.max_angle_radians
```

* local SLAM이 완료되는 range data의 양
```bash
TRAJECTORY_BUILDER_nD.submaps.num_range_data
```

* submap은 range data를 저장하는 2가지 자료구조를 가짐
```bash
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type
```

* hit/miss에 대해서 다른 weight 추기 (센서 특징에 따라서)
```bash
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability
```
* Scan mathcing 시작 방법 : 낮은 해상도의 멀리 있는 point를 낮은 해상도의 hybrid grid에 먼저 맞추고 가까운 높은 해상도 point를 높은 해상도 hybrid grid에 맞춰서 pose를 정확히 맞춰 나감.
```bash
```bash
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution
TRAJECTORY_BUILDER_3D.submaps.high_resolution
TRAJECTORY_BUILDER_3D.submaps.low_resolution
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range
```

## 3. Global SLAM
```bash
POSE_GRAPH.optimize_every_n_nodes
```

```bash
POSE_GRAPH.constraint_builder.max_constraint_distance
POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window
POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_xy_search_window
POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_z_search_window
POSE_GRAPH.fast_correlative_scan_matcher*.angular_search_window
```

```bash
POSE_GRAPH.constraint_builder.sampling_ratio
```


```bash
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth
```

```bash
POSE_GRAPH.constraint_builder.min_score
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d
POSE_GRAPH.constraint_builder.ceres_scan_matcher
```

```bash
POSE_GRAPH.constraint_builder.loop_closure_translation_weight
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight
POSE_GRAPH.matcher_translation_weight
POSE_GRAPH.matcher_rotation_weight
POSE_GRAPH.optimization_problem.*_weight
POSE_GRAPH.optimization_problem.ceres_solver_options
```

```bash
POSE_GRAPH.optimization_problem.log_solver_summary
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d
```

```bash
POSE_GRAPH.optimization_problem.huber_scale
```

```bash
POSE_GRAPH.max_num_final_iterations
```

