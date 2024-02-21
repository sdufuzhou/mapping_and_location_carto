/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-30 21:34:22
 * @LastEditTime: 2022-10-25 16:18:01
 */
/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-28 09:48:44
 * @LastEditTime: 2022-09-30 21:34:19
 * @Author: lcfc-desktop
 */
#pragma once
#include <string>
namespace gomros {
namespace data_process {
namespace mapping_and_location {
/**
 * @brief 建图方式枚举：Online为在线建图，Offline为离线建图
 *
 */
enum MappingPattern { Online, Offline };

enum MappingAlgorithm { Cartro, Karto };

struct KartoMappingConfig {
  MappingPattern mapping_pattern;  // 建图方式
  float distance_step;             // 0.1
  float angular_step;              // 0.1
  float mapping_start_angle;       // -95.0
  float mapping_end_angle;         // 95.0
  float radar_resolution;          // 0.5
  float mapping_resolution;        // 0.02
  float radar_position_x;          // 0.361
  float radar_position_y;          // 0.0
  float radar_position_theta;      // 0.0
  float mapping_laser_min_range;   // 0.5
  float mapping_laser_max_range;   // 19.0
  float
      travel_distance;  //设置最小距离，里程计移动长度超过此距离，则建立一个节点,0.2
  float
      travel_angle;  // 设置最小偏转角，如果里程计转向超过此值，则建立一个节点,0.175
  float loop_search_distance;  // 搜寻回环匹配的最大距离,4.0
  std::string map_data_file_path;
  std::string map_config_file_path;
};

typedef struct AdaptiveVoxelFilterConfig {
  float max_length;      // 0.5,
  float min_num_points;  // 200,
  float max_range;       // 50.,
} AdaptiveVoxelFilterConfig;

typedef struct LoopClosureAdaptiveVoxelFilterConfig {
  float max_length;      // 0.9
  float min_num_points;  // 100,
  float max_range;       // 50.,
} LoopClosureAdaptiveVoxelFilterConfig;

typedef struct RealTimeCorrelativeScanMatcherConfig {
  double linear_search_window;   // 0.1,
  double angular_search_window;  // math.rad(20.),
  double translation_delta_cost_weight;  // 1e-1,   -- 用于计算各部分score的权重
  double rotation_delta_cost_weight;  // 1e-1,
} RealTimeCorrelativeScanMatcherConfig;

typedef struct CeresSolverConfig {
  bool use_nonmonotonic_steps;  // false;
  int max_num_iterations;       // 10;
  int num_threads;
} CeresSolverConfig;

// 1.4 -- ceres匹配的一些配置参数
typedef struct CeresScanMatcherConfig {
  double occupied_space_weight;  // 1.,
  
  double translation_weight;     // 10.,
  double rotation_weight;        // 40.,
  // CeresSolverConfig
  CeresSolverConfig ceres_solver_config;
} CeresScanMatcherConfig;

// 1.5 为了防止子图里插入太多数据, MotionFilterConfig
typedef struct MotionFilterConfig {
  double max_time_seconds;     // 5.,
  double max_distance_meters;  // 0.2,
  double max_angle_radians;    // math.rad(1.),
} MotionFilterConfig;

typedef struct ConstantVelocityConfig {
  double imu_gravity_time_constant;  // 10.0
  double pose_queue_duration;        // 0.001
} ConstantVelocityConfig;

typedef struct ImuBasedConfig {
  double pose_queue_duration;          // 5.,
  double gravity_constant;             // 9.806,
  double pose_translation_weight;      // 1.,
  double pose_rotation_weight;         // 1.,
  double imu_acceleration_weight;      // 1.,
  double imu_rotation_weight;          // 1.,
  double odometry_translation_weight;  // 1.,
  double odometry_rotation_weight;     // 1.,
  CeresSolverConfig ceres_solver_config;
} ImuBasedConfig;

typedef struct PoseExtrapolatorConfig {
  bool use_imu_based;  // false,
  ConstantVelocityConfig constant_velocity_config;
  ImuBasedConfig imu_based_config;
} PoseExtrapolatorConfig;

typedef struct ProbabilityGridRangeDataInserterConfig {
  bool insert_free_space;   // true,
  double hit_probability;   // 0.55,
  double miss_probability;  // 0.49,
} ProbabilityGridRangeDataInserterConfig;

typedef struct TsdfRangeDataInserterConfig {
  double truncation_distance;
  double maximum_weight;
  bool update_free_space;
  int num_normal_samples;
  double sample_radius;
  bool project_sdf_distance_to_scan_normal;
  double update_weight_range_exponent;
  double update_weight_angle_scan_normal_to_ray_kernel_bandwidth;
  double update_weight_distance_cell_to_hit_kernel_bandwidth;
} TsdfRangeDataInserterConfig;

typedef struct RangeDataInserterConfig {
  int range_data_inserter_type;  // "PROBABILITY_GRID_INSERTER_2D",
  ProbabilityGridRangeDataInserterConfig
      probability_grid_range_data_inserter_config;
  TsdfRangeDataInserterConfig tsdf_range_data_inserter_config;
} RangeDataInserterConfig;

// 1.7 子图(采用概率栅格地图)相关的一些配置
typedef struct SubmapConfig {
  int num_range_data;  // 90,  一个子图里插入雷达数据的个数的一半
  int grid_type;       //"PROBABILITY_GRID"
  float resolution;    // 0.05,
  RangeDataInserterConfig range_data_inserter_config;
} SubmapConfig;

//------------后端--------------
typedef struct FastCorrelativeScanMatcherConfig {
  double linear_search_window;   // 7.,
  double angular_search_window;  // math.rad(30.),
  int branch_and_bound_depth;    // 7,
} FastCorrelativeScanMatcherConfig;

typedef struct ConstraintBuilderConfig {
  double sampling_ratio;                   // 0.3
  double max_constraint_distance;          // 15.,
  double min_score;                        // 0.55
  double global_localization_min_score;    // 0.6,  --
  double loop_closure_translation_weight;  // 1.1e4,
  double loop_closure_rotation_weight;     // 1e5,
  bool log_matches;                        // true,
  FastCorrelativeScanMatcherConfig fast_correlative_scan_matcher_config;
  CeresScanMatcherConfig ceres_scan_matcher_config;
} ConstraintBuilderConfig;

typedef struct OptimizationProblemConfig {
  double huber_scale;  // 1e1,
  double acceleration_weight;
  double rotation_weight;
  double local_slam_pose_translation_weight;      // 1e5,
  double local_slam_pose_rotation_weight;         // 1e5,
  double odometry_translation_weight;             // 1e5,
  double odometry_rotation_weight;                // 1e5,
  double fixed_frame_pose_translation_weight;     // 1e1,
  double fixed_frame_pose_rotation_weight;        // 1e2,
  bool fixed_frame_pose_use_tolerant_loss;        // false,
  double fixed_frame_pose_tolerant_loss_param_a;  // 1,
  double fixed_frame_pose_tolerant_loss_param_b;  // 1,
  bool log_solver_summary;                        // false,
  bool use_online_imu_extrinsics_in_3d;           // true,
  bool fix_z_in_3d;                               // false,
  CeresSolverConfig ceres_solver_config;          // 7,
} OptimizationProblemConfig;

typedef struct PoseGraphConfig {
  int optimize_every_n_nodes;  // 90,
  ConstraintBuilderConfig constraint_builder_config;
  float matcher_translation_weight;  // 5e2,
  float matcher_rotation_weight;     // 1.6e3,
  OptimizationProblemConfig optimization_problem_config;
  int max_num_final_iterations;                    // 200,
  float global_sampling_ratio;                     // 0.003,
  bool log_residual_histograms;                    // true,
  float global_constraint_search_after_n_seconds;  // 10.,
} PoseGraphConfig;

typedef struct TrajectoryBuilderConfig {
  bool use_imu_data;
  bool use_odom_data;
  double min_range;
  double max_range;
  float min_z;                     // -0.8,
  float max_z;                     // 2.,
  float missing_data_ray_length;   // 5.,
  int num_accumulated_range_data;  // 1,
  float voxel_filter_size;         // 0.025,
  AdaptiveVoxelFilterConfig adaptive_voxel_filter_config;
  LoopClosureAdaptiveVoxelFilterConfig
      loop_closure_adaptive_voxel_filter_config;
  bool use_online_correlative_scan_matching;
  RealTimeCorrelativeScanMatcherConfig
      real_time_correlative_scan_matcher_config;
  CeresScanMatcherConfig ceres_scan_matcher_config;
  MotionFilterConfig motion_filter_config;
  double imu_gravity_time_constant;
  PoseExtrapolatorConfig pose_extrapolator_config;
  SubmapConfig submap_config;
  bool collate_fixed_frame;  // true,
  bool collate_landmarks;    // false
} TrajectoryBuilderConfig;

typedef struct CartoMappingConfig {
  // 前端
  int mapping_pattern;  // 工程文件中用int类型，后面进行强制类型转换
  int mapping_start_angle;
  int mapping_end_angle;
  float mapping_resolution;
  float radar_resolution;
  float radar_position_x;      // 0.361
  float radar_position_y;      // 0.0
  float radar_position_theta;  // 0.0
  //-----新加：轮式里程计在tracking_frame下的位姿()------
  float wheel_odom_position_x;      // 0
  float wheel_odom_position_y;      // 0
  float wheel_odom_position_theta;  // 0
  std::string map_data_file_path;
  std::string map_config_file_path;

  int num_background_threads;
  TrajectoryBuilderConfig trajectory_config;
  PoseGraphConfig pose_graph_config;
  bool collate_by_trajectory;
} CartoMappingConfig;

/**
 * @brief 栅格定位配置参数
 *
 */
typedef struct GridLocationConfig {
  float mapping_start_angle;   // -95.0
  float mapping_end_angle;     // 95.0
  float laser_resolution;      // 1.0
  float radar_position_x;      // 0.361
  float radar_position_y;      // 0.0
  float radar_position_theta;  // 0.0
  bool use_scan_matching;
  float scan_matching_min_distance;     // 0.0  scan_matching
  float scan_matching_max_distance;     // 19.0 scan_matching
  float occupied_space_cost_factor;     // 1.0 scan_matching
  float translation_delta_cost_factor;  // 10.0 scan_matching
  float rotation_delta_cost_factor;     // 10.0 scan_matching
  int num_threads;                      // 2 scan_matching
  int max_num_iterations;               // 10 scan_matching
  float laser_max_range;                // 19.0
  float laser_min_range;                // 0.5
  float distance_threhold;              // 粒子滤波触发位移条件，0.1
  float angle_threahold;                // 粒子滤波触发角度条件，0.1
  int resample_interval;                // 重采样间隔，5
  int laser_max_beams;  //  更新滤波器时，每次扫描中多少个等间距的光束被使用,60
  int odom_type;        // 0
  int only_radar_pose;  // 0
  bool is_initial_locate;
  // 初始位姿文件存放路径,必须和地图管理模块的地图文件路径一样
  std::string init_pose_file_path;
} GridLocationConfig;

typedef struct MappingAndLocationConfig {
  std::string node_name;
  std::string sub_odom_topic;   // 订阅里程计话题
  std::string sub_radar_topic;  // 订阅雷达话题
  std::string sub_imu_topic;    // ----------订阅Imu话题-----------
  std::string sub_global_locate_cmd_topic;  // 订阅全局初始化命令
  std::string sub_start_mapping_cmd_topic;  // 订阅开始建图命令
  std::string sub_stop_mapping_cmd_topic;   // 订阅停止建图命令
  std::string ad_state_topic;               // 更改状态发布的话题名
  std::string ad_pose_topic;                // 位姿发布的话题名
  std::string log_file_name;                // 日志文件文件名
  int log_level;                            // 日志等级
  bool is_printf_to_terminal;               // 是否将日志输出到终端
  MappingAlgorithm mapping_algorithm;  // 建图算法，0为carto，1为karto
  KartoMappingConfig karto_config;    //Karto算法建图配置参数
  CartoMappingConfig carto_config;    //Carto算法建图配置参数
  GridLocationConfig location_config;  //机器人建图前的栅格定位参数配置
} MappingAndLocationConfig;
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
