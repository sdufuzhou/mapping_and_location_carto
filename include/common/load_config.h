/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-10 13:47:58
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-09 06:15:35
 */
#pragma once
#include <string>

#include "agv_config_lib/ConfigUtils.h"
#include "include/config_struct.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

typedef struct KartoMappingCfg {
  int mapping_pattern;  // 建图方式
  float distance_step;  // 0.1
  float angular_step;   // 0.1
  float mapping_resolution;
  float mapping_laser_min_range;  // 0.5
  float mapping_laser_max_range;  // 19.0
  //设置最小距离，里程计移动长度超过此距离，则建立一个节点,0.2
  float travel_distance;
  // 设置最小偏转角，如果里程计转向超过此值，则建立一个节点,0.175
  float travel_angle;
  // 搜寻回环匹配的最大距离,4.0
  float loop_search_distance;
  std::string map_data_file_path;
  std::string map_config_file_path;
  XPACK(O(mapping_pattern, distance_step, angular_step, mapping_resolution,
          mapping_laser_min_range, mapping_laser_max_range, travel_distance,
          travel_angle, loop_search_distance, map_data_file_path,
          map_config_file_path));
} KartoMappingCfg;

typedef struct AdaptiveVoxelFilterCfg {
  float max_length;      // 0.5,
  float min_num_points;  // 200,
  float max_range;       // 50.,
  XPACK(O(max_length, min_num_points, max_range));
} AdaptiveVoxelFilterCfg;

// 1.2  闭环检测的自适应体素滤波器, 用于生成稀疏点云 以进行 闭环检测
typedef struct LoopClosureAdaptiveVoxelFilterCfg {
  float max_length;      // 0.9,
  float min_num_points;  // 100,
  float max_range;       // 50.,
  XPACK(O(max_length, min_num_points, max_range));
} LoopClosureAdaptiveVoxelFilterCfg;

// 1.3 RealTimeCorrelativeScanMatcherCfg
typedef struct RealTimeCorrelativeScanMatcherCfg {
  double linear_search_window;   // 0.1,
  double angular_search_window;  // math.rad(20.),
  double translation_delta_cost_weight;  // 1e-1,   -- 用于计算各部分score的权重
  double rotation_delta_cost_weight;  // 1e-1,
  XPACK(O(linear_search_window, angular_search_window,
          translation_delta_cost_weight, rotation_delta_cost_weight));
} RealTimeCorrelativeScanMatcherCfg;

typedef struct CeresSolverCfg {
  bool use_nonmonotonic_steps;  // false;
  int max_num_iterations;       // 10;
  int num_threads;
  XPACK(O(use_nonmonotonic_steps, max_num_iterations, num_threads));
} CeresSolverCfg;

// 1.4 -- ceres匹配的一些配置参数
typedef struct CeresScanMatcherCfg {
  double occupied_space_weight;  // 1.,
  double translation_weight;     // 10.,
  double rotation_weight;        // 40.,
  // CeresSolverCfg
  CeresSolverCfg ceres_solver_config;
  XPACK(O(occupied_space_weight, translation_weight, rotation_weight,
          ceres_solver_config));
} CeresScanMatcherCfg;

// 1.5 为了防止子图里插入太多数据, MotionFilterCfg
typedef struct MotionFilterCfg {
  double max_time_seconds;     // 5.,
  double max_distance_meters;  // 0.2,
  double max_angle_radians;    // math.rad(1.),
  XPACK(O(max_time_seconds, max_distance_meters, max_angle_radians));
} MotionFilterCfg;

// 1.6 位姿预测器  PoseExtrapolatorCfg
typedef struct ConstantVelocityCfg {
  double imu_gravity_time_constant;  // 10.0
  double pose_queue_duration;        // 0.001
  XPACK(O(imu_gravity_time_constant, pose_queue_duration));
} ConstantVelocityCfg;

typedef struct ImuBasedCfg {
  double pose_queue_duration;          // 5.,
  double gravity_constant;             // 9.806,
  double pose_translation_weight;      // 1.,
  double pose_rotation_weight;         // 1.,
  double imu_acceleration_weight;      // 1.,
  double imu_rotation_weight;          // 1.,
  double odometry_translation_weight;  // 1.,
  double odometry_rotation_weight;     // 1.,
  CeresSolverCfg ceres_solver_config;
  XPACK(O(pose_queue_duration, gravity_constant, pose_translation_weight,
          pose_rotation_weight, imu_acceleration_weight, imu_rotation_weight,
          odometry_translation_weight, odometry_rotation_weight,
          ceres_solver_config));
} ImuBasedCfg;

typedef struct PoseExtrapolatorCfg {
  bool use_imu_based;  // false,
  ConstantVelocityCfg constant_velocity_config;
  ImuBasedCfg imu_based_config;
  XPACK(O(use_imu_based, constant_velocity_config, imu_based_config));
} PoseExtrapolatorCfg;

typedef struct ProbabilityGridRangeDataInserterCfg {
  bool insert_free_space;   // true,
  double hit_probability;   // 0.55,
  double miss_probability;  // 0.49,
  XPACK(O(insert_free_space, hit_probability, miss_probability));
} ProbabilityGridRangeDataInserterCfg;

typedef struct TsdfRangeDataInserterCfg {
  double truncation_distance;
  double maximum_weight;
  bool update_free_space;
  int num_normal_samples;
  double sample_radius;
  bool project_sdf_distance_to_scan_normal;
  double update_weight_range_exponent;
  double update_weight_angle_scan_normal_to_ray_kernel_bandwidth;
  double update_weight_distance_cell_to_hit_kernel_bandwidth;
  XPACK(O(truncation_distance, maximum_weight, update_free_space,
          num_normal_samples, sample_radius,
          project_sdf_distance_to_scan_normal, update_weight_range_exponent,
          update_weight_angle_scan_normal_to_ray_kernel_bandwidth,
          update_weight_distance_cell_to_hit_kernel_bandwidth));
} TsdfRangeDataInserterCfg;

typedef struct RangeDataInserterCfg {
  int range_data_inserter_type;  // "PROBABILITY_GRID_INSERTER_2D",
  ProbabilityGridRangeDataInserterCfg
      probability_grid_range_data_inserter_config;
  TsdfRangeDataInserterCfg tsdf_range_data_inserter_config;
  XPACK(O(range_data_inserter_type, probability_grid_range_data_inserter_config,
          tsdf_range_data_inserter_config));
} RangeDataInserterCfg;

// 1.7 子图(采用概率栅格地图)相关的一些配置
typedef struct SubmapCfg {
  int num_range_data;  // 90,  一个子图里插入雷达数据的个数的一半
  int grid_type;       //"PROBABILITY_GRID"
  float resolution;    // 0.05,
  RangeDataInserterCfg range_data_inserter_config;
  XPACK(O(num_range_data, grid_type, resolution, range_data_inserter_config));
} SubmapCfg;

//------------后端--------------
typedef struct FastCorrelativeScanMatcherCfg {
  double linear_search_window;   // 7.,
  double angular_search_window;  // math.rad(30.),
  int branch_and_bound_depth;    // 7,
  XPACK(O(linear_search_window, angular_search_window, branch_and_bound_depth));
} FastCorrelativeScanMatcherCfg;

typedef struct ConstraintBuilderCfg {
  double sampling_ratio;                   // 0.3
  double max_constraint_distance;          // 15.,
  double min_score;                        // 0.55
  double global_localization_min_score;    // 0.6,  --
  double loop_closure_translation_weight;  // 1.1e4,
  double loop_closure_rotation_weight;     // 1e5,
  bool log_matches;                        // true,
  FastCorrelativeScanMatcherCfg fast_correlative_scan_matcher_config;
  CeresScanMatcherCfg ceres_scan_matcher_config;
  XPACK(O(sampling_ratio, max_constraint_distance, min_score,
          global_localization_min_score, loop_closure_translation_weight,
          loop_closure_rotation_weight, log_matches,
          fast_correlative_scan_matcher_config, ceres_scan_matcher_config));
} ConstraintBuilderCfg;

typedef struct OptimizationProblemCfg {
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
  CeresSolverCfg ceres_solver_config;             // 7,
  XPACK(O(huber_scale, acceleration_weight, rotation_weight,
          local_slam_pose_translation_weight, local_slam_pose_rotation_weight,
          odometry_translation_weight, odometry_rotation_weight,
          fixed_frame_pose_translation_weight, fixed_frame_pose_rotation_weight,
          fixed_frame_pose_use_tolerant_loss,
          fixed_frame_pose_tolerant_loss_param_a,
          fixed_frame_pose_tolerant_loss_param_b, log_solver_summary,
          use_online_imu_extrinsics_in_3d, fix_z_in_3d, ceres_solver_config));
} OptimizationProblemCfg;

typedef struct PoseGraphCfg {
  int optimize_every_n_nodes;  // 90,
  ConstraintBuilderCfg constraint_builder_config;
  float matcher_translation_weight;  // 5e2,
  float matcher_rotation_weight;     // 1.6e3,
  OptimizationProblemCfg optimization_problem_config;
  int max_num_final_iterations;                    // 200,
  float global_sampling_ratio;                     // 0.003,
  bool log_residual_histograms;                    // true,
  float global_constraint_search_after_n_seconds;  // 10.,
  XPACK(O(optimize_every_n_nodes, constraint_builder_config,
          matcher_translation_weight, matcher_rotation_weight,
          optimization_problem_config, max_num_final_iterations,
          global_sampling_ratio, log_residual_histograms,
          global_constraint_search_after_n_seconds
          ));
} PoseGraphCfg;

typedef struct TrajectoryBuilderCfg {
  bool use_imu_data;
  bool use_odom_data;
  double min_range;
  double max_range;
  float min_z;                     // -0.8,
  float max_z;                     // 2.,
  float missing_data_ray_length;   // 5.,
  int num_accumulated_range_data;  // 1,
  float voxel_filter_size;         // 0.025,
  AdaptiveVoxelFilterCfg adaptive_voxel_filter_config;
  LoopClosureAdaptiveVoxelFilterCfg loop_closure_adaptive_voxel_filter_config;
  bool use_online_correlative_scan_matching;
  RealTimeCorrelativeScanMatcherCfg real_time_correlative_scan_matcher_config;
  CeresScanMatcherCfg ceres_scan_matcher_config;
  MotionFilterCfg motion_filter_config;
  double imu_gravity_time_constant;
  PoseExtrapolatorCfg pose_extrapolator_config;
  SubmapCfg submap_config;
  bool collate_fixed_frame;  // true,
  bool collate_landmarks;    // false
  XPACK(O(use_imu_data, use_odom_data, min_range, max_range, min_z, max_z,
          missing_data_ray_length, num_accumulated_range_data,
          voxel_filter_size, adaptive_voxel_filter_config,
          loop_closure_adaptive_voxel_filter_config,
          use_online_correlative_scan_matching,
          real_time_correlative_scan_matcher_config, ceres_scan_matcher_config,
          motion_filter_config, imu_gravity_time_constant,
          pose_extrapolator_config, submap_config, collate_fixed_frame,
          collate_landmarks))
} TrajectoryBuilderCfg;

typedef struct CartoMappingCfg {
  // 前端
  int mapping_pattern;  // 工程文件中用int类型，后面进行强制类型转换
  float mapping_resolution;
  //-----新加：轮式里程计在tracking_frame下的位姿()------
  float wheel_odom_position_x;      // 0
  float wheel_odom_position_y;      // 0
  float wheel_odom_position_theta;  // 0
  std::string map_data_file_path;
  std::string map_config_file_path;
  int num_background_threads;
  TrajectoryBuilderCfg trajectory_config;
  PoseGraphCfg pose_graph_config;
  bool collate_by_trajectory;
  XPACK(O(mapping_pattern, mapping_resolution, wheel_odom_position_x,
          wheel_odom_position_y, wheel_odom_position_theta, map_data_file_path,
          map_config_file_path, num_background_threads, trajectory_config,
          pose_graph_config, collate_by_trajectory));
} CartoMappingCfg;

/**
 * @brief 栅格定位配置参数
 *
 */
typedef struct GridLocationCfg {
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
  XPACK(O(use_scan_matching, scan_matching_min_distance,
          scan_matching_max_distance, occupied_space_cost_factor,
          translation_delta_cost_factor, rotation_delta_cost_factor,
          num_threads, max_num_iterations, laser_max_range, laser_min_range,
          distance_threhold, angle_threahold, resample_interval,
          laser_max_beams, odom_type, only_radar_pose, is_initial_locate,
          init_pose_file_path));
} GridLocationCfg;

typedef struct MappingAndLocationCfg {
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
  int mapping_algorithm;
  float radar_start_angle;       // -95.0
  float radar_end_angle;         // 95.0
  float radar_resolution;        // 0.5
  float radar_position_x;        // 0.361
  float radar_position_y;        // 0.0
  float radar_position_theta;    // 0.0
  CartoMappingCfg carto_config;  // Carto算法建图配置参数
  KartoMappingCfg karto_config;
  GridLocationCfg location_config;  // 机器人建图前的栅格定位参数配置
  XPACK(O(node_name, sub_odom_topic, sub_radar_topic, sub_imu_topic,
          sub_global_locate_cmd_topic, sub_start_mapping_cmd_topic,
          sub_stop_mapping_cmd_topic, ad_state_topic, ad_pose_topic,
          log_file_name, log_level, is_printf_to_terminal, mapping_algorithm,
          radar_start_angle, radar_end_angle, radar_resolution,
          radar_position_x, radar_position_y, radar_position_theta,
          carto_config, karto_config, location_config));
} MappingAndLocationCfg;

void MappingAndLocationConfigTransition(
    const MappingAndLocationCfg& config,
    MappingAndLocationConfig& config_result);

bool ReadMappingAndLocationConfig(const std::string& config_dir,
                                  MappingAndLocationConfig* config_result);

void DefaultWriteMappingAndLocationConfig(const std::string& config_dir);

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros