/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Autor: fuzhou
 * @LastEditors: fuzhou
 */
/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-15 15:10:01
 * @LastEditTime: 2023-03-30 03:37:40
 * @Author: lcfc-desktop
 */
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "agv_config_lib/ConfigUtils.h"
#include "common_lib/gomros.h"
#include "include/mapping_and_location/config_struct.h"
#include "include/mapping_and_location/mapping_and_location.h"

/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-21 09:39:47
 * @LastEditTime: 2023-04-05 12:36:05
 */
#pragma once

#include <string>
#include "agv_config_lib/ConfigUtils.h"
#include "mapping_and_location/config_struct.h"

// enum MappingPattern { Online, Offline };
//工程文件中不用枚举，涉及到从json格式文件（读的是int值）中读参，然后赋值给工程文件命名空间中的参数

//-----------前端----------
// 1.1 自适应体素滤波器参数
typedef struct adaptive_voxel_filter {
  float max_length;      // 0.5,
  float min_num_points;  // 200,
  float max_range;       // 50.,
  XPACK(O(max_length, min_num_points, max_range));
} adaptive_voxel_filter;
// 1.2  闭环检测的自适应体素滤波器, 用于生成稀疏点云 以进行 闭环检测
typedef struct loop_closure_adaptive_voxel_filter {
  float max_length;      // 0.9,
  float min_num_points;  // 100,
  float max_range;       // 50.,
  XPACK(O(max_length, min_num_points, max_range));
} loop_closure_adaptive_voxel_filter;

// 1.3 real_time_correlative_scan_matcher
typedef struct real_time_correlative_scan_matcher {
  double linear_search_window;   // 0.1,
  double angular_search_window;  // math.rad(20.),
  double translation_delta_cost_weight;  // 1e-1,   -- 用于计算各部分score的权重
  double rotation_delta_cost_weight;  // 1e-1,
  XPACK(O(linear_search_window, angular_search_window,
          translation_delta_cost_weight, rotation_delta_cost_weight));
} real_time_correlative_scan_matcher;
// 1.4 -- ceres匹配的一些配置参数
typedef struct ceres_scan_matcher_front {
  double occupied_space_weight;  // 1.,
  double translation_weight;     // 10.,
  double rotation_weight;        // 40.,
  // ceres_solver_options
  bool use_nonmonotonic_steps;  // false,
  int max_num_iterations;       // 20,
  int num_threads;              // 1,
  XPACK(O(occupied_space_weight, translation_weight, rotation_weight,
          use_nonmonotonic_steps, max_num_iterations, num_threads));
} ceres_scan_matcher_front;
// 1.5 为了防止子图里插入太多数据, motion_filter
typedef struct motion_filter {
  double max_time_seconds;     // 5.,
  double max_distance_meters;  // 0.2,
  double max_angle_radians;    // math.rad(1.),
  XPACK(O(max_time_seconds, max_distance_meters, max_angle_radians));
} motion_filter;
// 1.6 位姿预测器  pose_extrapolator
typedef struct pose_extrapolator {
  bool use_imu_based;  // false,
  // constant_velocity =
  double imu_gravity_time_constant;  // 10.,
  double pose_queue_duration;        // 0.001,
  // imu_based
  double pose_queue_duration_;         // 5.,
  double gravity_constant;             // 9.806,
  double pose_translation_weight;      // 1.,
  double pose_rotation_weight;         // 1.,
  double imu_acceleration_weight;      // 1.,
  double imu_rotation_weight;          // 1.,
  double odometry_translation_weight;  // 1.,
  double odometry_rotation_weight;     // 1.,
  // solver_options
  bool use_nonmonotonic_steps;  // false;
  int max_num_iterations;       // 10;
  int num_threads;              // 1  ;
  XPACK(O(use_imu_based, imu_gravity_time_constant, pose_queue_duration,
          pose_queue_duration_, gravity_constant, pose_translation_weight,
          pose_rotation_weight, imu_acceleration_weight, imu_rotation_weight,
          odometry_translation_weight, odometry_rotation_weight,
          use_nonmonotonic_steps, max_num_iterations, num_threads));
} pose_extrapolator;

// 1.7 子图(采用概率栅格地图)相关的一些配置
typedef struct submaps {
  int num_range_data;  // 90,  一个子图里插入雷达数据的个数的一半
  int grid_type;       //"PROBABILITY_GRID"
  float resolution;    // 0.05,
  // range_data_inserter =
  int range_data_inserter_type;  // "PROBABILITY_GRID_INSERTER_2D",
  // probability_grid_range_data_inserter
  bool insert_free_space;   // true,
  double hit_probability;   // 0.55,
  double miss_probability;  // 0.49,
  XPACK(O(num_range_data, grid_type, resolution, range_data_inserter_type,
          insert_free_space, hit_probability, miss_probability));
} submaps;

//------------后端--------------
typedef struct fast_correlative_scan_matcher {
  double linear_search_window;   // 7.,
  double angular_search_window;  // math.rad(30.),
  int branch_and_bound_depth;    // 7,
  XPACK(O(linear_search_window, angular_search_window, branch_and_bound_depth));
} fast_correlative_scan_matcher;

typedef struct ceres_scan_matcher_end {
  double occupied_space_weight;  // 20.,
  double translation_weight;     // 10.,
  double rotation_weight;        // 1.,
  bool use_nonmonotonic_steps;   // true,
  int max_num_iterations;        // 10,
  int num_threads;               // 1,
  XPACK(O(occupied_space_weight, translation_weight, rotation_weight,
          use_nonmonotonic_steps, max_num_iterations, num_threads));
} ceres_scan_matcher_end;
typedef struct constraint_builder {
  double sampling_ratio;                   // 0.3
  double max_constraint_distance;          // 15.,
  double min_score;                        // 0.55
  double global_localization_min_score;    // 0.6,  --
  double loop_closure_translation_weight;  // 1.1e4,
  double loop_closure_rotation_weight;     // 1e5,
  bool log_matches;                        // true,
  fast_correlative_scan_matcher fast_correlative_scan_matcher_;
  ceres_scan_matcher_end ceres_scan_matcher_end_;
  XPACK(O(sampling_ratio, max_constraint_distance, min_score,
          global_localization_min_score, loop_closure_translation_weight,
          loop_closure_rotation_weight, log_matches,
          fast_correlative_scan_matcher_, ceres_scan_matcher_end_));
} constraint_builder;

typedef struct optimization_problem {
  double huber_scale;                             // 1e1,
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
  bool use_nonmonotonic_steps;                    // false,
  int max_num_iterations;                         // 50,
  int num_threads;                                // 7,
  XPACK(O(huber_scale, local_slam_pose_translation_weight,
          local_slam_pose_rotation_weight, odometry_translation_weight,
          odometry_rotation_weight, fixed_frame_pose_translation_weight,
          fixed_frame_pose_rotation_weight, fixed_frame_pose_use_tolerant_loss,
          fixed_frame_pose_tolerant_loss_param_a,
          fixed_frame_pose_tolerant_loss_param_b, log_solver_summary,
          use_online_imu_extrinsics_in_3d, fix_z_in_3d, use_nonmonotonic_steps,
          max_num_iterations, num_threads));
} optimization_problem;

typedef struct CartoMappingConfig {
  // 前端
  int mapping_pattern;  // 工程文件中用int类型，后面进行强制类型转换
  bool use_imu_data;   // true
  bool use_odom_data;  // 加odom
  float min_range;     // 0.,
  float max_range;     // 30

  //--------------新加--------------
  int mapping_start_angle;
  int mapping_end_angle;
  float mapping_resolution;
  float laser_resolution;
  float distance_step;
  float angular_step;

  float radar_position_x;      // 0.361
  float radar_position_y;      // 0.0
  float radar_position_theta;  // 0.0

  //-----新加：轮式里程计在tracking_frame下的位姿()------
  float wheel_odom_position_x;      // 0
  float wheel_odom_position_y;      // 0
  float wheel_odom_position_theta;  // 0

  float min_z;                     // -0.8,
  float max_z;                     // 2.,
  float missing_data_ray_length;   // 5.,
  int num_accumulated_range_data;  // 1,
  float voxel_filter_size;         // 0.025,

  // 是否将GPS数据放入阻塞队列中，按时间排序再进行分发（也就是说接收的传感器数据是要先按时间进行排序的）
  bool collate_fixed_frame;        // true,
  bool use_trajectory_builder_2d;  // true
  bool collate_landmarks;          // false
  // 是否将landmarks数据放入阻塞队列中，按时间排序再进行分发
  bool use_online_correlative_scan_matching;  // false,

  adaptive_voxel_filter adaptive_voxel_filter_;
  loop_closure_adaptive_voxel_filter loop_closure_adaptive_voxel_filter_;
  real_time_correlative_scan_matcher real_time_correlative_scan_matcher_;
  ceres_scan_matcher_front ceres_scan_matcher_front_;
  motion_filter motion_filter_;
  pose_extrapolator pose_extrapolator_;
  submaps submaps_;

  // 后端
  int optimize_every_n_nodes;                      // 90,
  float matcher_translation_weight;                // 5e2,
  float matcher_rotation_weight;                   // 1.6e3,
  int max_num_final_iterations;                    // 200,
  float global_sampling_ratio;                     // 0.003,
  bool log_residual_histograms;                    // true,
  float global_constraint_search_after_n_seconds;  // 10.,
  int num_background_threads;                      // 4,
  bool collate_by_trajectory;                      // false
  constraint_builder constraint_builder_;
  optimization_problem optimization_problem_;
  std::string map_data_file_path;
  std::string map_config_file_path;
  XPACK(O(mapping_pattern, use_imu_data, use_odom_data, min_range, max_range,
          mapping_start_angle, mapping_end_angle, mapping_resolution,
          laser_resolution, distance_step, angular_step, radar_position_x,
          radar_position_y, radar_position_theta, wheel_odom_position_x,
          wheel_odom_position_y, wheel_odom_position_theta, min_z, max_z,
          missing_data_ray_length, num_accumulated_range_data,
          voxel_filter_size, collate_fixed_frame, use_trajectory_builder_2d,
          collate_landmarks, use_online_correlative_scan_matching,
          adaptive_voxel_filter_, loop_closure_adaptive_voxel_filter_,
          real_time_correlative_scan_matcher_, ceres_scan_matcher_front_,
          motion_filter_, pose_extrapolator_, submaps_, optimize_every_n_nodes,
          matcher_translation_weight, matcher_rotation_weight,
          max_num_final_iterations, global_sampling_ratio,
          log_residual_histograms, global_constraint_search_after_n_seconds,
          num_background_threads, collate_by_trajectory, constraint_builder_,
          optimization_problem_, map_data_file_path, map_config_file_path));
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
  XPACK(O(mapping_start_angle, mapping_end_angle, laser_resolution,
          radar_position_x, radar_position_y, radar_position_theta,
          use_scan_matching, scan_matching_min_distance,
          scan_matching_max_distance, occupied_space_cost_factor,
          translation_delta_cost_factor, rotation_delta_cost_factor,
          num_threads, max_num_iterations, laser_max_range, laser_min_range,
          distance_threhold, angle_threahold, resample_interval,
          laser_max_beams, odom_type, only_radar_pose, is_initial_locate,
          init_pose_file_path));
} GridLocationConfig;

typedef struct CartoAndGridConfig {
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
  CartoMappingConfig Carto_config;          // Carto算法建图配置参数
  GridLocationConfig location_config_;  // 机器人建图前的栅格定位参数配置
  XPACK(O(node_name, sub_odom_topic, sub_radar_topic, sub_imu_topic,
          sub_global_locate_cmd_topic, sub_start_mapping_cmd_topic,
          sub_stop_mapping_cmd_topic, ad_state_topic, ad_pose_topic,
          log_file_name, log_level, is_printf_to_terminal, Carto_config,
          location_config_));
} CartoAndGridConfig;

//----这个函数的内部调用了下面的Transition函数
// bool ReadMappingAndLocationConfig(
//     gomros::data_process::mapping_and_location::CartoAndGridConfig*
//         config_result) {

// }

void MappingAndLocationConfigTransition(
    const CartoAndGridConfig& config,
    gomros::data_process::mapping_and_location::CartoAndGridConfig*
        config_result) {
  using namespace gomros::data_process::mapping_and_location;

  // 独有的(当时忘记配置里程计话题参数)
  config_result->sub_radar_topic = config.sub_radar_topic;
  config_result->sub_imu_topic = config.sub_imu_topic;
  config_result->sub_odom_topic = config.sub_odom_topic;
  config_result->sub_global_locate_cmd_topic =
      config.sub_global_locate_cmd_topic;
  config_result->sub_start_mapping_cmd_topic =
      config.sub_start_mapping_cmd_topic;
  config_result->sub_stop_mapping_cmd_topic = config.sub_stop_mapping_cmd_topic;
  config_result->ad_state_topic = config.ad_state_topic;
  config_result->ad_pose_topic = config.ad_pose_topic;
  config_result->log_file_name = config.log_file_name;
  config_result->log_level = config.log_level;
  config_result->is_printf_to_terminal = config.is_printf_to_terminal;

  // 前端
  config_result->Carto_config.mapping_pattern =
      (gomros::data_process::mapping_and_location::MappingPattern)(
          config.Carto_config.mapping_pattern);

  config_result->Carto_config.use_imu_data = config.Carto_config.use_imu_data;
  config_result->Carto_config.use_odom_data = config.Carto_config.use_odom_data;
  config_result->Carto_config.min_range = config.Carto_config.min_range;
  config_result->Carto_config.max_range = config.Carto_config.max_range;

  config_result->Carto_config.mapping_start_angle =
      config.Carto_config.mapping_start_angle;
  config_result->Carto_config.mapping_end_angle =
      config.Carto_config.mapping_end_angle;
  config_result->Carto_config.mapping_resolution =
      config.Carto_config.mapping_resolution;
  config_result->Carto_config.laser_resolution =
      config.Carto_config.laser_resolution;
  config_result->Carto_config.distance_step = config.Carto_config.distance_step;
  config_result->Carto_config.angular_step = config.Carto_config.angular_step;

  config_result->Carto_config.radar_position_x =
      config.Carto_config.radar_position_x;
  config_result->Carto_config.radar_position_y =
      config.Carto_config.radar_position_y;
  config_result->Carto_config.radar_position_theta =
      config.Carto_config.radar_position_theta;

  config_result->Carto_config.wheel_odom_position_x =
      config.Carto_config.wheel_odom_position_x;
  config_result->Carto_config.wheel_odom_position_y =
      config.Carto_config.wheel_odom_position_y;
  config_result->Carto_config.wheel_odom_position_theta =
      config.Carto_config.wheel_odom_position_theta;

  config_result->Carto_config.min_z = config.Carto_config.min_z;
  config_result->Carto_config.max_z = config.Carto_config.max_z;

  config_result->Carto_config.missing_data_ray_length =
      config.Carto_config.missing_data_ray_length;
  config_result->Carto_config.num_accumulated_range_data =
      config.Carto_config.num_accumulated_range_data;
  config_result->Carto_config.voxel_filter_size =
      config.Carto_config.voxel_filter_size;
  config_result->Carto_config.use_trajectory_builder_2d =
      config.Carto_config.use_trajectory_builder_2d;

  config_result->Carto_config.collate_landmarks =
      config.Carto_config.collate_landmarks;
  config_result->Carto_config.collate_fixed_frame =
      config.Carto_config.collate_fixed_frame;
  config_result->Carto_config.use_online_correlative_scan_matching =
      config.Carto_config.use_online_correlative_scan_matching;

  // 前端嵌套的结构体
  config_result->Carto_config.adaptive_voxel_filter_.max_length =
      config.Carto_config.adaptive_voxel_filter_.max_length;
  config_result->Carto_config.adaptive_voxel_filter_.min_num_points =
      config.Carto_config.adaptive_voxel_filter_.min_num_points;
  config_result->Carto_config.adaptive_voxel_filter_.max_range =
      config.Carto_config.adaptive_voxel_filter_.max_range;

  config_result->Carto_config.loop_closure_adaptive_voxel_filter_.max_length =
      config.Carto_config.loop_closure_adaptive_voxel_filter_.max_length;
  config_result->Carto_config.loop_closure_adaptive_voxel_filter_
      .min_num_points =
      config.Carto_config.loop_closure_adaptive_voxel_filter_.min_num_points;
  config_result->Carto_config.loop_closure_adaptive_voxel_filter_.max_range =
      config.Carto_config.loop_closure_adaptive_voxel_filter_.max_range;

  config_result->Carto_config.real_time_correlative_scan_matcher_
      .linear_search_window =
      config.Carto_config.real_time_correlative_scan_matcher_
          .linear_search_window;
  config_result->Carto_config.real_time_correlative_scan_matcher_
      .angular_search_window =
      (config.Carto_config.real_time_correlative_scan_matcher_
           .angular_search_window) *
      M_PI / 180.0;
  config_result->Carto_config.real_time_correlative_scan_matcher_
      .translation_delta_cost_weight =
      config.Carto_config.real_time_correlative_scan_matcher_
          .translation_delta_cost_weight;
  config_result->Carto_config.real_time_correlative_scan_matcher_
      .rotation_delta_cost_weight =
      config.Carto_config.real_time_correlative_scan_matcher_
          .rotation_delta_cost_weight;

  config_result->Carto_config.ceres_scan_matcher_front_.occupied_space_weight =
      config.Carto_config.ceres_scan_matcher_front_.occupied_space_weight;
  config_result->Carto_config.ceres_scan_matcher_front_.translation_weight =
      config.Carto_config.ceres_scan_matcher_front_.translation_weight;
  config_result->Carto_config.ceres_scan_matcher_front_.rotation_weight =
      config.Carto_config.ceres_scan_matcher_front_.rotation_weight;
  config_result->Carto_config.ceres_scan_matcher_front_.use_nonmonotonic_steps =
      config.Carto_config.ceres_scan_matcher_front_.use_nonmonotonic_steps;
  config_result->Carto_config.ceres_scan_matcher_front_.max_num_iterations =
      config.Carto_config.ceres_scan_matcher_front_.max_num_iterations;
  config_result->Carto_config.ceres_scan_matcher_front_.num_threads =
      config.Carto_config.ceres_scan_matcher_front_.num_threads;

  config_result->Carto_config.motion_filter_.max_time_seconds =
      config.Carto_config.motion_filter_.max_time_seconds;
  config_result->Carto_config.motion_filter_.max_distance_meters =
      config.Carto_config.motion_filter_.max_distance_meters;
  config_result->Carto_config.motion_filter_.max_angle_radians =
      (config.Carto_config.motion_filter_.max_angle_radians) * M_PI / 180.0;

  config_result->Carto_config.pose_extrapolator_.use_imu_based =
      config.Carto_config.pose_extrapolator_.use_imu_based;
  config_result->Carto_config.pose_extrapolator_.imu_gravity_time_constant =
      config.Carto_config.pose_extrapolator_.imu_gravity_time_constant;
  config_result->Carto_config.pose_extrapolator_.pose_queue_duration =
      config.Carto_config.pose_extrapolator_.pose_queue_duration;
  config_result->Carto_config.pose_extrapolator_.pose_queue_duration_ =
      config.Carto_config.pose_extrapolator_.pose_queue_duration_;
  config_result->Carto_config.pose_extrapolator_.gravity_constant =
      config.Carto_config.pose_extrapolator_.gravity_constant;
  config_result->Carto_config.pose_extrapolator_.pose_translation_weight =
      config.Carto_config.pose_extrapolator_.pose_translation_weight;
  config_result->Carto_config.pose_extrapolator_.pose_rotation_weight =
      config.Carto_config.pose_extrapolator_.pose_rotation_weight;
  config_result->Carto_config.pose_extrapolator_.imu_acceleration_weight =
      config.Carto_config.pose_extrapolator_.imu_acceleration_weight;
  config_result->Carto_config.pose_extrapolator_.imu_rotation_weight =
      config.Carto_config.pose_extrapolator_.imu_rotation_weight;
  config_result->Carto_config.pose_extrapolator_.odometry_translation_weight =
      config.Carto_config.pose_extrapolator_.odometry_translation_weight;
  config_result->Carto_config.pose_extrapolator_.odometry_rotation_weight =
      config.Carto_config.pose_extrapolator_.odometry_rotation_weight;
  config_result->Carto_config.pose_extrapolator_.use_nonmonotonic_steps =
      config.Carto_config.pose_extrapolator_.use_nonmonotonic_steps;
  config_result->Carto_config.pose_extrapolator_.max_num_iterations =
      config.Carto_config.pose_extrapolator_.max_num_iterations;
  config_result->Carto_config.pose_extrapolator_.num_threads =
      config.Carto_config.pose_extrapolator_.num_threads;

  config_result->Carto_config.submaps_.num_range_data =
      config.Carto_config.submaps_.num_range_data;
  config_result->Carto_config.submaps_.grid_type =
      (gomros::data_process::mapping_and_location::GridOptions2D_GridType)(
          config.Carto_config.submaps_.grid_type);
  config_result->Carto_config.submaps_.resolution =
      config.Carto_config.submaps_.resolution;
  config_result->Carto_config.submaps_.range_data_inserter_type =
      (gomros::data_process::mapping_and_location::
           RangeDataInserterOptions_RangeDataInserterType)(
          config.Carto_config.submaps_.range_data_inserter_type);
  config_result->Carto_config.submaps_.insert_free_space =
      config.Carto_config.submaps_.insert_free_space;
  config_result->Carto_config.submaps_.hit_probability =
      config.Carto_config.submaps_.hit_probability;
  config_result->Carto_config.submaps_.miss_probability =
      config.Carto_config.submaps_.miss_probability;

  // 后端
  config_result->Carto_config.optimize_every_n_nodes =
      config.Carto_config.optimize_every_n_nodes;
  config_result->Carto_config.matcher_translation_weight =
      config.Carto_config.matcher_translation_weight;
  config_result->Carto_config.matcher_rotation_weight =
      config.Carto_config.matcher_rotation_weight;
  config_result->Carto_config.max_num_final_iterations =
      config.Carto_config.max_num_final_iterations;
  config_result->Carto_config.global_sampling_ratio =
      config.Carto_config.global_sampling_ratio;
  config_result->Carto_config.log_residual_histograms =
      config.Carto_config.log_residual_histograms;
  config_result->Carto_config.global_constraint_search_after_n_seconds =
      config.Carto_config.global_constraint_search_after_n_seconds;
  config_result->Carto_config.num_background_threads =
      config.Carto_config.num_background_threads;
  config_result->Carto_config.collate_by_trajectory =
      config.Carto_config.collate_by_trajectory;

  config_result->Carto_config.constraint_builder_.sampling_ratio =
      config.Carto_config.constraint_builder_.sampling_ratio;
  config_result->Carto_config.constraint_builder_.max_constraint_distance =
      config.Carto_config.constraint_builder_.min_score;
  config_result->Carto_config.constraint_builder_.min_score =
      config.Carto_config.constraint_builder_.min_score;
  config_result->Carto_config.constraint_builder_
      .global_localization_min_score =
      config.Carto_config.constraint_builder_.global_localization_min_score;
  config_result->Carto_config.constraint_builder_
      .loop_closure_translation_weight =
      config.Carto_config.constraint_builder_.loop_closure_translation_weight;
  config_result->Carto_config.constraint_builder_.loop_closure_rotation_weight =
      config.Carto_config.constraint_builder_.loop_closure_rotation_weight;
  config_result->Carto_config.constraint_builder_.log_matches =
      config.Carto_config.constraint_builder_.log_matches;

  config_result->Carto_config.constraint_builder_.fast_correlative_scan_matcher_
      .linear_search_window =
      config.Carto_config.constraint_builder_.fast_correlative_scan_matcher_
          .linear_search_window;
  config_result->Carto_config.constraint_builder_.fast_correlative_scan_matcher_
      .angular_search_window =
      (config.Carto_config.constraint_builder_.fast_correlative_scan_matcher_
           .angular_search_window) *
      M_PI / 180.0;
  config_result->Carto_config.constraint_builder_.fast_correlative_scan_matcher_
      .branch_and_bound_depth =
      config.Carto_config.constraint_builder_.fast_correlative_scan_matcher_
          .branch_and_bound_depth;

  config_result->Carto_config.constraint_builder_.ceres_scan_matcher_end_
      .occupied_space_weight =
      config.Carto_config.constraint_builder_.ceres_scan_matcher_end_
          .occupied_space_weight;
  config_result->Carto_config.constraint_builder_.ceres_scan_matcher_end_
      .translation_weight = config.Carto_config.constraint_builder_
                                .ceres_scan_matcher_end_.translation_weight;
  config_result->Carto_config.constraint_builder_.ceres_scan_matcher_end_
      .rotation_weight = config.Carto_config.constraint_builder_
                             .ceres_scan_matcher_end_.rotation_weight;
  config_result->Carto_config.constraint_builder_.ceres_scan_matcher_end_
      .use_nonmonotonic_steps =
      config.Carto_config.constraint_builder_.ceres_scan_matcher_end_
          .use_nonmonotonic_steps;
  config_result->Carto_config.constraint_builder_.ceres_scan_matcher_end_
      .max_num_iterations = config.Carto_config.constraint_builder_
                                .ceres_scan_matcher_end_.max_num_iterations;
  config_result->Carto_config.constraint_builder_.ceres_scan_matcher_end_
      .num_threads = config.Carto_config.constraint_builder_
                         .ceres_scan_matcher_end_.num_threads;

  config_result->Carto_config.optimization_problem_.huber_scale =
      config.Carto_config.optimization_problem_.huber_scale;
  config_result->Carto_config.optimization_problem_
      .local_slam_pose_translation_weight =
      config.Carto_config.optimization_problem_
          .local_slam_pose_translation_weight;
  config_result->Carto_config.optimization_problem_
      .local_slam_pose_rotation_weight =
      config.Carto_config.optimization_problem_.local_slam_pose_rotation_weight;
  config_result->Carto_config.optimization_problem_
      .odometry_translation_weight =
      config.Carto_config.optimization_problem_.odometry_translation_weight;
  config_result->Carto_config.optimization_problem_.odometry_rotation_weight =
      config.Carto_config.optimization_problem_.odometry_rotation_weight;
  config_result->Carto_config.optimization_problem_
      .fixed_frame_pose_translation_weight =
      config.Carto_config.optimization_problem_
          .fixed_frame_pose_translation_weight;
  config_result->Carto_config.optimization_problem_
      .fixed_frame_pose_rotation_weight =
      config.Carto_config.optimization_problem_
          .fixed_frame_pose_rotation_weight;
  config_result->Carto_config.optimization_problem_
      .fixed_frame_pose_use_tolerant_loss =
      config.Carto_config.optimization_problem_
          .fixed_frame_pose_use_tolerant_loss;
  config_result->Carto_config.optimization_problem_
      .fixed_frame_pose_tolerant_loss_param_a =
      config.Carto_config.optimization_problem_
          .fixed_frame_pose_tolerant_loss_param_a;
  config_result->Carto_config.optimization_problem_
      .fixed_frame_pose_tolerant_loss_param_b =
      config.Carto_config.optimization_problem_
          .fixed_frame_pose_tolerant_loss_param_b;
  config_result->Carto_config.optimization_problem_.log_solver_summary =
      config.Carto_config.optimization_problem_.log_solver_summary;
  config_result->Carto_config.optimization_problem_
      .use_online_imu_extrinsics_in_3d =
      config.Carto_config.optimization_problem_.use_online_imu_extrinsics_in_3d;
  config_result->Carto_config.optimization_problem_.fix_z_in_3d =
      config.Carto_config.optimization_problem_.fix_z_in_3d;
  config_result->Carto_config.optimization_problem_.use_nonmonotonic_steps =
      config.Carto_config.optimization_problem_.use_nonmonotonic_steps;
  config_result->Carto_config.optimization_problem_.max_num_iterations =
      config.Carto_config.optimization_problem_.max_num_iterations;
  config_result->Carto_config.optimization_problem_.num_threads =
      config.Carto_config.optimization_problem_.num_threads;

  config_result->Carto_config.map_data_file_path =
      config.Carto_config.map_data_file_path;
  config_result->Carto_config.map_config_file_path =
      config.Carto_config.map_config_file_path;

  // 定位
  config_result->location_config_.mapping_start_angle =
      config.location_config_.mapping_start_angle;
  config_result->location_config_.mapping_end_angle =
      config.location_config_.mapping_end_angle;
  config_result->location_config_.laser_resolution =
      config.location_config_.laser_resolution;
  config_result->location_config_.radar_position_x =
      config.location_config_.radar_position_x;
  config_result->location_config_.radar_position_y =
      config.location_config_.radar_position_y;
  config_result->location_config_.radar_position_theta =
      config.location_config_.radar_position_theta;
  config_result->location_config_.use_scan_matching =
      config.location_config_.use_scan_matching;
  config_result->location_config_.scan_matching_min_distance =
      config.location_config_.scan_matching_min_distance;
  config_result->location_config_.scan_matching_max_distance =
      config.location_config_.scan_matching_max_distance;
  config_result->location_config_.occupied_space_cost_factor =
      config.location_config_.occupied_space_cost_factor;
  config_result->location_config_.translation_delta_cost_factor =
      config.location_config_.translation_delta_cost_factor;
  config_result->location_config_.rotation_delta_cost_factor =
      config.location_config_.rotation_delta_cost_factor;
  config_result->location_config_.num_threads =
      config.location_config_.num_threads;
  config_result->location_config_.max_num_iterations =
      config.location_config_.max_num_iterations;
  config_result->location_config_.laser_max_range =
      config.location_config_.laser_max_range;
  config_result->location_config_.laser_min_range =
      config.location_config_.laser_min_range;
  config_result->location_config_.distance_threhold =
      config.location_config_.distance_threhold;
  config_result->location_config_.angle_threahold =
      config.location_config_.angle_threahold;
  config_result->location_config_.resample_interval =
      config.location_config_.resample_interval;
  config_result->location_config_.laser_max_beams =
      config.location_config_.laser_max_beams;
  config_result->location_config_.odom_type = config.location_config_.odom_type;
  config_result->location_config_.only_radar_pose =
      config.location_config_.only_radar_pose;
  config_result->location_config_.is_initial_locate =
      config.location_config_.is_initial_locate;
  config_result->location_config_.init_pose_file_path =
      config.location_config_.init_pose_file_path;

  return;
}
// void DefaultWriteMappingAndLocationConfig(void) {

// }
bool ReadMappingAndLocationConfig(
    gomros::data_process::mapping_and_location::CartoAndGridConfig*
        config_result) {
  std::string mapping_and_location_config_dir =
      "./config/mappingandlocation.json";
  CartoAndGridConfig temp_config;
  if (ConfigUtils::decode(mapping_and_location_config_dir.c_str(),
                          temp_config)) {
    MappingAndLocationConfigTransition(temp_config, config_result);
    return true;
  }
  return false;
}

TEST(mapping_location_test, mapping_test) { 
  gomros::common::InitMaster(1); 
  using namespace  gomros::data_process::mapping_and_location;
  gomros::data_process::mapping_and_location::CartoAndGridConfig config;
  if (ReadMappingAndLocationConfig(&config)) {
    MappingAndLocation* p = new MappingAndLocation(config);
    p->StopMapping("carto");
  }
  while(true) {
    usleep(1);
  }
  delete p;
}
