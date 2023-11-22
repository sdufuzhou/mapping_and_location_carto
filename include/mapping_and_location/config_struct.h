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

/**
 * @brief Karto建图算法配置参数
 *
 */
struct KartoMappingConfig {
  MappingPattern mapping_pattern;  // 建图方式
  float distance_step;             // 0.1
  float angular_step;              // 0.1
  float mapping_start_angle;       // -95.0
  float mapping_end_angle;         // 95.0
  float laser_resolution;          // 0.5
  float mapping_resolution;        // 0.02

  float radar_position_x;      // 0.361
  float radar_position_y;      // 0.0
  float radar_position_theta;  // 0.0

  float mapping_laser_min_range;  // 0.5
  float mapping_laser_max_range;  // 19.0
  float travel_distance;  // 设置最小距离，里程计移动长度超过此距离，则建立一个节点,0.2
  float travel_angle;  // 设置最小偏转角，如果里程计转向超过此值，则建立一个节点,0.175
  float loop_search_distance;  // 搜寻回环匹配的最大距离,4.0
  std::string map_data_file_path;
  std::string map_config_file_path;
};

/**
 * @brief 栅格定位配置参数
 *
 */
struct GridLocationConfig {
  float mapping_start_angle;   // -95.0
  float mapping_end_angle;     // 95.0
  float laser_resolution;      // 0.5
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
};

/**
 * @brief karto建图和栅格定位配置参数
 *
 */
struct KartoAndGridConfig {
  std::string node_name;
  std::string sub_odom_topic;               // 订阅里程计话题
  std::string sub_radar_topic;              // 订阅雷达话题
  std::string sub_global_locate_cmd_topic;  // 订阅全局初始化命令
  std::string sub_start_mapping_cmd_topic;  // 订阅开始建图命令
  std::string sub_stop_mapping_cmd_topic;   // 订阅停止建图命令
  std::string ad_state_topic;               // 更改状态发布的话题名
  std::string ad_pose_topic;                // 位姿发布的话题名
  std::string log_file_name;                // 日志文件名
  int log_level;                            // 日志等级
  bool is_printf_to_terminal;               // 是否将日志输出到终端
  KartoMappingConfig mapping_config;        // karto建图算法配置参数
  GridLocationConfig location_config;       // 栅格定位配置参数
};
/**
 * @brief Cartographer算法前端配置参数
 *
 */
// 1.1 自适应体素滤波器参数
struct adaptive_voxel_filter {
  float max_length;      // 0.5,
  float min_num_points;  // 200,
  float max_range;       // 50.,
};
// 1.2  闭环检测的自适应体素滤波器, 用于生成稀疏点云 以进行 闭环检测
struct loop_closure_adaptive_voxel_filter {
  float max_length;      // 0.9,
  float min_num_points;  // 100,
  float max_range;       // 50.,
};

// 1.3 real_time_correlative_scan_matcher
struct real_time_correlative_scan_matcher {
  double linear_search_window;  // 0.1,             -- 线性搜索窗口的大小
  double
      angular_search_window;  // math.rad(20.),  --
                              // 角度搜索窗口的大小，比如机器人的姿态搜索范围[-10度-10度]
  double translation_delta_cost_weight;  // 1e-1,   -- 用于计算各部分score的权重
  double rotation_delta_cost_weight;  // 1e-1,
};
// 1.4 -- ceres匹配的一些配置参数
struct ceres_scan_matcher_front {
  double occupied_space_weight;  // 1.,
  double translation_weight;     // 10.,
  double rotation_weight;        // 40.,
  // ceres_solver_options
  bool use_nonmonotonic_steps;  // false,
  int max_num_iterations;       // 20,
  int num_threads;              // 1,
};
// 1.5 为了防止子图里插入太多数据, motion_filter
struct motion_filter {
  double max_time_seconds;     // 5.,
  double max_distance_meters;  // 0.2,
  double max_angle_radians;    // math.rad(1.),
};
// 1.6 位姿预测器  pose_extrapolator
struct pose_extrapolator {
  bool use_imu_based;  // false,
  // constant_velocity =
  double imu_gravity_time_constant;  // 10.,
  double pose_queue_duration;        // 0.001,
  // imu_based
  double pose_queue_duration_;          // 5.,
  double gravity_constant;             // 9.806,
  double pose_translation_weight;      // 1.,
  double pose_rotation_weight;         // 1.,
  double imu_acceleration_weight;      // 1.,
  double imu_rotation_weight;          // 1.,
  double odometry_translation_weight;  // 1.,
  double odometry_rotation_weight;     // 1.,+
  // solver_options
  bool use_nonmonotonic_steps;  // false;
  int max_num_iterations;       // 10;
  int num_threads;              // 1;
};
//用户输入int值，用强制类型转换
enum GridOptions2D_GridType {
  GridOptions2D_GridType_INVALID_GRID = 0,
  GridOptions2D_GridType_PROBABILITY_GRID = 1,
  GridOptions2D_GridType_TSDF = 2,
};

enum RangeDataInserterOptions_RangeDataInserterType {
  RangeDataInserterOptions_RangeDataInserterType_INVALID_INSERTER = 0,
  RangeDataInserterOptions_RangeDataInserterType_PROBABILITY_GRID_INSERTER_2D =
      1,
  RangeDataInserterOptions_RangeDataInserterType_TSDF_INSERTER_2D = 2,
};
// 1.7 子图(采用概率栅格地图)相关的一些配置
struct submaps {
  int num_range_data;  // 90,  一个子图里插入雷达数据的个数的一半
  GridOptions2D_GridType grid_type;  //"PROBABILITY_GRID"
  float resolution;                  // 0.05,

  // range_data_inserter =
  RangeDataInserterOptions_RangeDataInserterType
      range_data_inserter_type;  // "PROBABILITY_GRID_INSERTER_2D",
  // probability_grid_range_data_inserter
  bool insert_free_space;   // true,
  double hit_probability;   // 0.55,
  double miss_probability;  // 0.49,
};
/**
 * @brief Cartographer算法后端配置参数
 *
 */
// 基于分支定界算法的2d粗匹配器
struct fast_correlative_scan_matcher {
  double linear_search_window;   // 7.,
  double angular_search_window;  // math.rad(30.),
  int branch_and_bound_depth;    // 7,
};
// 基于ceres的2d精匹配器
struct ceres_scan_matcher_end {
  double occupied_space_weight;  // 20.,
  double translation_weight;     // 10.,
  double rotation_weight;        // 1.,
  // ceres_solver_options =
  bool use_nonmonotonic_steps;  // true,--允许局部增大，然后再降低到全局最低点
  int max_num_iterations;  // 10,--最大的迭代次数
  int num_threads;         // 1,
};
// 约束构建的相关参数
struct constraint_builder {
  double sampling_ratio;  // 0.3,
                          // 对局部子图进行回环检测时的计算频率, 数值越大,
                          // 计算次数越多
  double
      max_constraint_distance;  // 15.,
                                // 对局部子图进行回环检测时能成为约束的最大距离
                                // 0.55
                                // 对局部子图进行回环检测时的最低分数阈值
  double min_score;
  double global_localization_min_score;  // 0.6,  --
                                      // 对整体子图进行回环检测时的最低分数阈值
  double loop_closure_translation_weight;  // 1.1e4,--平移权重
  double loop_closure_rotation_weight;     // 1e5,--旋转权重
  bool log_matches;  // true,                   -- 打印约束计算的log
  fast_correlative_scan_matcher fast_correlative_scan_matcher_;
  ceres_scan_matcher_end ceres_scan_matcher_end_;
};
struct optimization_problem {
  double huber_scale;  // 1e1,         -- 值越大,（潜在）异常值的影响就越大

  // acceleration_weight = 1.1e2,      -- 3d里imu的线加速度的权重
  // rotation_weight = 1.6e4,          -- 3d里imu的旋转的权重

  // 前端结果残差的权重
  double local_slam_pose_translation_weight;  // 1e5,
  double local_slam_pose_rotation_weight;     // 1e5,

  // 里程计残差的权重
  double odometry_translation_weight;  // 1e5,
  double odometry_rotation_weight;     // 1e5,

  // gps残差的权重
  double fixed_frame_pose_translation_weight;     // 1e1,
  double fixed_frame_pose_rotation_weight;        // 1e2,
  bool fixed_frame_pose_use_tolerant_loss;        // false,
  double fixed_frame_pose_tolerant_loss_param_a;  // 1,
  double fixed_frame_pose_tolerant_loss_param_b;  // 1,

  bool log_solver_summary;               // false,
  bool use_online_imu_extrinsics_in_3d;  // true,
  bool fix_z_in_3d;                      // false,

  // ceres_solver_options =
  bool use_nonmonotonic_steps;  // false,
  int max_num_iterations;       // 50,
  int num_threads;              // 7,
};

struct CartoMappingConfig {
  MappingPattern mapping_pattern;  // 建图方式
  // 前端
  bool use_imu_data;  // true
  // 加odom
  bool use_odom_data;

  float min_range;  // 0.,
  float max_range;  // 30

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
  float wheel_odom_position_x;      // 待确定
  float wheel_odom_position_y;      //
  float wheel_odom_position_theta;  //

  float min_z;                     // -0.8,
  float max_z;                     // 2.,
  float missing_data_ray_length;   // 5.,
  int num_accumulated_range_data;  // 1,
  float voxel_filter_size;         // 0.025,
  // 是否将GPS数据放入阻塞队列中，按时间排序再进行分发（也就是说接收的传感器数据是要先按时间进行排序的）
  bool collate_fixed_frame;  // true,
  bool use_trajectory_builder_2d;
  bool collate_landmarks;
  // 是否将landmarks数据放入阻塞队列中，按时间排序再进行分发
  bool use_online_correlative_scan_matching;  // false,

  //前面声明的是结构体，这里进行实例化，通过变量去调用结构体里的变量
  adaptive_voxel_filter adaptive_voxel_filter_;
  loop_closure_adaptive_voxel_filter loop_closure_adaptive_voxel_filter_;
  real_time_correlative_scan_matcher real_time_correlative_scan_matcher_;
  ceres_scan_matcher_front ceres_scan_matcher_front_;
  motion_filter motion_filter_;
  pose_extrapolator pose_extrapolator_;
  submaps submaps_;

  // 后端
  int optimize_every_n_nodes;  // 90,  -- 每隔多少个节点执行一次后端优化
  float matcher_translation_weight;  // 5e2,
  float matcher_rotation_weight;     // 1.6e3,
  int max_num_final_iterations;  // 200,   -- 在建图结束之后执行一次全局优化,
                                 // 不要求实时性, 迭代次数多
  float global_sampling_ratio;  // 0.003,    -- 纯定位时候查找回环的频率
  bool log_residual_histograms;  // true,
  // 纯定位时多少秒执行一次全子图的约束计算
  float global_constraint_search_after_n_seconds;  // 10.,

  int num_background_threads;  // 4,
  bool collate_by_trajectory;  // false
//结构体包含结构体，某个结构体的变量作为另一个结构体的成员
  constraint_builder constraint_builder_;
  optimization_problem optimization_problem_;

  //------------新加------------
  std::string map_data_file_path;
  std::string map_config_file_path;
};  // struct CartoMappingConfig

// 新加
struct CartoAndGridConfig {
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
};
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
