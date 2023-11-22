#include "include/mapping_lib/carto_mapping/create_options.h"
#include <iostream>
namespace gomros {
namespace data_process {
namespace mapping_and_location {

//地图创建参数配置具体过程
cartographer::mapping::proto::MapBuilderOptions GetMapBuilderOptions(
    const CartoMappingConfig &m_MappingConfig) {

  cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  
  map_builder_options.set_use_trajectory_builder_2d(
      m_MappingConfig.use_trajectory_builder_2d);
  map_builder_options.set_num_background_threads(
      m_MappingConfig.num_background_threads);
  map_builder_options.set_collate_by_trajectory(
      m_MappingConfig.collate_by_trajectory);
  // 该函数返回pose_graph_options_是一个指针，类型为PoseGraphOptions
  PoseGraphOptions *pose_graph_options =
      map_builder_options.mutable_pose_graph_options();
  // PoseGraphOptions *p还有一些单独的变量需要赋值
  pose_graph_options->set_optimize_every_n_nodes(
      m_MappingConfig.optimize_every_n_nodes);
  pose_graph_options->set_matcher_translation_weight(
      m_MappingConfig.matcher_translation_weight);
  pose_graph_options->set_matcher_rotation_weight(
      m_MappingConfig.matcher_rotation_weight);
  pose_graph_options->set_max_num_final_iterations(
      m_MappingConfig.max_num_final_iterations);
  pose_graph_options->set_global_sampling_ratio(
      m_MappingConfig.global_sampling_ratio);
  pose_graph_options->set_log_residual_histograms(
      m_MappingConfig.log_residual_histograms);
  pose_graph_options->set_global_constraint_search_after_n_seconds(
      m_MappingConfig.global_constraint_search_after_n_seconds);

  GetConstraintBuilderOptions(pose_graph_options, m_MappingConfig);
  GetOptimizationProblemOptions(pose_graph_options, m_MappingConfig);
  // map_builder_options.set_allocated_pose_graph_options(pose_graph_options);
  return map_builder_options;
}

//约束构建的相关参数
void GetConstraintBuilderOptions(PoseGraphOptions *pose_graph_options,
                                 const CartoMappingConfig &m_MappingConfig) {
  // 约束构建的相关参数
  ::cartographer::mapping::constraints::proto::ConstraintBuilderOptions
      *constraint_builder_options =
          pose_graph_options->mutable_constraint_builder_options();
  constraint_builder_options->set_sampling_ratio(
      m_MappingConfig.constraint_builder_.sampling_ratio);
  constraint_builder_options->set_max_constraint_distance(
      m_MappingConfig.constraint_builder_.max_constraint_distance);
  constraint_builder_options->set_min_score(
      m_MappingConfig.constraint_builder_.min_score);
  constraint_builder_options->set_global_localization_min_score(
      m_MappingConfig.constraint_builder_.global_localization_min_score);
  constraint_builder_options->set_loop_closure_translation_weight(
      m_MappingConfig.constraint_builder_.loop_closure_translation_weight);
  constraint_builder_options->set_loop_closure_rotation_weight(
      m_MappingConfig.constraint_builder_.loop_closure_rotation_weight);
  constraint_builder_options->set_log_matches(
      m_MappingConfig.constraint_builder_.log_matches);  //-- 打印约束计算的log
  // 基于分支定界算法的2d粗匹配器
  ::cartographer::mapping::scan_matching::proto::
      FastCorrelativeScanMatcherOptions2D *fast_correlative_scan_matcher =
          constraint_builder_options
              ->mutable_fast_correlative_scan_matcher_options();
  fast_correlative_scan_matcher->set_linear_search_window(
      m_MappingConfig.constraint_builder_.fast_correlative_scan_matcher_
          .linear_search_window);
  fast_correlative_scan_matcher->set_angular_search_window(
      m_MappingConfig.constraint_builder_.fast_correlative_scan_matcher_
          .angular_search_window);
  fast_correlative_scan_matcher->set_branch_and_bound_depth(
      m_MappingConfig.constraint_builder_.fast_correlative_scan_matcher_
          .branch_and_bound_depth);
  // 基于ceres的2d精匹配器
  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
      *ceres_scan_matcher =
          constraint_builder_options->mutable_ceres_scan_matcher_options();
  ceres_scan_matcher->set_occupied_space_weight(
      m_MappingConfig.constraint_builder_.ceres_scan_matcher_end_
          .occupied_space_weight);
  ceres_scan_matcher->set_translation_weight(
      m_MappingConfig.constraint_builder_.ceres_scan_matcher_end_
          .translation_weight);
  ceres_scan_matcher->set_rotation_weight(
      m_MappingConfig.constraint_builder_.ceres_scan_matcher_end_
          .rotation_weight);
  ::cartographer::common::proto::CeresSolverOptions *ceres_solver_options =
      ceres_scan_matcher->mutable_ceres_solver_options();
  ceres_solver_options->set_use_nonmonotonic_steps(
      m_MappingConfig.constraint_builder_.ceres_scan_matcher_end_
          .use_nonmonotonic_steps);
  ceres_solver_options->set_max_num_iterations(
      m_MappingConfig.constraint_builder_.ceres_scan_matcher_end_
          .max_num_iterations);
  ceres_solver_options->set_num_threads(
      m_MappingConfig.constraint_builder_.ceres_scan_matcher_end_.num_threads);
  //   ceres_scan_matcher->set_allocated_ceres_solver_options(ceres_solver_options);
  //   constraint_builder_options->set_allocated_ceres_scan_matcher_options(
  //       ceres_scan_matcher);
  //   constraint_builder_options
  //       ->set_allocated_fast_correlative_scan_matcher_options(
  //           fast_correlative_scan_matcher);
  //   pose_graph_options->set_allocated_constraint_builder_options(
  //       constraint_builder_options);
}

// 优化残差方程的相关参数

void GetOptimizationProblemOptions(PoseGraphOptions *pose_graph_options,
                                   const CartoMappingConfig &m_MappingConfig) {
  // 通过调用函数返回对象
  ::cartographer::mapping::optimization::proto::OptimizationProblemOptions
      *optimization_problem_options =
          pose_graph_options->mutable_optimization_problem_options();
  optimization_problem_options->set_huber_scale(
      m_MappingConfig.optimization_problem_.huber_scale);
  // 前端结果残差的权重
  optimization_problem_options->set_local_slam_pose_translation_weight(
      m_MappingConfig.optimization_problem_.local_slam_pose_translation_weight);
  optimization_problem_options->set_local_slam_pose_rotation_weight(
      m_MappingConfig.optimization_problem_.local_slam_pose_rotation_weight);
  // 里程计残差的权重
  optimization_problem_options->set_odometry_translation_weight(
      m_MappingConfig.optimization_problem_.odometry_translation_weight);
  optimization_problem_options->set_odometry_rotation_weight(
      m_MappingConfig.optimization_problem_.odometry_rotation_weight);
  // gps残差的权重
  optimization_problem_options->set_fixed_frame_pose_translation_weight(
      m_MappingConfig.optimization_problem_
          .fixed_frame_pose_translation_weight);
  optimization_problem_options->set_fixed_frame_pose_rotation_weight(
      m_MappingConfig.optimization_problem_.fixed_frame_pose_rotation_weight);
  optimization_problem_options->set_fixed_frame_pose_use_tolerant_loss(
      m_MappingConfig.optimization_problem_.fixed_frame_pose_use_tolerant_loss);
  optimization_problem_options->set_fixed_frame_pose_tolerant_loss_param_a(
      m_MappingConfig.optimization_problem_
          .fixed_frame_pose_tolerant_loss_param_a);
  optimization_problem_options->set_fixed_frame_pose_tolerant_loss_param_b(
      m_MappingConfig.optimization_problem_
          .fixed_frame_pose_tolerant_loss_param_b);

  optimization_problem_options->set_log_solver_summary(
      m_MappingConfig.optimization_problem_.log_solver_summary);
  optimization_problem_options->set_use_online_imu_extrinsics_in_3d(
      m_MappingConfig.optimization_problem_.use_online_imu_extrinsics_in_3d);
  optimization_problem_options->set_fix_z_in_3d(
      m_MappingConfig.optimization_problem_.fix_z_in_3d);

  ::cartographer::common::proto::CeresSolverOptions *ceres_solver_options =
      optimization_problem_options->mutable_ceres_solver_options();
  ceres_solver_options->set_use_nonmonotonic_steps(
      m_MappingConfig.optimization_problem_.use_nonmonotonic_steps);
  ceres_solver_options->set_max_num_iterations(
      m_MappingConfig.optimization_problem_.max_num_iterations);
  ceres_solver_options->set_num_threads(
      m_MappingConfig.optimization_problem_.num_threads);
  //   optimization_problem_options->set_allocated_ceres_solver_options(
  //       ceres_solver_options);
  //   pose_graph_options->set_allocated_optimization_problem_options(
  //       optimization_problem_options);
}

//前端
cartographer::mapping::proto::TrajectoryBuilderOptions
GetTrajectoryBuilderOptions(const CartoMappingConfig &m_MappingConfig) {
  // 使用智能指针创建 TrajectoryBuilderOptions 对象
  cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  trajectory_builder_options.set_collate_fixed_frame(
      m_MappingConfig.collate_fixed_frame);
  trajectory_builder_options.set_collate_landmarks(
      m_MappingConfig.collate_landmarks);
  LocalTrajectoryBuilderOptions2D *p =
      trajectory_builder_options.mutable_trajectory_builder_2d_options();

  // LocalTrajectoryBuilderOptions2D *p还有一些单独的变量需要赋值
  p->set_use_imu_data(m_MappingConfig.use_imu_data);
  p->set_min_range(m_MappingConfig.min_range);
  p->set_max_range(m_MappingConfig.max_range);
  p->set_min_z(m_MappingConfig.min_z);
  p->set_max_z(m_MappingConfig.max_z);
  p->set_missing_data_ray_length(m_MappingConfig.missing_data_ray_length);
  p->set_num_accumulated_range_data(m_MappingConfig.num_accumulated_range_data);
  p->set_voxel_filter_size(m_MappingConfig.voxel_filter_size);

  GetAdaptiveVoxelFilterOptions(p, m_MappingConfig);
  GetLoopClosureAdaptiveVoxelFilterOptions(p, m_MappingConfig);
  GetRtCsmOptions(p, m_MappingConfig);
  GetCeresScanMatcherOptions(p, m_MappingConfig);
  GetMotionFilterOptions(p, m_MappingConfig);
  GetPoseExp(p, m_MappingConfig);
  GetSubMapOptions(p, m_MappingConfig);
  //   trajectory_builder_options.set_allocated_trajectory_builder_2d_options(p);
  return trajectory_builder_options;
}
//如果想拿到CartoMappingConfig里的参数，通过函数，必须要作为参数之一传进去----------
void GetAdaptiveVoxelFilterOptions(LocalTrajectoryBuilderOptions2D *p,
                                   const CartoMappingConfig &m_MappingConfig) {
  // 1.1 自适应体素滤波器参数配置
  // 通过调用函数返回对象
  ::cartographer::sensor::proto::AdaptiveVoxelFilterOptions *p1 =
      p->mutable_adaptive_voxel_filter_options();
  p1->set_min_num_points(m_MappingConfig.adaptive_voxel_filter_.min_num_points);
  p1->set_max_length(m_MappingConfig.adaptive_voxel_filter_.max_length);
  p1->set_max_range(m_MappingConfig.adaptive_voxel_filter_.max_range);
  //   p->set_allocated_adaptive_voxel_filter_options(p1);
}
void GetLoopClosureAdaptiveVoxelFilterOptions(
    LocalTrajectoryBuilderOptions2D *p,
    const CartoMappingConfig &m_MappingConfig) {
  // 1.2闭环检测的自适应体素滤波器
  ::cartographer::sensor::proto::AdaptiveVoxelFilterOptions *p1 =
      p->mutable_loop_closure_adaptive_voxel_filter_options();
  p1->set_min_num_points(
      m_MappingConfig.loop_closure_adaptive_voxel_filter_.min_num_points);
  p1->set_max_length(
      m_MappingConfig.loop_closure_adaptive_voxel_filter_.max_length);
  p1->set_max_range(
      m_MappingConfig.loop_closure_adaptive_voxel_filter_.max_range);
  //   p->set_allocated_loop_closure_adaptive_voxel_filter_options(p1);
}
void GetRtCsmOptions(LocalTrajectoryBuilderOptions2D *p,
                     const CartoMappingConfig &m_MappingConfig) {
  // 1.3 real_time_correlative_scan_matcher_
  p->set_use_online_correlative_scan_matching(
      m_MappingConfig.use_online_correlative_scan_matching);
  ::cartographer::mapping::scan_matching::proto::
      RealTimeCorrelativeScanMatcherOptions *p1 =
          p->mutable_real_time_correlative_scan_matcher_options();
  p1->set_linear_search_window(
      m_MappingConfig.real_time_correlative_scan_matcher_.linear_search_window);
  p1->set_angular_search_window(
      m_MappingConfig.real_time_correlative_scan_matcher_
          .angular_search_window);
  p1->set_translation_delta_cost_weight(
      m_MappingConfig.real_time_correlative_scan_matcher_
          .translation_delta_cost_weight);
  p1->set_rotation_delta_cost_weight(
      m_MappingConfig.real_time_correlative_scan_matcher_
          .rotation_delta_cost_weight);
  //   p->set_allocated_real_time_correlative_scan_matcher_options(p1);
}
void GetCeresScanMatcherOptions(LocalTrajectoryBuilderOptions2D *p,
                                const CartoMappingConfig &m_MappingConfig) {
  // 1.4 -- ceres匹配的一些配置参数
  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D *p1 =
      p->mutable_ceres_scan_matcher_options();
  p1->set_occupied_space_weight(
      m_MappingConfig.ceres_scan_matcher_front_.occupied_space_weight);
  p1->set_translation_weight(
      m_MappingConfig.ceres_scan_matcher_front_.translation_weight);
  p1->set_rotation_weight(
      m_MappingConfig.ceres_scan_matcher_front_.rotation_weight);
  ::cartographer::common::proto::CeresSolverOptions *ceres_solver =
      p1->mutable_ceres_solver_options();
  ceres_solver->set_use_nonmonotonic_steps(
      m_MappingConfig.ceres_scan_matcher_front_.use_nonmonotonic_steps);
  ceres_solver->set_max_num_iterations(
      m_MappingConfig.ceres_scan_matcher_front_.max_num_iterations);
  ceres_solver->set_num_threads(
      m_MappingConfig.ceres_scan_matcher_front_.num_threads);
  //   p1->set_allocated_ceres_solver_options(ceres_solver);
  //   p->set_allocated_ceres_scan_matcher_options(p1);
}
void GetMotionFilterOptions(LocalTrajectoryBuilderOptions2D *p,
                            const CartoMappingConfig &m_MappingConfig) {
  // 1.5 为了防止子图里插入太多数据, motion_filter_
  ::cartographer::mapping::proto::MotionFilterOptions *p1 =
      p->mutable_motion_filter_options();
  p1->set_max_time_seconds(m_MappingConfig.motion_filter_.max_time_seconds);
  p1->set_max_distance_meters(
      m_MappingConfig.motion_filter_.max_distance_meters);
  p1->set_max_angle_radians(m_MappingConfig.motion_filter_.max_angle_radians);
  //   p->set_allocated_motion_filter_options(p1);
}
void GetPoseExp(LocalTrajectoryBuilderOptions2D *p,
                const CartoMappingConfig &m_MappingConfig) {
  // 1.6 位姿预测器  pose_extrapolator_
  ::cartographer::mapping::proto::PoseExtrapolatorOptions *p1 =
      p->mutable_pose_extrapolator_options();
  p1->set_use_imu_based(m_MappingConfig.pose_extrapolator_.use_imu_based);
  ::cartographer::mapping::proto::ConstantVelocityPoseExtrapolatorOptions
      *Constant_Velocity_ = p1->mutable_constant_velocity();
  Constant_Velocity_->set_imu_gravity_time_constant(
      m_MappingConfig.pose_extrapolator_.imu_gravity_time_constant);
  Constant_Velocity_->set_pose_queue_duration(
      m_MappingConfig.pose_extrapolator_.pose_queue_duration);
  //   p1->set_allocated_constant_velocity(Constant_Velocity_);
  ::cartographer::mapping::proto::ImuBasedPoseExtrapolatorOptions *Imu_based_ =
      p1->mutable_imu_based();
  Imu_based_->set_pose_queue_duration(
      m_MappingConfig.pose_extrapolator_.pose_queue_duration_);
  Imu_based_->set_gravity_constant(
      m_MappingConfig.pose_extrapolator_.gravity_constant);
  Imu_based_->set_pose_translation_weight(
      m_MappingConfig.pose_extrapolator_.pose_translation_weight);
  Imu_based_->set_pose_rotation_weight(
      m_MappingConfig.pose_extrapolator_.pose_rotation_weight);
  Imu_based_->set_imu_acceleration_weight(
      m_MappingConfig.pose_extrapolator_.imu_acceleration_weight);
  Imu_based_->set_imu_rotation_weight(
      m_MappingConfig.pose_extrapolator_.imu_rotation_weight);
  Imu_based_->set_odometry_translation_weight(
      m_MappingConfig.pose_extrapolator_.odometry_translation_weight);
  Imu_based_->set_odometry_rotation_weight(
      m_MappingConfig.pose_extrapolator_.odometry_rotation_weight);
  //   p1->set_allocated_imu_based(Imu_based_);
  ::cartographer::common::proto::CeresSolverOptions *Solver_options_ =
      Imu_based_->mutable_solver_options();
  Solver_options_->set_use_nonmonotonic_steps(
      m_MappingConfig.pose_extrapolator_.use_nonmonotonic_steps);
  Solver_options_->set_max_num_iterations(
      m_MappingConfig.pose_extrapolator_.max_num_iterations);
  Solver_options_->set_num_threads(
      m_MappingConfig.pose_extrapolator_.num_threads);
  //   Imu_based_->set_allocated_solver_options(Solver_options_);
  //   p->set_allocated_pose_extrapolator_options(p1);
}
void GetSubMapOptions(LocalTrajectoryBuilderOptions2D *p,
                      const CartoMappingConfig &m_MappingConfig) {
  ::cartographer::mapping::proto::SubmapsOptions2D *p1 =
      p->mutable_submaps_options();
  p1->set_num_range_data(m_MappingConfig.submaps_.num_range_data);
  ::cartographer::mapping::proto::GridOptions2D *Grid_Options_2d =
      p1->mutable_grid_options_2d();
  Grid_Options_2d->set_grid_type(
      (::cartographer::mapping::proto::GridOptions2D_GridType)
          m_MappingConfig.submaps_.grid_type);
  // 传入的参数类型::cartographer::mapping::proto::GridOptions2D_GridType
  Grid_Options_2d->set_resolution(m_MappingConfig.submaps_.resolution);
  //   p1->set_allocated_grid_options_2d(Grid_Options_2d);
  ::cartographer::mapping::proto::RangeDataInserterOptions
      *Range_data_inserter_options_ = p1->mutable_range_data_inserter_options();
  Range_data_inserter_options_->set_range_data_inserter_type(
      (::cartographer::mapping::proto::
           RangeDataInserterOptions_RangeDataInserterType)
          m_MappingConfig.submaps_.range_data_inserter_type);
  //强制类型转换
  //   p1->set_allocated_range_data_inserter_options(Range_data_inserter_options_);
  ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D
      *probability_grid_range_data_inserter_options_2d =
          Range_data_inserter_options_
              ->mutable_probability_grid_range_data_inserter_options_2d();
  probability_grid_range_data_inserter_options_2d->set_hit_probability(
      m_MappingConfig.submaps_.hit_probability);
  probability_grid_range_data_inserter_options_2d->set_miss_probability(
      m_MappingConfig.submaps_.miss_probability);
  probability_grid_range_data_inserter_options_2d->set_insert_free_space(
      m_MappingConfig.submaps_.insert_free_space);
  //   Range_data_inserter_options_
  //       ->set_allocated_probability_grid_range_data_inserter_options_2d(
  //           probability_grid_range_data_inserter_options_2d);
  //   p->set_allocated_submaps_options(p1);
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros