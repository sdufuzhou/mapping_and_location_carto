#include "include/mapping_lib/carto_mapping/create_options.h"
#include <iostream>
#include "include/common/logger.h"
namespace gomros {
namespace data_process {
namespace mapping_and_location {

//地图创建参数配置具体过程
cartographer::mapping::proto::MapBuilderOptions GetMapBuilderOptions(
    const CartoMappingConfig &mapping_config) {
  cartographer::mapping::proto::MapBuilderOptions map_builder_options;

  map_builder_options.set_use_trajectory_builder_2d(true);
  map_builder_options.set_num_background_threads(
      mapping_config.num_background_threads);
  map_builder_options.set_collate_by_trajectory(
      mapping_config.collate_by_trajectory);
  // 该函数返回pose_graph_options_是一个指针，类型为PoseGraphOptions
  PoseGraphOptions *pose_graph_options =
      map_builder_options.mutable_pose_graph_options();
  // PoseGraphOptions *p还有一些单独的变量需要赋值
  SetPoseGraphOptions(mapping_config.pose_graph_config, pose_graph_options);
  return map_builder_options;
}

cartographer::mapping::proto::TrajectoryBuilderOptions
GetTrajectoryBuilderOptions(
    const TrajectoryBuilderConfig &trajectory_builder_config) {
  cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  trajectory_builder_options.set_collate_fixed_frame(
      trajectory_builder_config.collate_fixed_frame);
  trajectory_builder_options.set_collate_landmarks(
      trajectory_builder_config.collate_landmarks);
  auto *trajectory_builder_2d_options =
      trajectory_builder_options.mutable_trajectory_builder_2d_options();
  trajectory_builder_2d_options->set_use_imu_data(
      trajectory_builder_config.use_imu_data);
  trajectory_builder_2d_options->set_min_range(
      trajectory_builder_config.min_range);
  trajectory_builder_2d_options->set_max_range(
      trajectory_builder_config.max_range);
  trajectory_builder_2d_options->set_min_z(trajectory_builder_config.min_z);
  trajectory_builder_2d_options->set_max_z(trajectory_builder_config.max_z);
  trajectory_builder_2d_options->set_missing_data_ray_length(
      trajectory_builder_config.missing_data_ray_length);
  trajectory_builder_2d_options->set_num_accumulated_range_data(
      trajectory_builder_config.num_accumulated_range_data);
  trajectory_builder_2d_options->set_voxel_filter_size(
      trajectory_builder_config.voxel_filter_size);
  auto *adaptive_voxel_filter_options =
      trajectory_builder_2d_options->mutable_adaptive_voxel_filter_options();
  SetAdaptiveVoxelFilterOptions(
      trajectory_builder_config.adaptive_voxel_filter_config,
      adaptive_voxel_filter_options);
  auto *loop_closure_adaptive_voxel_filter_options =
      trajectory_builder_2d_options
          ->mutable_loop_closure_adaptive_voxel_filter_options();
  SetLoopClosureAdaptiveVoxelFilterOptions(
      trajectory_builder_config.loop_closure_adaptive_voxel_filter_config,
      loop_closure_adaptive_voxel_filter_options);
  trajectory_builder_2d_options->set_use_online_correlative_scan_matching(
      trajectory_builder_config.use_online_correlative_scan_matching);
  auto *real_time_correlative_scan_matcher_options =
      trajectory_builder_2d_options
          ->mutable_real_time_correlative_scan_matcher_options();
  SetRealTimeCorrelativeScanMatcherOptions(
      trajectory_builder_config.real_time_correlative_scan_matcher_config,
      real_time_correlative_scan_matcher_options);
  auto *ceres_scan_matcher_options =
      trajectory_builder_2d_options->mutable_ceres_scan_matcher_options();
  SetCeresScanMatcherOptions(
      trajectory_builder_config.ceres_scan_matcher_config,
      ceres_scan_matcher_options);
  LOG_WARN << "######trajectory_builder_config.ceres_scan_matcher_config:"
           << trajectory_builder_config.ceres_scan_matcher_config
                  .occupied_space_weight;
  auto *motion_filter_options =
      trajectory_builder_2d_options->mutable_motion_filter_options();
  SetMotionFilterOptions(trajectory_builder_config.motion_filter_config,
                         motion_filter_options);
  trajectory_builder_2d_options->set_imu_gravity_time_constant(
      trajectory_builder_config.imu_gravity_time_constant);
  auto *pose_extrapolator_options =
      trajectory_builder_2d_options->mutable_pose_extrapolator_options();
  SetPoseExtrapolatorOptions(trajectory_builder_config.pose_extrapolator_config,
                             pose_extrapolator_options);
  auto *submap_options =
      trajectory_builder_2d_options->mutable_submaps_options();
  SetSubmapOptions(trajectory_builder_config.submap_config, submap_options);
  return trajectory_builder_options;
}

void SetAdaptiveVoxelFilterOptions(
    const AdaptiveVoxelFilterConfig adaptive_voxel_filter_config,
    AdaptiveVoxelFilterOptions *adaptive_voxel_filter_options) {
  adaptive_voxel_filter_options->set_max_length(
      adaptive_voxel_filter_config.max_length);
  adaptive_voxel_filter_options->set_min_num_points(
      adaptive_voxel_filter_config.min_num_points);
  adaptive_voxel_filter_options->set_max_range(
      adaptive_voxel_filter_config.max_range);
}

void SetLoopClosureAdaptiveVoxelFilterOptions(
    const LoopClosureAdaptiveVoxelFilterConfig
        loop_closure_adaptive_voxel_filter_config,
    AdaptiveVoxelFilterOptions *loop_closure_adaptive_voxel_filter_options) {
  loop_closure_adaptive_voxel_filter_options->set_max_length(
      loop_closure_adaptive_voxel_filter_config.max_length);
  loop_closure_adaptive_voxel_filter_options->set_min_num_points(
      loop_closure_adaptive_voxel_filter_config.min_num_points);
  loop_closure_adaptive_voxel_filter_options->set_max_range(
      loop_closure_adaptive_voxel_filter_config.max_range);
}

void SetRealTimeCorrelativeScanMatcherOptions(
    const RealTimeCorrelativeScanMatcherConfig
        real_time_correlative_scan_matcher_config,
    RealTimeCorrelativeScanMatcherOptions
        *real_time_correlative_scan_matcher_options) {
  real_time_correlative_scan_matcher_options->set_linear_search_window(
      real_time_correlative_scan_matcher_config.linear_search_window);
  real_time_correlative_scan_matcher_options->set_angular_search_window(
      real_time_correlative_scan_matcher_config.angular_search_window);
  real_time_correlative_scan_matcher_options->set_translation_delta_cost_weight(
      real_time_correlative_scan_matcher_config.translation_delta_cost_weight);
  real_time_correlative_scan_matcher_options->set_rotation_delta_cost_weight(
      real_time_correlative_scan_matcher_config.rotation_delta_cost_weight);
}

void SetCeresScanMatcherOptions(
    const CeresScanMatcherConfig ceres_scan_matcher_config,
    CeresScanMatcherOptions2D *ceres_scan_matcher_options) {
  ceres_scan_matcher_options->set_occupied_space_weight(
      ceres_scan_matcher_config.occupied_space_weight);
  ceres_scan_matcher_options->set_translation_weight(
      ceres_scan_matcher_config.translation_weight);
  ceres_scan_matcher_options->set_rotation_weight(
      ceres_scan_matcher_config.rotation_weight);
  auto *ceres_slover_options =
      ceres_scan_matcher_options->mutable_ceres_solver_options();
  SetCeresSolverOptions(ceres_scan_matcher_config.ceres_solver_config,
                        ceres_slover_options);
}

void SetCeresSolverOptions(const CeresSolverConfig ceres_slover_config,
                           CeresSolverOptions *ceres_slover_options) {
  ceres_slover_options->set_use_nonmonotonic_steps(
      ceres_slover_config.use_nonmonotonic_steps);
  ceres_slover_options->set_max_num_iterations(
      ceres_slover_config.max_num_iterations);
  ceres_slover_options->set_num_threads(ceres_slover_config.num_threads);
}

void SetMotionFilterOptions(const MotionFilterConfig montion_filter_config,
                            MotionFilterOptions *montion_filter_options) {
  montion_filter_options->set_max_time_seconds(
      montion_filter_config.max_time_seconds);
  montion_filter_options->set_max_distance_meters(
      montion_filter_config.max_distance_meters);
  montion_filter_options->set_max_angle_radians(
      montion_filter_config.max_angle_radians);
}

void SetPoseExtrapolatorOptions(
    const PoseExtrapolatorConfig pose_extrapolator_config,
    PoseExtrapolatorOptions *pose_extrapolator_options) {
  pose_extrapolator_options->set_use_imu_based(
      pose_extrapolator_config.use_imu_based);
  pose_extrapolator_options->mutable_constant_velocity()
      ->set_imu_gravity_time_constant(
          pose_extrapolator_config.constant_velocity_config
              .imu_gravity_time_constant);
  auto *imu_based_options = pose_extrapolator_options->mutable_imu_based();
  SetImuBasedOptions(pose_extrapolator_config.imu_based_config,
                     imu_based_options);
}

void SetImuBasedOptions(const ImuBasedConfig imu_based_config,
                        ImuBasedPoseExtrapolatorOptions *imu_based_options) {
  imu_based_options->set_pose_queue_duration(
      imu_based_config.pose_queue_duration);
  imu_based_options->set_gravity_constant(imu_based_config.gravity_constant);
  imu_based_options->set_pose_translation_weight(
      imu_based_config.pose_translation_weight);
  imu_based_options->set_pose_rotation_weight(
      imu_based_config.pose_rotation_weight);
  imu_based_options->set_imu_acceleration_weight(
      imu_based_config.imu_acceleration_weight);
  imu_based_options->set_imu_rotation_weight(
      imu_based_config.imu_rotation_weight);
  imu_based_options->set_odometry_translation_weight(
      imu_based_config.odometry_translation_weight);
  imu_based_options->set_odometry_rotation_weight(
      imu_based_config.odometry_rotation_weight);
  auto *ceres_solver_options = imu_based_options->mutable_solver_options();
  SetCeresSolverOptions(imu_based_config.ceres_solver_config,
                        ceres_solver_options);
}

void SetSubmapOptions(const SubmapConfig submap_config,
                      SubmapsOptions2D *submap_options) {
  submap_options->set_num_range_data(submap_config.num_range_data);
  submap_options->mutable_grid_options_2d()->set_grid_type(
      (::cartographer::mapping::proto::GridOptions2D_GridType)(
          submap_config.grid_type));
  submap_options->mutable_grid_options_2d()->set_resolution(
      submap_config.resolution);
  auto *range_data_inserter_options =
      submap_options->mutable_range_data_inserter_options();
  SetRangeDataInserterOptions(submap_config.range_data_inserter_config,
                              range_data_inserter_options);
}

void SetRangeDataInserterOptions(
    const RangeDataInserterConfig range_data_inserter_config,
    RangeDataInserterOptions *range_data_inserter_options) {
  range_data_inserter_options->set_range_data_inserter_type(
      (RangeDataInserterOptions_RangeDataInserterType)
          range_data_inserter_config.range_data_inserter_type);
  SLAM_INFO("range_data_inserter_type为：%d\n",
            range_data_inserter_config.range_data_inserter_type);
  auto *probability_grid_range_data_inserter_options =
      range_data_inserter_options
          ->mutable_probability_grid_range_data_inserter_options_2d();
  SetProbabilityGridRangeDataInserterOptions2D(
      range_data_inserter_config.probability_grid_range_data_inserter_config,
      probability_grid_range_data_inserter_options);

  auto *tsdf_range_data_inserter_options =
      range_data_inserter_options
          ->mutable_tsdf_range_data_inserter_options_2d();
  SetTSDFRangeDataInserterOptions2D(
      range_data_inserter_config.tsdf_range_data_inserter_config,
      tsdf_range_data_inserter_options);
}

void SetProbabilityGridRangeDataInserterOptions2D(
    const ProbabilityGridRangeDataInserterConfig config,
    ProbabilityGridRangeDataInserterOptions2D *options) {
  options->set_insert_free_space(config.insert_free_space);
  options->set_hit_probability(config.hit_probability);
  options->set_miss_probability(config.miss_probability);
}

void SetTSDFRangeDataInserterOptions2D(
    const TsdfRangeDataInserterConfig config,
    TSDFRangeDataInserterOptions2D *options) {
  options->set_truncation_distance(config.truncation_distance);
  options->set_maximum_weight(config.maximum_weight);
  options->set_update_free_space(config.update_free_space);
  options->mutable_normal_estimation_options()->set_num_normal_samples(
      config.num_normal_samples);
  options->mutable_normal_estimation_options()->set_sample_radius(
      config.sample_radius);
  options->set_project_sdf_distance_to_scan_normal(
      config.project_sdf_distance_to_scan_normal);
  options->set_update_weight_range_exponent(
      config.update_weight_range_exponent);
  options->set_update_weight_angle_scan_normal_to_ray_kernel_bandwidth(
      config.update_weight_angle_scan_normal_to_ray_kernel_bandwidth);
  options->set_update_weight_distance_cell_to_hit_kernel_bandwidth(
      config.update_weight_distance_cell_to_hit_kernel_bandwidth);
}

void SetPoseGraphOptions(const PoseGraphConfig pose_graph_config,
                         PoseGraphOptions *pose_graph_options) {
  pose_graph_options->set_optimize_every_n_nodes(
      pose_graph_config.optimize_every_n_nodes);
  auto *constraint_builder_options =
      pose_graph_options->mutable_constraint_builder_options();
  SetConstraintBuilderOptions(pose_graph_config.constraint_builder_config,
                              constraint_builder_options);
  pose_graph_options->set_matcher_translation_weight(
      pose_graph_config.matcher_translation_weight);
  pose_graph_options->set_matcher_rotation_weight(
      pose_graph_config.matcher_rotation_weight);
  auto *optimization_problem =
      pose_graph_options->mutable_optimization_problem_options();
  SetOptimizationProblemOptions(pose_graph_config.optimization_problem_config,
                                optimization_problem);
  pose_graph_options->set_max_num_final_iterations(
      pose_graph_config.max_num_final_iterations);
  pose_graph_options->set_global_sampling_ratio(
      pose_graph_config.global_sampling_ratio);
  pose_graph_options->set_log_residual_histograms(
      pose_graph_config.log_residual_histograms);
  pose_graph_options->set_global_constraint_search_after_n_seconds(
      pose_graph_config.global_constraint_search_after_n_seconds);
}

//约束构建的相关参数
void SetConstraintBuilderOptions(const ConstraintBuilderConfig config,
                                 ConstraintBuilderOptions *options) {
  // 约束构建的相关参数
  options->set_sampling_ratio(config.sampling_ratio);
  options->set_max_constraint_distance(config.max_constraint_distance);
  options->set_min_score(config.min_score);
  options->set_global_localization_min_score(
      config.global_localization_min_score);
  options->set_loop_closure_translation_weight(
      config.loop_closure_translation_weight);
  options->set_loop_closure_rotation_weight(
      config.loop_closure_rotation_weight);
  options->set_log_matches(config.log_matches);
  auto *fast_correlative_scan_matcher_options =
      options->mutable_fast_correlative_scan_matcher_options();
  SetFastCorrelativeScanMatcherOptions2DOptions(
      config.fast_correlative_scan_matcher_config,
      fast_correlative_scan_matcher_options);
  auto *ceres_scan_matcher_options =
      options->mutable_ceres_scan_matcher_options();
  SetCeresScanMatcherOptions(config.ceres_scan_matcher_config,
                             ceres_scan_matcher_options);
}

void SetFastCorrelativeScanMatcherOptions2DOptions(
    const FastCorrelativeScanMatcherConfig config,
    FastCorrelativeScanMatcherOptions2D *options) {
  options->set_linear_search_window(config.linear_search_window);
  options->set_angular_search_window(config.angular_search_window);
  options->set_branch_and_bound_depth(config.branch_and_bound_depth);
}

void SetOptimizationProblemOptions(const OptimizationProblemConfig config,
                                   OptimizationProblemOptions *options) {
  options->set_huber_scale(config.huber_scale);
  // 前端结果残差的权重
  options->set_local_slam_pose_translation_weight(
      config.local_slam_pose_translation_weight);
  options->set_local_slam_pose_rotation_weight(
      config.local_slam_pose_rotation_weight);
  // 里程计残差的权重
  options->set_odometry_translation_weight(config.odometry_translation_weight);
  options->set_odometry_rotation_weight(config.odometry_rotation_weight);
  // gps残差的权重
  options->set_fixed_frame_pose_translation_weight(
      config.fixed_frame_pose_translation_weight);
  options->set_fixed_frame_pose_rotation_weight(
      config.fixed_frame_pose_rotation_weight);
  options->set_fixed_frame_pose_use_tolerant_loss(
      config.fixed_frame_pose_use_tolerant_loss);
  options->set_fixed_frame_pose_tolerant_loss_param_a(
      config.fixed_frame_pose_tolerant_loss_param_a);
  options->set_fixed_frame_pose_tolerant_loss_param_b(
      config.fixed_frame_pose_tolerant_loss_param_b);
  options->set_log_solver_summary(config.log_solver_summary);
  options->set_use_online_imu_extrinsics_in_3d(
      config.use_online_imu_extrinsics_in_3d);
  options->set_fix_z_in_3d(config.fix_z_in_3d);
  auto *ceres_scan_matcher_options = options->mutable_ceres_solver_options();
  SetCeresSolverOptions(config.ceres_solver_config, ceres_scan_matcher_options);
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros