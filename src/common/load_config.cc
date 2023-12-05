/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-10 13:48:07
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-02 15:20:27
 */
#include "include/common/load_config.h"
#include <math.h>
// #include <filesystem>

namespace gomros {
namespace data_process {
namespace mapping_and_location {

void MappingAndLocationConfigTransition(
    const MappingAndLocationCfg& config,
    MappingAndLocationConfig& config_result) {
  config_result.node_name = config.node_name;
  config_result.sub_odom_topic = config.sub_odom_topic;
  config_result.sub_radar_topic = config.sub_radar_topic;
  config_result.sub_imu_topic = config.sub_imu_topic;
  config_result.sub_global_locate_cmd_topic =
      config.sub_global_locate_cmd_topic;
  config_result.sub_start_mapping_cmd_topic =
      config.sub_start_mapping_cmd_topic;
  config_result.sub_stop_mapping_cmd_topic = config.sub_stop_mapping_cmd_topic;
  config_result.ad_state_topic = config.ad_state_topic;
  config_result.ad_pose_topic = config.ad_pose_topic;
  config_result.log_file_name = config.log_file_name;
  config_result.log_level = config.log_level;
  config_result.is_printf_to_terminal = config.is_printf_to_terminal;
  config_result.mapping_algorithm = (MappingAlgorithm)config.mapping_algorithm;
  // karto_config
  {
    auto& karto_config = config_result.karto_config;
    karto_config.mapping_pattern =
        (MappingPattern)(config.karto_config.mapping_pattern);
    karto_config.distance_step = config.karto_config.distance_step;
    karto_config.angular_step = config.karto_config.angular_step;
    karto_config.mapping_start_angle = config.radar_start_angle;
    karto_config.mapping_end_angle = config.radar_end_angle;
    karto_config.radar_resolution = config.radar_resolution;
    karto_config.mapping_resolution = config.radar_resolution;
    karto_config.radar_position_x = config.radar_position_x;
    karto_config.radar_position_y = config.radar_position_y;
    karto_config.radar_position_theta = config.radar_position_theta;
    karto_config.mapping_laser_min_range =
        config.karto_config.mapping_laser_min_range;
    karto_config.mapping_laser_max_range =
        config.karto_config.mapping_laser_max_range;
    karto_config.travel_distance = config.karto_config.distance_step;
    karto_config.travel_angle = config.karto_config.angular_step;
    karto_config.loop_search_distance =
        config.karto_config.loop_search_distance;
    karto_config.map_data_file_path = config.karto_config.map_data_file_path;
    karto_config.map_config_file_path =
        config.karto_config.map_config_file_path;
  }

  // carto_config
  {
    auto& carto_config = config_result.carto_config;
    carto_config.mapping_pattern = config.carto_config.mapping_pattern;
    carto_config.mapping_start_angle = config.radar_start_angle;
    carto_config.mapping_end_angle = config.radar_end_angle;
    carto_config.mapping_resolution = config.radar_resolution;
    carto_config.radar_resolution = config.radar_resolution;
    carto_config.radar_position_x = config.radar_position_x;
    carto_config.radar_position_y = config.radar_position_y;
    carto_config.radar_position_theta = config.radar_position_theta;
    carto_config.wheel_odom_position_theta =
        config.carto_config.wheel_odom_position_theta;
    carto_config.wheel_odom_position_x =
        config.carto_config.wheel_odom_position_x;
    carto_config.wheel_odom_position_y =
        config.carto_config.wheel_odom_position_y;
    carto_config.map_config_file_path =
        config.carto_config.map_config_file_path;
    carto_config.map_data_file_path = config.carto_config.map_data_file_path;
    carto_config.num_background_threads =
        config.carto_config.num_background_threads;
    auto& trajectory_config = carto_config.trajectory_config;
    auto& pose_graph_config = carto_config.pose_graph_config;
    carto_config.collate_by_trajectory =
        config.carto_config.collate_by_trajectory;
    // trajectory_config
    {
      auto trajectory_config_in = config.carto_config.trajectory_config;
      trajectory_config.use_imu_data = trajectory_config_in.use_imu_data;
      trajectory_config.use_odom_data = trajectory_config_in.use_odom_data;
      trajectory_config.min_range = trajectory_config_in.min_range;
      trajectory_config.max_range = trajectory_config_in.max_range;
      trajectory_config.min_z = trajectory_config_in.min_z;
      trajectory_config.max_z = trajectory_config_in.max_z;
      trajectory_config.missing_data_ray_length =
          trajectory_config_in.missing_data_ray_length;
      trajectory_config.num_accumulated_range_data =
          trajectory_config_in.num_accumulated_range_data;
      trajectory_config.voxel_filter_size =
          trajectory_config_in.voxel_filter_size;
      auto& adaptive_voxel_filter_config =
          trajectory_config.adaptive_voxel_filter_config;
      auto& loop_closure_adaptive_voxel_filter_config =
          trajectory_config.loop_closure_adaptive_voxel_filter_config;
      trajectory_config.use_online_correlative_scan_matching =
          trajectory_config_in.use_online_correlative_scan_matching;
      auto& real_time_correlative_scan_matcher_config =
          trajectory_config.real_time_correlative_scan_matcher_config;
      auto& ceres_scan_matcher_config =
          trajectory_config_in.ceres_scan_matcher_config;
      auto& motion_filter_config = trajectory_config.motion_filter_config;
      trajectory_config.imu_gravity_time_constant =
          trajectory_config_in.imu_gravity_time_constant;
      auto& pose_extrapolator_config =
          trajectory_config.pose_extrapolator_config;
      auto& submap_config = trajectory_config.submap_config;
      trajectory_config.collate_fixed_frame =
          trajectory_config_in.collate_fixed_frame;
      trajectory_config.collate_landmarks =
          trajectory_config_in.collate_landmarks;
      // adaptive_voxel_filter_config
      {
        auto adaptive_voxel_filter_config_in =
            trajectory_config_in.adaptive_voxel_filter_config;
        adaptive_voxel_filter_config = {
            adaptive_voxel_filter_config_in.max_length,
            adaptive_voxel_filter_config_in.min_num_points,
            adaptive_voxel_filter_config_in.max_range};
      }

      // loop_closure_adaptive_voxel_filter_config
      {
        auto loop_closure_adaptive_voxel_filter_config_in =
            trajectory_config_in.loop_closure_adaptive_voxel_filter_config;
        loop_closure_adaptive_voxel_filter_config = {
            loop_closure_adaptive_voxel_filter_config_in.max_length,
            loop_closure_adaptive_voxel_filter_config_in.min_num_points,
            loop_closure_adaptive_voxel_filter_config_in.max_range};
      }

      // real_time_correlative_scan_matcher_config
      {
        auto real_time_correlative_scan_matcher_config_in =
            trajectory_config_in.real_time_correlative_scan_matcher_config;
        real_time_correlative_scan_matcher_config = {
            real_time_correlative_scan_matcher_config_in.linear_search_window,
            real_time_correlative_scan_matcher_config_in.angular_search_window /
                180.0 * M_PI,
            real_time_correlative_scan_matcher_config_in
                .translation_delta_cost_weight,
            real_time_correlative_scan_matcher_config_in
                .rotation_delta_cost_weight};
      }

      // ceres_scan_matcher_config
      {
        auto ceres_scan_matcher_config_in =
            trajectory_config_in.ceres_scan_matcher_config;
        ceres_scan_matcher_config.occupied_space_weight =
            ceres_scan_matcher_config_in.occupied_space_weight;
        ceres_scan_matcher_config.translation_weight =
            ceres_scan_matcher_config_in.translation_weight;
        ceres_scan_matcher_config.rotation_weight =
            ceres_scan_matcher_config_in.rotation_weight;
        auto& ceres_solver_options =
            ceres_scan_matcher_config.ceres_solver_config;
        ceres_solver_options = {
            ceres_scan_matcher_config_in.ceres_solver_config
                .use_nonmonotonic_steps,
            ceres_scan_matcher_config_in.ceres_solver_config.max_num_iterations,
            ceres_scan_matcher_config_in.ceres_solver_config.num_threads};
      }

      // motion_filter_config
      {
        auto motion_filter_config_in =
            trajectory_config_in.motion_filter_config;
        motion_filter_config = {
            motion_filter_config_in.max_time_seconds,
            motion_filter_config_in.max_distance_meters,
            motion_filter_config_in.max_angle_radians / 180.0 * M_PI};
      }

      // pose_extrapolator_config
      {
        auto pose_extrapolator_in =
            trajectory_config_in.pose_extrapolator_config;
        pose_extrapolator_config.use_imu_based =
            pose_extrapolator_in.use_imu_based;
        auto& constant_velocity_config =
            pose_extrapolator_config.constant_velocity_config;
        auto& imu_based_config = pose_extrapolator_config.imu_based_config;

        constant_velocity_config = {
            pose_extrapolator_in.constant_velocity_config
                .imu_gravity_time_constant,
            pose_extrapolator_in.constant_velocity_config.pose_queue_duration};
        {
          auto imu_based_config_in = pose_extrapolator_in.imu_based_config;
          imu_based_config.pose_queue_duration =
              imu_based_config_in.pose_queue_duration;
          imu_based_config.gravity_constant =
              imu_based_config_in.gravity_constant;
          imu_based_config.pose_translation_weight =
              imu_based_config_in.pose_translation_weight;
          imu_based_config.pose_rotation_weight =
              imu_based_config_in.pose_rotation_weight;
          imu_based_config.imu_acceleration_weight =
              imu_based_config_in.imu_acceleration_weight;
          imu_based_config.imu_rotation_weight =
              imu_based_config_in.imu_rotation_weight;
          imu_based_config.odometry_translation_weight =
              imu_based_config_in.odometry_translation_weight;
          imu_based_config.odometry_rotation_weight =
              imu_based_config_in.odometry_rotation_weight;
          auto& ceres_solver_options = imu_based_config.ceres_solver_config;
          ceres_solver_options = {
              imu_based_config_in.ceres_solver_config.use_nonmonotonic_steps,
              imu_based_config_in.ceres_solver_config.max_num_iterations,
              imu_based_config_in.ceres_solver_config.num_threads};
        }
      }

      // submap_config
      {
        auto submap_config_in = trajectory_config_in.submap_config;
        submap_config.num_range_data = submap_config_in.num_range_data;
        submap_config.grid_type = submap_config_in.grid_type;
        submap_config.resolution = submap_config_in.resolution;
        auto& range_data_inserter_config =
            submap_config.range_data_inserter_config;

        // range_data_inserter_config
        {
          auto range_data_inserter_config_in =
              submap_config_in.range_data_inserter_config;
          range_data_inserter_config.range_data_inserter_type =
              range_data_inserter_config_in.range_data_inserter_type;
          range_data_inserter_config
              .probability_grid_range_data_inserter_config = {
              range_data_inserter_config_in
                  .probability_grid_range_data_inserter_config
                  .insert_free_space,
              range_data_inserter_config_in
                  .probability_grid_range_data_inserter_config.hit_probability,
              range_data_inserter_config_in
                  .probability_grid_range_data_inserter_config
                  .miss_probability};
          range_data_inserter_config.tsdf_range_data_inserter_config = {
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .truncation_distance,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .maximum_weight,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .update_free_space,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .num_normal_samples,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .sample_radius,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .project_sdf_distance_to_scan_normal,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .update_weight_range_exponent,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .update_weight_angle_scan_normal_to_ray_kernel_bandwidth,
              range_data_inserter_config_in.tsdf_range_data_inserter_config
                  .update_weight_distance_cell_to_hit_kernel_bandwidth};
        }
      }
    }

    // pose_graph_config
    {
      auto pose_graph_config_in = config.carto_config.pose_graph_config;
      pose_graph_config.optimize_every_n_nodes =
          pose_graph_config_in.optimize_every_n_nodes;
      auto& constraint_builder_config =
          pose_graph_config.constraint_builder_config;
      pose_graph_config.matcher_translation_weight =
          pose_graph_config_in.matcher_translation_weight;
      pose_graph_config.matcher_rotation_weight =
          pose_graph_config_in.matcher_rotation_weight;
      auto& optimization_problem_config =
          pose_graph_config.optimization_problem_config;
      pose_graph_config.max_num_final_iterations =
          pose_graph_config_in.max_num_final_iterations;
      pose_graph_config.global_sampling_ratio =
          pose_graph_config_in.global_sampling_ratio;
      pose_graph_config.log_residual_histograms =
          pose_graph_config_in.log_residual_histograms;
      pose_graph_config.global_constraint_search_after_n_seconds =
          pose_graph_config_in.global_constraint_search_after_n_seconds;

      // constraint_builder_config
      {
        auto constraint_builder_config_in =
            pose_graph_config_in.constraint_builder_config;
        constraint_builder_config.sampling_ratio =
            constraint_builder_config_in.sampling_ratio;
        constraint_builder_config.max_constraint_distance =
            constraint_builder_config_in.max_constraint_distance;
        constraint_builder_config.min_score =
            constraint_builder_config_in.min_score;
        constraint_builder_config.global_localization_min_score =
            constraint_builder_config_in.global_localization_min_score;
        constraint_builder_config.loop_closure_translation_weight =
            constraint_builder_config_in.loop_closure_translation_weight;
        constraint_builder_config.loop_closure_rotation_weight =
            constraint_builder_config_in.loop_closure_rotation_weight;
        constraint_builder_config.log_matches =
            constraint_builder_config_in.log_matches;
        auto& fast_correlative_scan_matcher_config =
            constraint_builder_config.fast_correlative_scan_matcher_config;
        auto& ceres_scan_matcher_config =
            constraint_builder_config.ceres_scan_matcher_config;

        {
          auto fast_correlative_scan_matcher_config_in =
              constraint_builder_config.fast_correlative_scan_matcher_config;
          fast_correlative_scan_matcher_config = {
              fast_correlative_scan_matcher_config_in.linear_search_window,
              fast_correlative_scan_matcher_config_in.angular_search_window /
                  180.0 * M_PI,
              fast_correlative_scan_matcher_config_in.branch_and_bound_depth};
        }
        // ceres_scan_matcher_config
        {
          auto ceres_scan_matcher_config_in =
              constraint_builder_config.ceres_scan_matcher_config;
          ceres_scan_matcher_config.occupied_space_weight =
              ceres_scan_matcher_config_in.occupied_space_weight;
          ceres_scan_matcher_config.translation_weight =
              ceres_scan_matcher_config_in.translation_weight;
          ceres_scan_matcher_config.rotation_weight =
              ceres_scan_matcher_config_in.rotation_weight;
          auto& ceres_solver_config =
              ceres_scan_matcher_config.ceres_solver_config;
          ceres_solver_config = {
              ceres_scan_matcher_config_in.ceres_solver_config
                  .use_nonmonotonic_steps,
              ceres_scan_matcher_config_in.ceres_solver_config
                  .max_num_iterations,
              ceres_scan_matcher_config_in.ceres_solver_config.num_threads};
        }
      }

      // optimization_problem_config
      {
        auto optimization_problem_config_in =
            pose_graph_config_in.optimization_problem_config;
        optimization_problem_config.huber_scale =
            optimization_problem_config_in.huber_scale;
        optimization_problem_config.rotation_weight =
            optimization_problem_config_in.rotation_weight;
        optimization_problem_config.acceleration_weight =
            optimization_problem_config_in.acceleration_weight;
        optimization_problem_config.local_slam_pose_translation_weight =
            optimization_problem_config_in.local_slam_pose_translation_weight;
        optimization_problem_config.local_slam_pose_rotation_weight =
            optimization_problem_config_in.local_slam_pose_rotation_weight;
        optimization_problem_config.odometry_translation_weight =
            optimization_problem_config_in.odometry_translation_weight;
        optimization_problem_config.odometry_rotation_weight =
            optimization_problem_config_in.odometry_rotation_weight;
        optimization_problem_config.fixed_frame_pose_translation_weight =
            optimization_problem_config_in.fixed_frame_pose_translation_weight;
        optimization_problem_config.fixed_frame_pose_rotation_weight =
            optimization_problem_config_in
                .fixed_frame_pose_tolerant_loss_param_b;
        optimization_problem_config.fixed_frame_pose_use_tolerant_loss =
            optimization_problem_config_in.fixed_frame_pose_use_tolerant_loss;
        optimization_problem_config.fixed_frame_pose_tolerant_loss_param_b =
            optimization_problem_config_in
                .fixed_frame_pose_tolerant_loss_param_b;
        optimization_problem_config.fixed_frame_pose_tolerant_loss_param_a =
            optimization_problem_config_in
                .fixed_frame_pose_tolerant_loss_param_a;
        optimization_problem_config.log_solver_summary =
            optimization_problem_config_in.log_solver_summary;
        optimization_problem_config.use_online_imu_extrinsics_in_3d =
            optimization_problem_config_in.use_online_imu_extrinsics_in_3d;
        optimization_problem_config.fix_z_in_3d =
            optimization_problem_config_in.fix_z_in_3d;
        auto& ceres_solver_config =
            optimization_problem_config.ceres_solver_config;
        ceres_solver_config = {
            optimization_problem_config_in.ceres_solver_config
                .use_nonmonotonic_steps,
            optimization_problem_config_in.ceres_solver_config
                .max_num_iterations,
            optimization_problem_config_in.ceres_solver_config.num_threads};
      }
    }
  }
  // location_config
  auto location_config_in = config.location_config;
  config_result.location_config = {
      config.radar_start_angle,
      config.radar_end_angle,
      config.radar_resolution,
      config.radar_position_x,
      config.radar_position_y,
      config.radar_position_theta,
      location_config_in.use_scan_matching,
      location_config_in.scan_matching_min_distance,
      location_config_in.scan_matching_max_distance,
      location_config_in.occupied_space_cost_factor,
      location_config_in.translation_delta_cost_factor,
      location_config_in.rotation_delta_cost_factor,
      location_config_in.num_threads,
      location_config_in.max_num_iterations,
      location_config_in.laser_max_range,
      location_config_in.laser_min_range,
      location_config_in.distance_threhold,
      location_config_in.angle_threahold,
      location_config_in.resample_interval,
      location_config_in.laser_max_beams,
      location_config_in.odom_type,
      location_config_in.only_radar_pose,
      location_config_in.is_initial_locate,
      location_config_in.init_pose_file_path};
}

bool ReadMappingAndLocationConfig(const std::string& config_dir,
                                  MappingAndLocationConfig* config_result) {
  MappingAndLocationCfg temp_config;
  if (ConfigUtils::decode(config_dir.c_str(), temp_config)) {
    MappingAndLocationConfigTransition(temp_config, *config_result);
    return true;
  }
  return false;
}

void DefaultWriteMappingAndLocationConfig(const std::string& config_dir) {
  std::string cmd = "mkdir -p " + config_dir;
  if (system(cmd.c_str())) {
    MappingAndLocationCfg config;
    // 独有的部分（最外层的Xpack直接按下面方式赋值）
    config.node_name = "mapping_and_location";
    config.sub_odom_topic = "odom_topic";
    config.sub_radar_topic = "radar/wj716/sensory";
    config.sub_imu_topic = "imu_sensory_msg";
    config.sub_global_locate_cmd_topic = "global_locate_cmd";
    config.sub_start_mapping_cmd_topic = "start_mapping";
    config.sub_stop_mapping_cmd_topic = "stop_mapping";
    config.ad_state_topic = "location_state";
    config.ad_pose_topic = "location_pose";
    config.log_file_name = "mapping_location_log";
    config.log_level = 0;
    config.is_printf_to_terminal = true;
    config.mapping_algorithm = 0;
    config.radar_start_angle = -95.0;
    config.radar_end_angle = 95.0;
    config.radar_resolution = 1.0;
    config.radar_position_x = 0.361;
    config.radar_position_y = 0.0;
    config.radar_position_theta = 0.0;

    {
      auto& karto_config = config.karto_config;
      karto_config.mapping_pattern = 1;
      karto_config.distance_step = 0.1;
      karto_config.angular_step = 0.1;
      karto_config.mapping_resolution = 0.02;
      karto_config.mapping_laser_min_range = 0.5;
      karto_config.mapping_laser_max_range = 19.0;
      karto_config.travel_distance = 0.2;
      karto_config.travel_angle = 0.175;
      karto_config.loop_search_distance = 4.0;
      karto_config.map_data_file_path = "./data";
      karto_config.map_config_file_path = "./config";
    }

    // carto_config
    {
      auto& carto_config = config.carto_config;
      carto_config.mapping_pattern = 1;
      carto_config.mapping_resolution = 0.02;
      carto_config.wheel_odom_position_theta = 0.0;
      carto_config.wheel_odom_position_x = 0.0;
      carto_config.wheel_odom_position_y = 0.0;
      carto_config.map_config_file_path = "./config";
      carto_config.map_data_file_path = "./data";
      carto_config.num_background_threads = 4;
      auto& trajectory_config = carto_config.trajectory_config;
      auto& pose_graph_config = carto_config.pose_graph_config;
      carto_config.collate_by_trajectory = false;
      // trajectory_config
      {
        trajectory_config.use_imu_data = false;
        trajectory_config.use_odom_data = false;
        trajectory_config.min_range = 0.3;
        trajectory_config.max_range = 19.0;
        trajectory_config.min_z = -0.8;
        trajectory_config.max_z = 2.0;
        trajectory_config.missing_data_ray_length = 5.0;
        trajectory_config.num_accumulated_range_data = 1;
        trajectory_config.voxel_filter_size = 0.05;
        auto& adaptive_voxel_filter_config =
            trajectory_config.adaptive_voxel_filter_config;
        auto& loop_closure_adaptive_voxel_filter_config =
            trajectory_config.loop_closure_adaptive_voxel_filter_config;
        trajectory_config.use_online_correlative_scan_matching = true;
        auto& real_time_correlative_scan_matcher_config =
            trajectory_config.real_time_correlative_scan_matcher_config;
        auto& ceres_scan_matcher_config =
            trajectory_config.ceres_scan_matcher_config;
        auto& motion_filter_config = trajectory_config.motion_filter_config;
        trajectory_config.imu_gravity_time_constant = 10.0;
        auto& pose_extrapolator_config =
            trajectory_config.pose_extrapolator_config;
        auto& submap_config = trajectory_config.submap_config;
        trajectory_config.collate_fixed_frame = true;
        trajectory_config.collate_landmarks = false;

        adaptive_voxel_filter_config = {0.5, 200.0, 50.0};

        loop_closure_adaptive_voxel_filter_config = {0.9, 100, 50.0};

        real_time_correlative_scan_matcher_config = {0.1, 20.0, 0.1, 0.1};

        // ceres_scan_matcher_config
        {
          ceres_scan_matcher_config.occupied_space_weight = 1.0;
          ceres_scan_matcher_config.translation_weight = 10.0;
          ceres_scan_matcher_config.rotation_weight = 40.0;
          auto& ceres_solver_options =
              ceres_scan_matcher_config.ceres_solver_config;
          ceres_solver_options = {false, 20, 2};
        }
        motion_filter_config = {2.0, 0.1, 5};

        // pose_extrapolator_config
        {
          pose_extrapolator_config.use_imu_based = false;
          auto& constant_velocity_config =
              pose_extrapolator_config.constant_velocity_config;
          auto& imu_based_config = pose_extrapolator_config.imu_based_config;

          constant_velocity_config = {10.0, 0.001};
          {
            imu_based_config.pose_queue_duration = 5;
            imu_based_config.gravity_constant = 10.0;
            imu_based_config.pose_translation_weight = 1.0;
            imu_based_config.pose_rotation_weight = 1.0;
            imu_based_config.imu_acceleration_weight = 1.0;
            imu_based_config.imu_rotation_weight = 1.0;
            imu_based_config.odometry_translation_weight = 1.0;
            imu_based_config.odometry_rotation_weight = 1.0;
            auto& ceres_solver_options = imu_based_config.ceres_solver_config;
            ceres_solver_options = {false, 10, 1};
          }
        }

        // submap_config
        {
          submap_config.num_range_data = 50;
          submap_config.grid_type = 1;
          submap_config.resolution = 0.04;
          auto& range_data_inserter_config =
              submap_config.range_data_inserter_config;

          // range_data_inserter_config
          {
            range_data_inserter_config.range_data_inserter_type = 1;
            range_data_inserter_config
                .probability_grid_range_data_inserter_config = {true, 0.55,
                                                                0.45};
            range_data_inserter_config.tsdf_range_data_inserter_config = {
                0.3, 10, true, 4, 0.5, true, 0, 0.5, 0.5};
          }
        }
      }

      // pose_graph_config
      {
        pose_graph_config.optimize_every_n_nodes = 50;
        auto& constraint_builder_config =
            pose_graph_config.constraint_builder_config;
        pose_graph_config.matcher_translation_weight = 0.0016;
        pose_graph_config.matcher_rotation_weight = 0.0016;
        auto& optimization_problem_config =
            pose_graph_config.optimization_problem_config;
        pose_graph_config.max_num_final_iterations = 200;
        pose_graph_config.global_sampling_ratio = 0.003;
        pose_graph_config.log_residual_histograms = false;
        pose_graph_config.global_constraint_search_after_n_seconds = 10;

        // constraint_builder_config
        {
          constraint_builder_config.sampling_ratio = 0.25;
          constraint_builder_config.max_constraint_distance = 6;
          constraint_builder_config.min_score = 0.7;
          constraint_builder_config.global_localization_min_score = 0.7;
          constraint_builder_config.loop_closure_translation_weight = 0.00001;
          constraint_builder_config.loop_closure_rotation_weight = 0.00001;
          constraint_builder_config.log_matches = true;
          auto& fast_correlative_scan_matcher_config =
              constraint_builder_config.fast_correlative_scan_matcher_config;
          auto& ceres_scan_matcher_config =
              constraint_builder_config.ceres_scan_matcher_config;

          fast_correlative_scan_matcher_config = {10.0, 30.0, 7};

          // ceres_scan_matcher_config
          {
            ceres_scan_matcher_config.occupied_space_weight = 20;
            ceres_scan_matcher_config.translation_weight = 10.0;
            ceres_scan_matcher_config.rotation_weight = 5.0;
            auto& ceres_solver_config =
                ceres_scan_matcher_config.ceres_solver_config;
            ceres_solver_config = {true, 9, 2};
          }
        }

        // optimization_problem_config
        {
          optimization_problem_config.huber_scale = 1;
          optimization_problem_config.rotation_weight = 0.00003;
          optimization_problem_config.acceleration_weight = 0.001;
          optimization_problem_config.local_slam_pose_translation_weight =
              0.00001;
          optimization_problem_config.local_slam_pose_rotation_weight = 0.00001;
          optimization_problem_config.odometry_translation_weight = 0.00001;
          optimization_problem_config.odometry_rotation_weight = 0.00001;
          optimization_problem_config.fixed_frame_pose_translation_weight = 0.1;
          optimization_problem_config.fixed_frame_pose_rotation_weight = 0.02;
          optimization_problem_config.fixed_frame_pose_use_tolerant_loss =
              false;
          optimization_problem_config.fixed_frame_pose_tolerant_loss_param_b =
              1;
          optimization_problem_config.fixed_frame_pose_tolerant_loss_param_a =
              1;
          optimization_problem_config.log_solver_summary = false;
          optimization_problem_config.use_online_imu_extrinsics_in_3d = true;
          optimization_problem_config.fix_z_in_3d = false;
          auto& ceres_solver_config =
              optimization_problem_config.ceres_solver_config;
          ceres_solver_config = {false, 50, 7};
        }
      }
    }
    // location_config
    config.location_config = {true, 0.0, 19.0, 1.0, 10.0,  10.0,
                              2,    10,  19.0, 0.5, 0.1,   0.1,
                              5,    60,  0,    0,   false, "./map"};

    //对赋值后的参数进行编码并写入json文件
    string s = ConfigUtils::ecode(config);
    std::string mapping_and_location_config_dir =
        config_dir + "mappingandlocation.json";
    auto f = fopen(mapping_and_location_config_dir.c_str(), "w+");
    auto r2 = fprintf(f, s.c_str());
    fclose(f);
  }
  // try {
  //       std::filesystem::create_directories(config_dir);
  //   } catch (const std::exception& e) {
  //       std::cerr << "Error creating directory: " << e.what() << std::endl;
  //   }
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
