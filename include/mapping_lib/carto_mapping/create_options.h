#pragma once

#include <memory>
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "include/config_struct.h"

// map_builder_interface.h又包含上面两个文件
namespace gomros {
namespace data_process {
namespace mapping_and_location {
// using LocalTrajectoryBuilderOptions2D =
//     ::cartographer::mapping::proto::LocalTrajectoryBuilderOptions2D;
// using PoseGraphOptions = ::cartographer::mapping::proto::PoseGraphOptions;
using namespace ::cartographer::mapping::proto;
using namespace ::cartographer::sensor::proto;
using namespace ::cartographer::mapping::scan_matching::proto;
using namespace ::cartographer::common::proto;
using namespace ::cartographer::mapping::constraints::proto;
using namespace ::cartographer::mapping::optimization::proto;

cartographer::mapping::proto::MapBuilderOptions GetMapBuilderOptions(
    const CartoMappingConfig &mapping_config);

cartographer::mapping::proto::TrajectoryBuilderOptions
GetTrajectoryBuilderOptions(const TrajectoryBuilderConfig& trajectory_builder_config);


void SetPoseGraphOptions(const PoseGraphConfig pose_graph_config,
                         PoseGraphOptions *pose_graph_options);

void SetAdaptiveVoxelFilterOptions(
    const AdaptiveVoxelFilterConfig adaptive_voxel_filter_config,
    AdaptiveVoxelFilterOptions *adaptive_voxel_filter_options);

void SetLoopClosureAdaptiveVoxelFilterOptions(
    const LoopClosureAdaptiveVoxelFilterConfig
        loop_closure_adaptive_voxel_filter_config,
    AdaptiveVoxelFilterOptions *loop_closure_adaptive_voxel_filter_options);

void SetRealTimeCorrelativeScanMatcherOptions(
    const RealTimeCorrelativeScanMatcherConfig
        real_time_correlative_scan_matcher_config,
    RealTimeCorrelativeScanMatcherOptions
        *real_time_correlative_scan_matcher_options);

void SetCeresScanMatcherOptions(
    const CeresScanMatcherConfig ceres_scan_matcher_config,
    CeresScanMatcherOptions2D *ceres_scan_matcher_options);
void SetCeresSolverOptions(const CeresSolverConfig ceres_slover_config,
                           CeresSolverOptions *ceres_slover_options);

void SetMotionFilterOptions(const MotionFilterConfig montion_filter_config,
                            MotionFilterOptions *montion_filter_options);

void SetPoseExtrapolatorOptions(
    const PoseExtrapolatorConfig pose_extrapolator_config,
    PoseExtrapolatorOptions *pose_extrapolator_options);

void SetImuBasedOptions(const ImuBasedConfig imu_based_config,
                        ImuBasedPoseExtrapolatorOptions *imu_based_options);
void SetRangeDataInserterOptions(
    const RangeDataInserterConfig range_data_ionserter_config,
    RangeDataInserterOptions *range_data_inserter_options);
void SetProbabilityGridRangeDataInserterOptions2D(
    const ProbabilityGridRangeDataInserterConfig config,
    ProbabilityGridRangeDataInserterOptions2D *opions);

void SetTSDFRangeDataInserterOptions2D(const TsdfRangeDataInserterConfig config,
                                       TSDFRangeDataInserterOptions2D *options);

void SetSubmapOptions(const SubmapConfig submap_config,
                      SubmapsOptions2D *submap_options);

void SetConstraintBuilderOptions(const ConstraintBuilderConfig config,
                                 ConstraintBuilderOptions *options);

void SetFastCorrelativeScanMatcherOptions2DOptions(
    const FastCorrelativeScanMatcherConfig config,
    FastCorrelativeScanMatcherOptions2D *options);

void SetOptimizationProblemOptions(const OptimizationProblemConfig config,
                                   OptimizationProblemOptions *options);

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
