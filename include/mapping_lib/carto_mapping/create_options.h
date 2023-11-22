#pragma once

#include <memory>
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "include/mapping_and_location/config_struct.h"

// map_builder_interface.h又包含上面两个文件
namespace gomros {
namespace data_process {
namespace mapping_and_location {
using LocalTrajectoryBuilderOptions2D =
    ::cartographer::mapping::proto::LocalTrajectoryBuilderOptions2D;
using PoseGraphOptions = ::cartographer::mapping::proto::PoseGraphOptions;

cartographer::mapping::proto::MapBuilderOptions
GetMapBuilderOptions(const CartoMappingConfig &m_MappingConfig);

cartographer::mapping::proto::TrajectoryBuilderOptions
GetTrajectoryBuilderOptions(const CartoMappingConfig &m_MappingConfig);

//前端
void GetAdaptiveVoxelFilterOptions(LocalTrajectoryBuilderOptions2D *p,
                                   const CartoMappingConfig &m_MappingConfig);
void GetLoopClosureAdaptiveVoxelFilterOptions(
    LocalTrajectoryBuilderOptions2D *p,
    const CartoMappingConfig &m_MappingConfig);
void GetRtCsmOptions(LocalTrajectoryBuilderOptions2D *p,
                     const CartoMappingConfig &m_MappingConfig);
void GetCeresScanMatcherOptions(LocalTrajectoryBuilderOptions2D *p,
                                const CartoMappingConfig &m_MappingConfig);
void GetMotionFilterOptions(LocalTrajectoryBuilderOptions2D *p,
                            const CartoMappingConfig &m_MappingConfig);
void GetPoseExp(LocalTrajectoryBuilderOptions2D *p,
                const CartoMappingConfig &m_MappingConfig);
void GetSubMapOptions(LocalTrajectoryBuilderOptions2D *p,
                      const CartoMappingConfig &m_MappingConfig);
//后端

void GetConstraintBuilderOptions(PoseGraphOptions *pose_graph_options,
                                 const CartoMappingConfig &m_MappingConfig);
void GetOptimizationProblemOptions(PoseGraphOptions *pose_graph_options,
                                   const CartoMappingConfig &m_MappingConfig);

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
