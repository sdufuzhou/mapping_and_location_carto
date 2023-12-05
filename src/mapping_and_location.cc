/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-30 22:16:15
 * @LastEditTime: 2023-03-30 02:55:17
 */
#include "include/mapping_and_location/mapping_and_location.h"
#include "include/mapping_and_location_impl.h"
#include "include/config_struct.h"
#include "include/common/load_config.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
/**
 * @brief Construct a new Mapping And Location:: Mapping And Location
 * object（对象）
 *
 * @param config
 */
MappingAndLocation::MappingAndLocation(std::string config_dir) {
  DefaultWriteMappingAndLocationConfig(config_dir);
  MappingAndLocationConfig config;
  if (ReadMappingAndLocationConfig(config_dir, &config)) {
    impl_ = std::make_shared<MappingAndLocationImpl>(config);
  } else {
    std::cout << "配置文件加載失败！！！！！！" << std::endl;
  }
  
}

/**
 * @brief Destroy the Mapping And Location:: Mapping And Location object
 *
 */
MappingAndLocation::~MappingAndLocation() {}



/**
 * @brief 从文件加载初始位姿函数
 *
 */
void MappingAndLocation::LoadPoseFromFile() { impl_->LoadPoseFromFile(); }

/**
 * @brief 从文件加载地图数据函数
 *
 * @param map_file_name 包含路径和后缀的地图文件名
 * @return true 加载成功
 * @return false 加载失败
 */
bool MappingAndLocation::LoadMapDataFromFile(std::string map_file_name) {
  return impl_->LoadMapDataFromFile(map_file_name);
}

/**
 * @brief 开始建图
 *
 */
void MappingAndLocation::StartMapping() { 
  //加打印信息----
  impl_->StartMapping(); }

/**
 * @brief 停止建图
 *
 * @param map_name 地图名，不带后缀
 */
void MappingAndLocation::StopMapping(std::string map_name) {
  impl_->StopMapping(map_name);
}

void MappingAndLocation::StartChargeRelocate() { impl_->StartChargeRelocate(); }

/**
 * @brief 设定初始位姿
 *
 * @param init_pose
 */
void MappingAndLocation::SetInitPose(gomros::message::Position init_pose) {
  impl_->SetInitPose(init_pose);
}
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
