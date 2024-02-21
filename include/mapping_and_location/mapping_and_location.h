/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-10-01 16:42:13
 * @LastEditTime: 2022-10-25 16:06:50
 */
#pragma once
#include <memory>
#include <string>
#include "message_lib/position_message.h"//在大工程里
namespace gomros {
namespace data_process {
namespace mapping_and_location {
class MappingAndLocationImpl;
class MappingAndLocation {
 public:
  explicit MappingAndLocation(std::string config_dir);//新加
  ~MappingAndLocation();
  void LoadPoseFromFile();  // 从文件中加载初始位姿
  bool LoadMapDataFromFile(
      std::string map_file_name);  // 从指定地图文件中读取地图数据
  void StartMapping();             // 开始建图
  void StopMapping(std::string map_name);  // 停止建图
  void SetInitPose(gomros::message::Position init_pose); // 设置初始位姿
  void StartChargeRelocate();//重定位

 private:
  std::shared_ptr<MappingAndLocationImpl> impl_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
