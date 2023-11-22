/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-25 10:37:44
 * @LastEditTime: 2022-10-26 20:25:20
 * @Author: lcfc-desktop
 */
#ifndef PLUGINS_LOCATION_LIB_INCLUDE_LOCATION_INTERFACE_H_
#define PLUGINS_LOCATION_LIB_INCLUDE_LOCATION_INTERFACE_H_

#include <memory>

#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class LocationInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using Position = gomros::message::Position;

  // 处理里程计数据
  virtual void HandleOdomData(const OdometerMessage &data) = 0;
  // 处理雷达数据
  virtual void HandleLaserData(const RadarSensoryMessage &data) = 0;

  // 设置配置参数
  virtual void SetConfiguration(void *config) = 0;

  // 设置初始位姿
  virtual void SetInitPose(float x, float y, float theta) = 0;

  // 开始全局定位
  virtual bool StartGlobalLocating(bool relocate) = 0;
 
  // 设置地图数据
  virtual void SetMapData(
      std::shared_ptr<gomros::message::SimpleGridMap> grid_map) = 0;

  // 推测当前位姿
  virtual Position ExpolateCurrentPositionTime(uint64_t timestamp) = 0;

  // 获取当前位姿
  virtual Position GetCurrentPosition() = 0;

  // 获取定位是否完成标志位
  virtual bool IsFinishLocate() = 0;

  virtual void StopLocate() = 0;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

#endif  // PLUGINS_LOCATION_LIB_INCLUDE_LOCATION_INTERFACE_H_
