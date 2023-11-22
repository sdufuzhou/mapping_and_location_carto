/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-05 17:09:35
 * @LastEditTime: 2022-10-26 14:43:58
 * @Author: lcfc-desktop
 */
#pragma once

#include <string>
#include "message_lib/odometer_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/imu_message.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class MappingInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  // 处理里程计数据
  virtual void HandleOdomData(const OdometerMessage &data) = 0;
  // 处理雷达数据
  virtual void HandleLaserData(const RadarSensoryMessage &data) = 0;
  //处理IMU数据
   virtual void HandleImuData(const ImuSensoryMessage &data) = 0;//----------------新加-------------------
  // 开始建图
  virtual void StartMapping() = 0;
  // 停止建图
  virtual void StopMapping(std::string map_name) = 0;

  virtual void SetConfiguration(void *config) = 0;

  virtual bool IsFinishMapping() = 0;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
