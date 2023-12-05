/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 10:07:51
 * @LastEditTime: 2022-10-25 15:35:48
 * @Author: lcfc-desktop
 */

#pragma once

#define DEL(p)        \
  if (p != nullptr) { \
    delete p;         \
    p = nullptr;      \
  }

#include <assert.h>
#include <string.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include "common_lib/log.h"
#include "common_lib/node.h"
#include "include/common/logger.h"
#include "include/config_struct.h"
#include "include/mapping_lib/carto_mapping/carto_mapping.h"
#include "include/mapping_lib/karto_mapping/karto_mapping.h"
#include "include/mapping_lib/mapping_interface.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"
#include "message_lib/imu_message.h"  //---------------------新加--------------------------
#include "message_lib/odometer_message.h"
#include "message_lib/radar_message.h"

#define PRINTF_TIME_DURATION 2

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class MappingManagerImpl {
 public:
  using Node = gomros::common::Node;
  using RawPublisher = gomros::common::RawPublisher;
  using OdometerMessage = gomros::message::OdometerMessage;
  using ImuSensoryMessage = gomros::message::
      ImuSensoryMessage;  //------------------新加-----------------
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using Position = gomros::message::Position;
  using CmdData = gomros::message::CmdData;
  using CMDTYPE = gomros::message::CMDTYPE;
  using MAP_CMD_TYPE = gomros::message::MAP_CMD_TYPE;
  using DeviceState = gomros::message::DeviceState;
  using Logger = gomros::common::Logger;

  explicit MappingManagerImpl(const CartoMappingConfig &config,
                              std::shared_ptr<DeviceState> device_state);
                              
  explicit MappingManagerImpl(const KartoMappingConfig &config,
                              std::shared_ptr<DeviceState> device_state);

  ~MappingManagerImpl();
  void SetConfiguration(const CartoMappingConfig &config);
  void SetRadarData(
      const RadarSensoryMessage &data);  //---------------就是接收数据----------
  void SetOdomData(const OdometerMessage &data);
  void SetImuData(
      const ImuSensoryMessage &data);  //----------------新加-------------
  void SetDeviceState(
      std::shared_ptr<gomros::message::DeviceState> device_state);

  Position GetPosition();
  bool StartMapping();
  void StopMapping(std::string map_name);

 private:  //线程函数，真正处理数据的函数
  static void *HandleOdomData(void *ptr);  // 处理里程计数据

  static void *HandleLaserData(void *ptr);  // 处理雷达数据

  static void *HandlemuData(void *ptr);  // --------------处理Imu数据-----------

  //线程互斥锁声明
  pthread_mutex_t mutex_radar_data;
  pthread_mutex_t
      mutex_odom_data;  //在多个不同的线程中，读、写都需要加锁，且在线程中声明变量
  pthread_mutex_t mutex_imu_data;  //--------------------imu-----------------
  pthread_mutex_t mutex_pose;

  bool have_odom_data_ = false;
  bool have_radar_data_ = false;
  bool have_imu_data_ = false;  //--------imu----------//
  bool is_stop_mapping_ = false;

  RadarSensoryMessage last_radar_data;
  OdometerMessage last_odom_data;
  ImuSensoryMessage last_imu_data;  //----------------imu---------------

  DeviceState robot_state_;
  std::shared_ptr<DeviceState> device_state_;

  MappingInterface *mp_MappingModule =
      nullptr;  //子类为CartoMapping，多态的应用

  //线程声明
  pthread_t handle_radar_thtead;
  pthread_t handle_odom_thread;
  pthread_t hande_imu_thread;  //-----------imu------------//
  Position m_MappingOdom;      //------------建图里程计---------
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
