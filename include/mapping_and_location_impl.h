/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-27 15:02:35
 * @LastEditTime: 2023-03-31 11:22:33
 * @Author: lcfc-desktop
 */
#pragma once
#include <jsoncpp/json/json.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common_lib/gomros.h"
#include "common_lib/log.h"
#include "include/location_lib/location_manager_impl.h"
#include "include/mapping_and_location/config_struct.h"
#include "include/mapping_lib/mapping_manage_impl.h"
#include "include/common/logger.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"
#include "message_lib/pose_and_odom_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/odometer_message.h"
#include "message_lib/imu_message.h"      
#include "message_lib/simple_grid_map.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
class MappingAndLocationImpl {
 public:
  using string = std::string;
  using RawPublisher = gomros::common::RawPublisher;
  using CallBackEvent = gomros::common::CallBackEvent;//包含了三个数据处理回调函数以及其他回调函数
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  using Position = gomros::message::Position;
  using Node = gomros::common::Node;

  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;

  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;//-----------------新加-------------------

  using DeviceState = gomros::message::DeviceState;
  using Logger = gomros::common::Logger;
  using CustomTaskString = gomros::message::CustomTaskString;
  typedef std::shared_ptr<SimpleGridMap> MapSharedPointer;
  explicit MappingAndLocationImpl(const KartoAndGridConfig &config);
  explicit MappingAndLocationImpl(const CartoAndGridConfig &config);
  ~MappingAndLocationImpl();
  void SetConfiguration(const CartoAndGridConfig &config);
  void LoadPoseFromFile();
  bool LoadMapDataFromFile(
      std::string map_file_name);  //从指定地图文件中读取地图数据
  void StartMapping();
  void StopMapping(std::string map_name);
  void SetInitPose(Position init_pose);
  void StartChargeRelocate();
  KartoAndGridConfig GetKartoAndGridConfig();
  CartoAndGridConfig GetCartoAndGridConfig();  
 private:
  void PublishRobotState();
  void JsonSave(const char *file_name,
                Json::Value v);  // 将JSON格式数据存入文件
  bool JsonRead(const char *file_name,
                std::string *read_data);  // 从文件中读取JSON格式数据
  void LoadGridMap(std::string map_data_);  // 由字符串数据生成地图

  void LoadMapNameFromFile();//加载已存在的地图来进行初始定位

  static void OdomCallBackFunc(void *object, char *buf, const int size);
  static void RadarCallBackFunc(void *object, char *buf, const int size);
  static void ImuCallBackFunc(void *object, char *buf, const int size);//-----------------------增加Imu回调函数

  static void StartLocateCallBack(void *object, char *buf, const int size);
  static void StartMappingCallBack(void *object, char *buf, const int size);
  static void StopMappingCallBack(void *object, char *buf, const int size);

  static void *SendPose(void *ptr);//-----------------应该是关于定位的
  std::shared_ptr<DeviceState> device_state_;
  OdometerMessage odom_data_;//-------------------这里并没有用上------------------

  Position radar_pose_base_link_;//雷达坐标系的位姿，相对于机器人坐标系

  KartoAndGridConfig karto_and_grid_config_;
  
  CartoAndGridConfig carto_and_grid_config_;//---------------------实例化结构体------------------

  MappingManagerImpl *mapping_module_ = nullptr;
  LocationManagerImpl *location_module = nullptr;

  MapSharedPointer map_shared_pointer_ = nullptr;  // 指向栅格地图的指针
  Node *node_ = nullptr;
  pthread_t send_pose_thread_;
  RawPublisher *state_publisher_;
  RawPublisher *pose_publisher_;
  pthread_t send_pose_thread;
  int recor_file_index_ = 1;
  Position initial_position_;//AGV的初始位姿
  Position charge_positipn_;//充电桩的位置
  bool have_charge_position = false;
  OdometerMessage current_position_;
  double x_min_;
  double y_min_;
  double x_max_;
  double y_max_;
  double total_odom_;
  string map_file_full_path_;
  std::string map_file_path;
  std::string init_pose_file_path;  // 初始位姿文件（机器人关机时侯的位置）存放路径,必须和地图管理模块的地图文件路径一样
  bool first_set_configuration = true;
};
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
