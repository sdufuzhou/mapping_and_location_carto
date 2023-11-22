/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-18 09:21:18
 * @LastEditTime: 2022-11-14 16:12:56
 * @Author: lcfc-desktop
 */

#pragma once

#include <jsoncpp/json/json.h>
#include <pthread.h>
#include <unistd.h>

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "common_lib/log.h"
#include "common_lib/node.h"
#include "include/location_lib/grid_location.h"
#include "include/location_lib/location_interface.h"
#include "include/mapping_and_location/config_struct.h"
#include "include/common/logger.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

#define DEL(p)        \
  if (p != nullptr) { \
    delete p;         \
    p = nullptr;      \
  }
#define MAX_STOP_CNT 65535
#define FLUSH_ODOM_LIST_SIZE 200
#define STOPPING_MAX_ANGLE 0.0005
#define STOPPING_MAX_DISTANCE 0.0001
#define PRINTF_TIME_DURATION 2
#define MAX_SAVE_POSE_CNT 20

class LocationManagerImpl {
 public:
  using Node = gomros::common::Node;
  using RawPublisher = gomros::common::RawPublisher;
  using DeviceState = gomros::message::DeviceState;
  using OdometerMessage = gomros::message::OdometerMessage;
  using Position = gomros::message::Position;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using CmdData = gomros::message::CmdData;
  using CMDTYPE = gomros::message::CMDTYPE;
  using MAP_CMD_TYPE = gomros::message::MAP_CMD_TYPE;
  using Logger = gomros::common::Logger;
  typedef std::shared_ptr<SimpleGridMap> MapSharedPointer;

  explicit LocationManagerImpl(const GridLocationConfig &config,
                               std::shared_ptr<DeviceState> device_state);

  ~LocationManagerImpl();

  void SetConfiguration(const GridLocationConfig &config);

  void SetRadarData(const RadarSensoryMessage &data);

  void ResetPose();

  void SetOdomData(const OdometerMessage &data);

  void SetMapData(SimpleGridMap &simple_grid_map);

  void SetIniPose(const Position &pose);

  void SetTotalOdom(double total_odom);

  void SetDeviceState(
      std::shared_ptr<gomros::message::DeviceState> device_state);

  double GetTotalOdom();

  OdometerMessage GetCurrentPose();

  void SetHaveMap(bool have_map);

  void SetHavePose(bool have_pose);

  bool FinishLocate();

  void StartGlobalLocating(bool relocate);

  void StopLocate();

  OdometerMessage GetRawOdomPose();
 private:
  void SavePoseToServer();

  void AddOdomData(const Position &pose_change);

  void UpdateLastPosition(const Position &pose);

  static void *MonitorGlobalLocateThreadFunction(void *param);

  void DoGlobalLocateMonitor();

  void json_save(const char *file_name, Json::Value v);

  static void *HandleOdomDataThread(void *ptr);  // 处理里程计

  static void *HandleLaserDataThread(void *ptr);  // 处理雷达数据

  void HandleOdomData(OdometerMessage odom_data);

  void HandleRadarData(RadarSensoryMessage radar_data);

  void FlushOdomData();

  void UpdateOdomListTimestamp(uint64_t timestamp_us);

  LocationInterface *mp_location_module_;

  std::string init_pose_file_path;

  MapSharedPointer m_pGridMap;  // 地图指针

  Position m_CurrentPose;  // AGV现在的位姿

  OdometerMessage raw_odom_pose_;

  bool radar_disconnect = true;

  bool m_PoseInitilized = false;  // 外部判断是否完成初始化的标志

  bool is_ini_locate_ = true;

  bool have_init_pose_ = false;

  bool have_map_ = false;

  bool can_locate = true;

  bool new_radar_data_ = false;

  bool have_odom_data_ = false;

  bool is_handling_radar_data = false;

  bool is_static_ = true;

  RadarSensoryMessage last_radar_data;

  OdometerMessage last_odom_data;

  std::list<OdometerMessage> odom_data_list;

  // 一些状态变量
  int m_RobotStopCnt;
  bool m_TrackLocatePause = false;
  char robot_state;
  std::shared_ptr<DeviceState> device_state_;
  DeviceState m_RobotState;
  Position m_PoseFromSaveFile;  // 从地图管理类中读来的初始位姿
  Position m_InitPose;  // 外部调用初始化定位时传入的初始位姿
  int m_SavePoseCnt = 0;
  int m_SavePoseFileIndex = 1;
  double m_TotalOdom = 0;
  double m_TodayOdom = 0;
  std::list<Position> m_OdomList;  //

  pthread_mutex_t mutex_have_map;   //
  pthread_mutex_t mutex_have_pose;  //
  pthread_mutex_t mutex_map;        //
  pthread_mutex_t mutex_pose;       //
  pthread_mutex_t mutex_radar_data;
  pthread_mutex_t mutex_odom_data;
  pthread_mutex_t mutex_total_odom;
  pthread_mutex_t mutex_odom_list;  //
  pthread_mutex_t mutex_state;      //
  pthread_t m_InitialLocateThread;  // 开机初始化全局定位线程
  pthread_t handle_radar_thtead;
  pthread_t handle_odom_thread;
  pthread_t m_StartGlobalLocateThread;
  pthread_t m_MonitorGlobalLocateThread;  // 监视初始化定位是否完成

  // config param 配置参数
  GridLocationConfig m_LocationConfig;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
