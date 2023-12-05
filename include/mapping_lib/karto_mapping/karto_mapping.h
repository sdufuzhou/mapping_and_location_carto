/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 11:10:15
 * @LastEditTime: 2022-10-25 15:55:35
 * @Author: lcfc-desktop
 */
#pragma once

#include <assert.h>
#include <pthread.h>

#include <list>
#include <memory>

#include "include/config_struct.h"
#include "include/common/logger.h"
#include "include/mapping_lib/karto_mapping/slam_kar.h"
#include "include/mapping_lib/mapping_interface.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"

#define FLUSH_ODOM_LIST_SIZE 200

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class KartoMapping : public MappingInterface {
 public:
  using Position = gomros::message::Position;
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Logger = gomros::common::Logger;

  KartoMapping(const KartoMappingConfig &config);
  ~KartoMapping();

  virtual void SetConfiguration(void *config);

  // 处理里程计数据
  virtual void HandleOdomData(const OdometerMessage &data);
  // 处理雷达数据
  virtual void HandleLaserData(const RadarSensoryMessage &data);
  // 处理Imu数据
  virtual void HandleImuData(const ImuSensoryMessage &data);

  // 开始建图
  virtual void StartMapping();
  // 停止建图
  virtual void StopMapping(std::string map_name);

  virtual bool IsFinishMapping();

 private:
  void AddOdomDataRaw(const Position &pose_change);

  void FlushOdomDataRaw();

  Position ExpolateCurrentPositionTimeRaw(uint64_t timestamp);

  void RecordMapRawData(const RadarSensoryMessage &data);  //录制一次数据

  static void *RecordDataFunc(void *ptr);  //线程函数

  static void *MappingFunc(void *ptr);  //线程函数

  KartoMappingConfig m_MappingConfig;

  float laser_min_angle_;
  float laser_max_angle_;
  float laser_min_range_;
  float laser_max_range_;
  float laser_resolution_;
  float angle_increment_;
  int stepIncreament_;
  bool first_laser_ = true;
  bool record_data_ = false;
  bool record_finish_ = true;
  bool file_open_ = false;
  bool start_mapping_ = false;
  bool finish_mapping_ = false;

  FILE *m_pMapRawDataFile;

  std::list<Position> m_OdomListRaw;
  std::list<RadarSensoryMessage> m_RidarList;
  std::list<Position> m_PoseList;  // NO
  pthread_mutex_t mutex_odom_list_raw;
  pthread_mutex_t mutex_pose_raw;
  pthread_mutex_t mutex_ridar_list_raw;
  pthread_mutex_t mutex_pose_list_raw;
  pthread_t record_data_thread_;
  pthread_t mapping_thread_;

  // 一些状态变量
  int m_RecordIndex;
  Position m_MappingOdom;        // 建图里程计
  Position m_LastRecordPose;     // 上一次记录的位姿
  Position m_CurrentRecordPose;  // 当前位姿
  bool m_FirstRecord;            // 是否第一次录制
  std::string map_name_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
