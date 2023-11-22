/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-18 09:21:41
 * @LastEditTime: 2023-03-31 22:19:09
 * @Author: lcfc-desktop
 */
/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-25 09:59:27
 * @LastEditTime: 2022-08-16 17:11:19
 * @Author: lcfc-desktop
 */

#include "include/location_lib/location_manager_impl.h"

#include <fstream>
#include <string>
#include <vector>

#include "common_lib/message.h"
#include "common_lib/time_utils.h"
#include "include/location_lib/reflector_location.h"
#include "message_lib/simple_grid_map.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

LocationManagerImpl::LocationManagerImpl(
    const GridLocationConfig &config,
    std::shared_ptr<DeviceState> device_state) {
  init_pose_file_path = config.init_pose_file_path;
  device_state_ = device_state;
  raw_odom_pose_.mclDeltaPosition.mlTimestamp = 0;
  raw_odom_pose_.mclDeltaPosition.mfX = 0;
  raw_odom_pose_.mclDeltaPosition.mfY = 0;
  raw_odom_pose_.mclDeltaPosition.mfTheta = 0;
  last_radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp = 0;
  last_odom_data.mclDeltaPosition.mlTimestamp = 0;
  m_RobotStopCnt = 0;
  mp_location_module_ = new GridLocation(config);
  pthread_mutex_init(&mutex_odom_list, nullptr);
  pthread_mutex_init(&mutex_state, nullptr);
  pthread_mutex_init(&mutex_map, nullptr);
  pthread_mutex_init(&mutex_pose, nullptr);
  pthread_mutex_init(&mutex_radar_data, nullptr);
  pthread_mutex_init(&mutex_odom_data, nullptr);
  pthread_mutex_init(&mutex_total_odom, nullptr);
  pthread_mutex_init(&mutex_have_map, nullptr);
  pthread_mutex_init(&mutex_have_pose, nullptr);
  SLAM_INFO("当前定位模式为栅格定位\n");
  // pthread_create(&handle_odom_thread, NULL, HandleOdomDataThread, this);
  pthread_create(&handle_radar_thtead, NULL, HandleLaserDataThread, this);
}

/**
 * @brief Destroy the Location Manager:: Location Manager object
 *
 */
LocationManagerImpl::~LocationManagerImpl() {
  DEL(mp_location_module_);
  // pthread_join(handle_odom_thread, NULL);
  pthread_join(handle_radar_thtead, NULL);
  pthread_mutex_destroy(&mutex_odom_list);
  pthread_mutex_destroy(&mutex_state);
  pthread_mutex_destroy(&mutex_map);
  pthread_mutex_destroy(&mutex_odom_data);
  pthread_mutex_destroy(&mutex_pose);
  pthread_mutex_destroy(&mutex_radar_data);
  pthread_mutex_destroy(&mutex_total_odom);
  pthread_mutex_destroy(&mutex_have_map);
  pthread_mutex_destroy(&mutex_have_pose);
}

void LocationManagerImpl::SetConfiguration(const GridLocationConfig &config) {
  init_pose_file_path = config.init_pose_file_path;
  mp_location_module_->SetConfiguration((void *)(&config));
}

void LocationManagerImpl::StopLocate() { mp_location_module_->StopLocate(); }

/**
 * @brief 开始全局定位
 *
 */
void LocationManagerImpl::StartGlobalLocating(bool relocate) {
  m_PoseInitilized = false;
  SLAM_INFO("栅格定位收到的初始定位位姿x= %f, y=%f, theta=%f\n", m_InitPose.mfX,
            m_InitPose.mfY, m_InitPose.mfTheta);
  UpdateLastPosition(m_InitPose);
  raw_odom_pose_.mclDeltaPosition = m_InitPose;
  while (radar_disconnect) {
    SLAM_WARN("LocationManagerImpl: 雷达尚未连接！等待连接......\n");
    usleep(100000);
  }
  SLAM_INFO("LocationManagerImpl: 雷达已连接......\n");
  SLAM_INFO("width=%d,height=%d\n", m_pGridMap->map_info.miMapWidth,
            m_pGridMap->map_info.miMapHeight);
  mp_location_module_->StartGlobalLocating(relocate);
  pthread_create(&m_MonitorGlobalLocateThread, NULL,
                 MonitorGlobalLocateThreadFunction, this);
}

/**
 * @brief 将里程计数据队列的时间戳小于传入时间戳的数据全部舍弃
 *
 * @param timestamp_us
 */
void LocationManagerImpl::UpdateOdomListTimestamp(uint64_t timestamp_us) {
  pthread_mutex_lock(&mutex_odom_list);
  while (!m_OdomList.empty()) {
    if (m_OdomList.front().mlTimestamp > timestamp_us) {
      break;
    }
    m_OdomList.pop_front();
  }
  pthread_mutex_unlock(&mutex_odom_list);
}

/**
 * @brief 监控定位是否完成的线程
 *
 * @param param
 * @return void*
 */

void *LocationManagerImpl::MonitorGlobalLocateThreadFunction(void *param) {
  pthread_detach(pthread_self());
  LocationManagerImpl *pc = reinterpret_cast<LocationManagerImpl *>(param);
  pc->DoGlobalLocateMonitor();
  return nullptr;
}

void LocationManagerImpl::SetMapData(SimpleGridMap &simple_grid_map) {
  pthread_mutex_lock(&mutex_map);
  m_pGridMap = std::make_shared<SimpleGridMap>(simple_grid_map);
  mp_location_module_->SetMapData(m_pGridMap);
  SetHaveMap(true);
  pthread_mutex_unlock(&mutex_map);
}

void LocationManagerImpl::SetIniPose(const Position &pose) {
  m_InitPose = pose;
  mp_location_module_->SetInitPose(m_InitPose.mfX, m_InitPose.mfY,
                                   m_InitPose.mfTheta);
  SetHavePose(true);
}

void LocationManagerImpl::SetHaveMap(bool have_map) {
  pthread_mutex_lock(&mutex_have_map);
  have_map_ = have_map;
  pthread_mutex_unlock(&mutex_have_map);
}

bool LocationManagerImpl::FinishLocate() { return m_PoseInitilized; }

void LocationManagerImpl::SetHavePose(bool have_pose) {
  pthread_mutex_lock(&mutex_have_pose);
  have_init_pose_ = have_pose;
  pthread_mutex_unlock(&mutex_have_pose);
}

void LocationManagerImpl::SetRadarData(const RadarSensoryMessage &data) {
  pthread_mutex_lock(&mutex_radar_data);
  new_radar_data_ = true;
  radar_disconnect = false;
  last_radar_data = data;
  pthread_mutex_unlock(&mutex_radar_data);
}

void LocationManagerImpl::SetOdomData(const OdometerMessage &data) {
  have_odom_data_ = true;
  last_odom_data = data;
  // raw_odom_pose_ = data;
  raw_odom_pose_.mclDeltaPosition =
      raw_odom_pose_.mclDeltaPosition * data.mclDeltaPosition;
  HandleOdomData(data);
}

void LocationManagerImpl::ResetPose() {
  SLAM_INFO("检测到进入建图状态,里程计置0!\n");
  FlushOdomData();
  pthread_mutex_lock(&mutex_pose);
  m_CurrentPose.mfX = 0.0;
  m_CurrentPose.mfY = 0.0;
  m_CurrentPose.mfTheta = 0.0;
  m_CurrentPose.mlTimestamp = gomros::common::GetCurrentTime_us();
  raw_odom_pose_.mclDeltaPosition.mfX = 0.0;
  raw_odom_pose_.mclDeltaPosition.mfY = 0.0;
  raw_odom_pose_.mclDeltaPosition.mfTheta = 0.0;
  raw_odom_pose_.mclDeltaPosition.mlTimestamp =
      gomros::common::GetCurrentTime_us();
  pthread_mutex_unlock(&mutex_pose);
}

void LocationManagerImpl::SetTotalOdom(double total_odom) {
  pthread_mutex_lock(&mutex_total_odom);
  m_TotalOdom = total_odom;
  pthread_mutex_unlock(&mutex_total_odom);
}

void LocationManagerImpl::SetDeviceState(
    std::shared_ptr<gomros::message::DeviceState> device_state) {
  device_state_ = device_state;
}

double LocationManagerImpl::GetTotalOdom() {
  pthread_mutex_lock(&mutex_total_odom);
  double total_odom = m_TotalOdom;
  pthread_mutex_unlock(&mutex_total_odom);
  return total_odom;
}

gomros::message::OdometerMessage LocationManagerImpl::GetRawOdomPose() {
  OdometerMessage odom_pose = last_odom_data;
  odom_pose.mclDeltaPosition = m_CurrentPose;
  return raw_odom_pose_;
}

gomros::message::OdometerMessage LocationManagerImpl::GetCurrentPose() {
  pthread_mutex_lock(&mutex_pose);
  OdometerMessage odom_pose = last_odom_data;
  odom_pose.mclDeltaPosition = m_CurrentPose;
  pthread_mutex_unlock(&mutex_pose);
  return odom_pose;
}

/**
 * @brief
 *
 * @param data
 */
void *LocationManagerImpl::HandleOdomDataThread(void *ptr) {
  LocationManagerImpl *lp_this = reinterpret_cast<LocationManagerImpl *>(ptr);
  OdometerMessage odom_data;
  int no_data_cnt = 0;
  while (1) {
    if (lp_this->device_state_->mcChassisState != SYSTEM_STATE_MAPPING) {
      pthread_mutex_lock(&(lp_this->mutex_have_map));
      pthread_mutex_lock(&(lp_this->mutex_have_pose));
      if (lp_this->have_odom_data_ && lp_this->have_init_pose_ &&
          lp_this->have_map_) {
        pthread_mutex_unlock(&(lp_this->mutex_have_pose));
        pthread_mutex_unlock(&(lp_this->mutex_have_map));
        pthread_mutex_lock(&(lp_this->mutex_odom_data));
        if (!lp_this->odom_data_list.empty()) {
          no_data_cnt = 0;
          odom_data = lp_this->odom_data_list.front();
          lp_this->odom_data_list.pop_front();
          pthread_mutex_unlock(&(lp_this->mutex_odom_data));
          lp_this->mp_location_module_->HandleOdomData(odom_data);
          Position pose = odom_data.mclDeltaPosition;
          lp_this->AddOdomData(pose);
          double delta_odom = sqrt(pose.mfX * pose.mfX + pose.mfY * pose.mfY);
          pthread_mutex_lock(&(lp_this->mutex_total_odom));
          lp_this->m_TotalOdom += delta_odom;
          lp_this->m_TodayOdom += delta_odom;
          pthread_mutex_unlock(&(lp_this->mutex_total_odom));

          // 通过里程计的移动量判断小车是否移动 ???
          if (fabs(pose.mfX) <= STOPPING_MAX_DISTANCE &&
              fabs(pose.mfY) <= STOPPING_MAX_DISTANCE &&
              fabs(pose.mfTheta) <= STOPPING_MAX_ANGLE) {
            if (lp_this->m_RobotStopCnt < MAX_STOP_CNT) {
              lp_this->m_RobotStopCnt++;
            } else {
              lp_this->m_RobotStopCnt = 0;
              lp_this->is_static_ = true;
            }
          } else {
            lp_this->m_RobotStopCnt = 0;
            lp_this->is_static_ = false;
          }
          if (lp_this->m_OdomList.size() > FLUSH_ODOM_LIST_SIZE)
            lp_this->FlushOdomData();
        } else {
          pthread_mutex_unlock(&(lp_this->mutex_odom_data));
          if ((lp_this->device_state_->mcChassisState == SYSTEM_STATE_FREE) &&
              (lp_this->device_state_->mcChassisState ==
               SYSTEM_STATE_INIT_POSE) &&
              (lp_this->device_state_->mcChassisState ==
               SYSTEM_STATE_LOCATING)) {
            no_data_cnt++;
            if (no_data_cnt >= PRINTF_TIME_DURATION * 50) {
              SLAM_WARN("定位模块里程计数据无更新\n");
              no_data_cnt = 0;
            }
          }
        }
      } else {
        pthread_mutex_unlock(&(lp_this->mutex_have_map));
        pthread_mutex_unlock(&(lp_this->mutex_have_pose));
      }
    }

    usleep(20000);
  }
}

/**
 * @brief
 *
 * @param data
 */
void *LocationManagerImpl::HandleLaserDataThread(void *ptr) {
  LocationManagerImpl *lp_this = reinterpret_cast<LocationManagerImpl *>(ptr);
  RadarSensoryMessage radar_data;
  radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp = 0;
  while (1) {
    pthread_mutex_lock(&(lp_this->mutex_have_map));
    pthread_mutex_lock(&(lp_this->mutex_have_pose));
    pthread_mutex_lock(&(lp_this->mutex_radar_data));
    if (lp_this->new_radar_data_ && lp_this->have_init_pose_ &&
        lp_this->have_map_) {
      lp_this->new_radar_data_ = false;
      radar_data = lp_this->last_radar_data;
      pthread_mutex_unlock(&(lp_this->mutex_radar_data));
      lp_this->mp_location_module_->HandleLaserData(radar_data);
      // 如果正在初始化定位则不在此更新定位信息
      Position pose_scan =
          lp_this->mp_location_module_->ExpolateCurrentPositionTime(
              radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
      // 如果正在初始化定位则不在此更新定位信息
      if (lp_this->device_state_->mcChassisState != SYSTEM_STATE_INIT_POSE) {
        // lp_this->UpdateLastPosition(pose_scan);
        pthread_mutex_lock(&(lp_this->mutex_pose));
        lp_this->m_CurrentPose = pose_scan;
        pthread_mutex_unlock(&(lp_this->mutex_pose));
        lp_this->m_SavePoseCnt++;
        if (lp_this->m_SavePoseCnt > 20) {
          lp_this->SavePoseToServer();
          lp_this->m_SavePoseCnt = 0;
        }
      }
    } else {
      pthread_mutex_unlock(&(lp_this->mutex_radar_data));
    }
    pthread_mutex_unlock(&(lp_this->mutex_have_map));
    pthread_mutex_unlock(&(lp_this->mutex_have_pose));
    usleep(50000);
  }
}

void LocationManagerImpl::HandleOdomData(OdometerMessage odom_data) {
  if (have_odom_data_ && have_init_pose_ && have_map_) {
    mp_location_module_->HandleOdomData(odom_data);
    // Position pose_scan = mp_location_module_->ExpolateCurrentPositionTime(
    //     odom_data.mclDeltaPosition.mlTimestamp);
    // // 如果正在初始化定位则不在此更新定位信息
    // if (device_state_->mcChassisState != SYSTEM_STATE_INIT_POSE) {
    //   UpdateLastPosition(pose_scan);
    // }
    Position pose = odom_data.mclDeltaPosition;
    AddOdomData(pose);
    double delta_odom = sqrt(pose.mfX * pose.mfX + pose.mfY * pose.mfY);
    pthread_mutex_lock(&(mutex_total_odom));
    pthread_mutex_unlock(&(mutex_total_odom));
    // 通过里程计的移动量判断小车是否移动 ???
    if (fabs(pose.mfX) <= STOPPING_MAX_DISTANCE &&
        fabs(pose.mfY) <= STOPPING_MAX_DISTANCE &&
        fabs(pose.mfTheta) <= STOPPING_MAX_ANGLE) {
      if (m_RobotStopCnt < MAX_STOP_CNT) {
        m_RobotStopCnt++;
      } else {
        m_RobotStopCnt = 0;
        is_static_ = true;
      }
    } else {
      m_RobotStopCnt = 0;
      is_static_ = false;
      m_TotalOdom += delta_odom;
      m_TodayOdom += delta_odom;
    }
    if (m_OdomList.size() > FLUSH_ODOM_LIST_SIZE) FlushOdomData();
  }
}

void LocationManagerImpl::HandleRadarData(RadarSensoryMessage radar_data) {
  if (new_radar_data_ && have_init_pose_ && have_map_) {
    // is_handling_radar_data = true;
    Position pose_scan;
    mp_location_module_->HandleLaserData(radar_data);
    pose_scan = mp_location_module_->ExpolateCurrentPositionTime(
        radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
    // 如果正在初始化定位则不在此更新定位信息
    if (device_state_->mcChassisState != SYSTEM_STATE_INIT_POSE) {
      m_SavePoseCnt++;
      if (m_SavePoseCnt > 20) {
        SavePoseToServer();
        m_SavePoseCnt = 0;
      }
    }
    // is_handling_radar_data = false;
  } else {
  }
}

void LocationManagerImpl::SavePoseToServer() {
  Json::Value zero_pose_json;
  Json::Value zero_pose;
  pthread_mutex_lock(&mutex_pose);
  zero_pose["AgvX"] = m_CurrentPose.mfX;
  zero_pose["AgvY"] = m_CurrentPose.mfY;
  zero_pose["AgvTheta"] = m_CurrentPose.mfTheta;
  pthread_mutex_unlock(&mutex_pose);
  pthread_mutex_lock(&mutex_total_odom);
  zero_pose["Total_odom"] = m_TotalOdom;
  pthread_mutex_unlock(&mutex_total_odom);
  zero_pose_json["zero_pos"] = zero_pose;

  if (m_SavePoseFileIndex == 1) {
    std::string filename = init_pose_file_path + "/zero.json";
    json_save(filename.c_str(), zero_pose_json);
    m_SavePoseFileIndex++;
  } else {
    std::string filename = init_pose_file_path + "/zero1.json";
    json_save(filename.c_str(), zero_pose_json);
    m_SavePoseFileIndex = 1;
  }
}

void LocationManagerImpl::json_save(const char *file_name, Json::Value v) {
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(v);
  str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
  str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
  str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
  std::ofstream ofs(file_name, std::ios::trunc);
  ofs << str;
  ofs.close();
}

/**
 * @brief 监控初始定位是否完成，完成则更新当期位姿
 *
 */
void LocationManagerImpl::DoGlobalLocateMonitor() {
  sleep(1);
  while (mp_location_module_->IsFinishLocate() == false) {
    usleep(200000);
  }
  SLAM_INFO("重定位定位完成\n");
  Position locate_pose;
  locate_pose = mp_location_module_->GetCurrentPosition();
  locate_pose.mlTimestamp = gomros::common::GetCurrentTime_us();
  FlushOdomData();
  UpdateLastPosition(locate_pose);
  pthread_mutex_lock(&mutex_pose);
  m_CurrentPose = locate_pose;
  Position pose = locate_pose;
  pthread_mutex_unlock(&mutex_pose);
  SLAM_INFO("定位得到的初始位姿 x=%f, y=%f, theta=%f\n", pose.mfX, pose.mfY,
            pose.mfTheta);
  usleep(100000);
  m_PoseInitilized = true;
  device_state_->mcChassisState = SYSTEM_STATE_FREE;
}

/**
 * @brief 更新小车当前位姿,并将里程计数据队列的时间戳小于最新位姿的数据全部舍弃
 *
 * @param pose
 */
void LocationManagerImpl::UpdateLastPosition(const Position &pose) {
  pthread_mutex_lock(&mutex_pose);
  pthread_mutex_lock(&mutex_odom_data);
  if (pose.mlTimestamp > m_CurrentPose.mlTimestamp) {
    m_CurrentPose = pose;
    UpdateOdomListTimestamp(pose.mlTimestamp);
  }
  pthread_mutex_unlock(&mutex_odom_data);
  pthread_mutex_unlock(&mutex_pose);
}

/**
 * @brief 利用里程计数据更新小车位姿，将里程计数据存入队列
 *
 * @param pose_change
 */
void LocationManagerImpl::AddOdomData(const Position &pose_change) {
  uint64_t timestamp = pose_change.mlTimestamp;
  if (timestamp < m_CurrentPose.mlTimestamp) {
    SLAM_WARN("##LocationManager realtime data timestamp is too small\n");
    SLAM_WARN("里程计时间戳:%llu, 当前位姿时间戳:%llu\n", timestamp,
              m_CurrentPose.mlTimestamp);
    return;
  }
  Position pose = m_CurrentPose * pose_change;
  UpdateLastPosition(pose);
  pthread_mutex_lock(&mutex_odom_list);
  if (m_OdomList.empty()) {
    m_OdomList.push_back(pose_change);
  } else {
    for (auto iter = m_OdomList.begin();; iter++) {
      if (iter == m_OdomList.end() || iter->mlTimestamp >= timestamp) {
        m_OdomList.insert(iter, pose_change);
        break;
      }
    }
  }
  pthread_mutex_unlock(&mutex_odom_list);
}

/**
 * @brief 清空里程计数据队列，如果有数据时间戳晚于最新位姿，则更新最新位姿
 *
 */
void LocationManagerImpl::FlushOdomData() {
  pthread_mutex_lock(&mutex_pose);
  Position pose = m_CurrentPose;
  pthread_mutex_lock(&mutex_odom_list);

  while (!m_OdomList.empty()) {
    if (m_OdomList.front().mlTimestamp > pose.mlTimestamp) {
      pose = pose * m_OdomList.front();
    }
    m_OdomList.pop_front();
  }
  m_CurrentPose = pose;
  pthread_mutex_unlock(&mutex_odom_list);
  pthread_mutex_unlock(&mutex_pose);
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
