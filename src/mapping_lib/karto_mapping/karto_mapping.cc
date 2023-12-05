/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 11:11:30
 * @LastEditTime: 2023-03-23 16:44:55
 * @Author: lcfc-desktop
 */

#include <fstream>
#include <string>
#include <vector>

#include "common_lib/time_utils.h"
#include "include/mapping_lib/karto_mapping/karto_mapping.h"
#include "include/mapping_lib/karto_mapping/slam_kar.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

KartoMapping::KartoMapping(const KartoMappingConfig &config) {
  m_MappingConfig = config;
  laser_min_angle_ =
      m_MappingConfig.mapping_start_angle / 180 * M_PI;  //转换为弧度
  laser_max_angle_ = m_MappingConfig.mapping_end_angle / 180 * M_PI;
  laser_resolution_ = m_MappingConfig.radar_resolution / 180 * M_PI;
  laser_max_range_ = m_MappingConfig.mapping_laser_max_range;
  laser_min_range_ = m_MappingConfig.mapping_laser_min_range;
  record_data_ = false;
  file_open_ = false;
  pthread_mutex_init(&mutex_pose_raw, nullptr);
  pthread_mutex_init(&mutex_odom_list_raw, nullptr);
  pthread_mutex_init(&mutex_ridar_list_raw, nullptr);
  pthread_mutex_init(&mutex_pose_list_raw, nullptr);
}

KartoMapping::~KartoMapping() {
  pthread_mutex_destroy(&mutex_pose_raw);
  pthread_mutex_destroy(&mutex_odom_list_raw);
  pthread_mutex_destroy(&mutex_ridar_list_raw);
  pthread_mutex_destroy(&mutex_pose_list_raw);
}

void KartoMapping::SetConfiguration(void *config) {}

void KartoMapping::HandleOdomData(const OdometerMessage &data) {
  AddOdomDataRaw(data.mclDeltaPosition);
  if (m_OdomListRaw.size() > FLUSH_ODOM_LIST_SIZE) {
    FlushOdomDataRaw();
  }
}

void KartoMapping::HandleLaserData(const RadarSensoryMessage &data) {
  if (m_MappingConfig.mapping_pattern == Offline) {
    m_CurrentRecordPose = ExpolateCurrentPositionTimeRaw(
        data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
    if (first_laser_) {
      first_laser_ = false;
      laser_resolution_ = fmod(laser_resolution_ + 5 * M_PI, 2 * M_PI) - M_PI;
      laser_resolution_ = fabs(laser_resolution_);
      angle_increment_ =
          data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament;
      angle_increment_ = fmod(angle_increment_ + 5 * M_PI, 2 * M_PI) - M_PI;
      angle_increment_ = fabs(angle_increment_);
      stepIncreament_ = std::round(laser_resolution_ / angle_increment_);
      float float_stepIncreament = laser_resolution_ / angle_increment_;

      if ((laser_resolution_ - angle_increment_) < 1e-4) {
        stepIncreament_ = 1;
        laser_resolution_ = angle_increment_;
        SLAM_ERROR("建图模块配置文件角度分辨率小于雷达数据分辨率\n");
      }
      if (fabs(float_stepIncreament - stepIncreament_) > 1e-4) {
        laser_resolution_ = angle_increment_ * stepIncreament_;
        SLAM_ERROR("建图模块配置文件角度分辨率必须是雷达数据分辨率整数倍\n");
      }
      if (laser_min_angle_ <
          data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) {
        laser_min_angle_ =
            data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin;
        SLAM_ERROR("建图模块配置文件雷达最小使用角度小于雷达数据最小角度\n");
      }
      if (laser_max_angle_ >
          data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax) {
        laser_max_angle_ =
            data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax;
        SLAM_ERROR("建图模块配置文件雷达最大使用角度大于雷达数据最大角度\n");
      }
      laser_min_angle_ = fmod(laser_min_angle_ + 5 * M_PI, 2 * M_PI) - M_PI;
      laser_max_angle_ = fmod(laser_max_angle_ + 5 * M_PI, 2 * M_PI) - M_PI;
      if (laser_min_range_ <
          data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMin) {
        laser_min_range_ =
            data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMin;
        SLAM_ERROR("建图模块配置文件雷达最小使用距离小于雷达数据最小距离\n");
      }
      if (laser_max_range_ >
          data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMax) {
        laser_max_range_ =
            data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMax;
        SLAM_ERROR("建图模块配置文件雷达最大使用距离大于雷达数据最大距离\n");
      }
      SLAM_DEBUG(
          "建图模块配置雷达角度分辨率为%f, 起始角度为%f, 终止角度为%f, "
          "最小距离为%f, 最大距离为%f\n",
          laser_resolution_, laser_min_angle_, laser_max_angle_,
          laser_min_range_, laser_max_range_);
    }
    if (record_data_) {
      RecordMapRawData(data);
    }
  } else if (m_MappingConfig.mapping_pattern == Online) {
  }
}

void KartoMapping::HandleImuData(const ImuSensoryMessage &data) {}

void KartoMapping::StartMapping() {
  finish_mapping_ = false;
  m_RecordIndex = 1;
  pthread_mutex_lock(&mutex_odom_list_raw);
  m_OdomListRaw.clear();
  pthread_mutex_unlock(&mutex_odom_list_raw);
  m_MappingOdom.mfX = 0;
  m_MappingOdom.mfY = 0;
  m_MappingOdom.mfTheta = 0;
  m_LastRecordPose = m_MappingOdom;
  m_CurrentRecordPose = m_MappingOdom;

  // Karto离线建图，保存的是采集到的激光雷达和里程计数据)
  //这段代码的作用是在指定路径下创建一个名为 "map.rawmap"
  //的空文件，并将其内容设置为一个空格。
  std::string str = " ";
  std::ofstream ofs(
      "./map/map.rawmap");  //创建一个输出文件流对象 ofs，并打开名为
                            //"./map/map.rawmap" 的文件。
  //如果文件不存在，则会创建新文件；如果文件已存在，则会清空文件内容。
  ofs << str;   //将空字符串 str 写入文件 ofs，即写入空格
  ofs.close();  //关闭文件流 ofs

  system("mkdir -p  map");  //命令用于创建一个名为 "map" 的目录。mkdir -p
                            //命令的作用是如果目录已存在，则不做任何操作；
  //如果目录不存在，则创建该目录。这里使用 -p
  //参数是为了确保在创建目录时能够创建所有需要的上级目录
  m_pMapRawDataFile = fopen("./map/map.rawmap", "w+");
  file_open_ = true;
  usleep(1500000);  // 等待文件打开完成
  m_FirstRecord = true;
  first_laser_ = true;
  record_data_ = true;
  record_finish_ = false;
  start_mapping_ = true;
  SLAM_DEBUG("创建存储数据线程\n");
  //并启动一个线程来记录数据。
  pthread_create(&record_data_thread_, NULL, RecordDataFunc, this);
}

bool KartoMapping::IsFinishMapping() { return finish_mapping_; }

void KartoMapping::StopMapping(std::string map_name) {
  record_data_ = false;
  //设置地图名称为给定的 map_name
  map_name_ = map_name;
  while (!record_finish_ && start_mapping_) {
    SLAM_INFO("等待数据存储结束\n");
    usleep(30000);
  }
  usleep(30000);
  if (file_open_) {
    fclose(m_pMapRawDataFile);
    file_open_ = false;
  }
  usleep(30000);
  pthread_create(&mapping_thread_, NULL, MappingFunc, this);
  // std::vector<char> map_data1;
  // // 基于karto算法离线建立地图
  // SlamKarto *my_karto = nullptr;
  // my_karto = new SlamKarto(m_MappingConfig);
  // my_karto->SetLaserParam(laser_min_angle_, laser_max_angle_,
  // laser_min_range_,
  //                         laser_max_range_, laser_resolution_);
  // my_karto->begin_slam(&map_data1, map_name);
  // usleep(30000);
  // if (!my_karto->map_successed) {
  //   SLAM_ERROR( "Karto 离线建图失败！\n");
  // }
  // delete (my_karto);
}

/**
 * @brief 录制一次数据
 *
 * @param data
 */
void KartoMapping::RecordMapRawData(const RadarSensoryMessage &data) {
  double xp, yp, laser_len;
  if (fabs(m_LastRecordPose.mfX - m_CurrentRecordPose.mfX) >
          m_MappingConfig.distance_step ||
      fabs(m_LastRecordPose.mfY - m_CurrentRecordPose.mfY) >
          m_MappingConfig.distance_step ||
      fabs(m_LastRecordPose.mfTheta - m_CurrentRecordPose.mfTheta) >
          m_MappingConfig.angular_step ||
      m_FirstRecord) {
    if (m_FirstRecord) {
      m_FirstRecord = false;
    }

    pthread_mutex_lock(&mutex_pose_list_raw);
    m_PoseList.push_back(m_CurrentRecordPose);
    pthread_mutex_unlock(&mutex_pose_list_raw);

    pthread_mutex_lock(&mutex_ridar_list_raw);
    m_RidarList.push_back(data);
    pthread_mutex_unlock(&mutex_ridar_list_raw);

    m_LastRecordPose = m_CurrentRecordPose;
    m_RecordIndex++;
  }
}

void *KartoMapping::RecordDataFunc(void *ptr) {
  pthread_detach(pthread_self());
  //将传入的指针转换为 KartoMapping 对象指针
  KartoMapping *p = reinterpret_cast<KartoMapping *>(ptr);
  int record_index = 0;
  // 在雷达数据列表非空或记录数据标志为真的情况下循环
  while (!p->m_RidarList.empty() || p->record_data_) {
    // 如果雷达数据列表非空
    if (!p->m_RidarList.empty()) {
      // 使用互斥锁保护雷达数据列表的操作
      pthread_mutex_lock(&p->mutex_ridar_list_raw);
      RadarSensoryMessage lidar_data = p->m_RidarList.front();
      p->m_RidarList.pop_front();
      pthread_mutex_unlock(&p->mutex_ridar_list_raw);

      // 使用互斥锁保护姿态列表的操作
      pthread_mutex_lock(&p->mutex_pose_list_raw);
      Position odom_data = p->m_PoseList.front();
      p->m_PoseList.pop_front();
      pthread_mutex_unlock(&p->mutex_pose_list_raw);

      fprintf(p->m_pMapRawDataFile, "%d\t%ld\t", record_index,
              odom_data.mlTimestamp);
      int start_i =
          fabs(p->laser_min_angle_ -
               lidar_data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
          p->angle_increment_;
      int record_cnt = fabs((p->laser_max_angle_ - p->laser_min_angle_) /
                            p->laser_resolution_);
      fprintf(p->m_pMapRawDataFile, "%f\t%f\t%f\t%d\t", odom_data.mfX,
              odom_data.mfY, odom_data.mfTheta, record_cnt);
      record_cnt *= p->stepIncreament_;
      // 遍历雷达数据点并写入文件
      for (int i = start_i; i <= (start_i + record_cnt);
           i += p->stepIncreament_) {
        float xp =
            lidar_data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](0);
        float yp =
            lidar_data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](1);
        float laser_len = sqrt(xp * xp + yp * yp);
        fprintf(p->m_pMapRawDataFile, "%f,", laser_len);
      }
      fprintf(p->m_pMapRawDataFile, "\n");
      record_index++;
    }
    usleep(100000);
  }
  // 标记记录结束
  p->record_finish_ = true;
}

void *KartoMapping::MappingFunc(void *ptr) {
  // 分离线程，使其在结束后自动释放资源
  pthread_detach(pthread_self());
  // 将 void 指针转换为 KartoMapping 类型指针
  KartoMapping *p = reinterpret_cast<KartoMapping *>(ptr);
  // 创建一个存储地图数据的 vector
  std::vector<char> map_data1;
  // 基于karto算法离线建立地图
  SlamKarto *my_karto = nullptr;
  my_karto = new SlamKarto(p->m_MappingConfig);
  // 设置激光参数
  my_karto->SetLaserParam(p->laser_min_angle_, p->laser_max_angle_,
                          p->laser_min_range_, p->laser_max_range_,
                          p->laser_resolution_);
  // 调用 begin_slam 函数开始离线建图，并将地图数据保存在 map_data1 中
  my_karto->begin_slam(&map_data1, p->map_name_);
  // 暂停一段时间，等待离线建图完成
  usleep(30000);
  // 如果离线建图失败，则输出错误日志
  if (!my_karto->map_successed) {
    SLAM_ERROR("Karto 离线建图失败！\n");
  }
  delete (my_karto);
  // 更新标志位，表示离线建图已完成
  p->first_laser_ = false;
  p->start_mapping_ = false;
  p->finish_mapping_ = true;
}

void KartoMapping::AddOdomDataRaw(const Position &pose_change) {
  uint64_t timestamp = pose_change.mlTimestamp;
  pthread_mutex_lock(&mutex_pose_raw);
  if (timestamp < m_MappingOdom.mlTimestamp) {
    // SLAM_WARN( "当前里程计数据时间戳小于最新里程计位姿时间戳\n");
  }
  pthread_mutex_lock(&mutex_odom_list_raw);
  if (m_OdomListRaw.empty()) {
    m_OdomListRaw.push_back(pose_change);
  } else {
    for (auto iter = m_OdomListRaw.begin();; iter++) {
      if (iter == m_OdomListRaw.end() || iter->mlTimestamp >= timestamp) {
        m_OdomListRaw.insert(iter, pose_change);
        break;
      }
    }
  }
  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
}

void KartoMapping::FlushOdomDataRaw() {
  pthread_mutex_lock(&mutex_pose_raw);
  Position pose = m_MappingOdom;
  pthread_mutex_lock(&mutex_odom_list_raw);
  while (!m_OdomListRaw.empty()) {
    pose = pose * m_OdomListRaw.front();
    m_OdomListRaw.pop_front();
  }
  m_MappingOdom = pose;

  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
}

/**
 * @brief 将建图里程计的位姿根据里程计数据预测到timestamp时刻
 *
 * @param timestamp
 * @return gomros::message::Position
 */
gomros::message::Position KartoMapping::ExpolateCurrentPositionTimeRaw(
    uint64_t timestamp) {
  pthread_mutex_lock(&mutex_pose_raw);
  Position ret = m_MappingOdom;
  pthread_mutex_lock(&mutex_odom_list_raw);
  for (Position &p : m_OdomListRaw) {
    if (p.mlTimestamp > ret.mlTimestamp && (p.mlTimestamp <= timestamp)) {
      ret = ret * p;
    }
  }
  m_MappingOdom = ret;
  while (!m_OdomListRaw.empty()) {
    if (m_OdomListRaw.front().mlTimestamp > timestamp) {
      break;
    }
    m_OdomListRaw.pop_front();
  }
  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
  return ret;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
