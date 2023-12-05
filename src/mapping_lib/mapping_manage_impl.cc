/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-17 14:58:10
 * @LastEditTime: 2023-03-23 16:30:13
 * @Author: lcfc-desktop
 */

#include "include/mapping_lib/mapping_manage_impl.h"

#include <string>
#include <vector>
#include "common_lib/message.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

MappingManagerImpl::MappingManagerImpl(
    const CartoMappingConfig &config,
    std::shared_ptr<DeviceState> device_state) {
  device_state_ = device_state;
  last_odom_data.mclDeltaPosition.mlTimestamp = 0;  //
  last_radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp = 0;
  last_imu_data.time_stamp =
      0;  //--------------------新加IMU数据的时间戳--------------------------
  is_stop_mapping_ = false;
  mp_MappingModule = new CartoMapping(config);  //父类的指针指向子类的对象
  pthread_mutex_init(&mutex_pose, nullptr);        //
  pthread_mutex_init(&mutex_radar_data, nullptr);  //
  pthread_mutex_init(&mutex_odom_data, nullptr);   //
  pthread_mutex_init(&mutex_imu_data,
                     nullptr);  //---------------新加-----------
  // pthread_create(&handle_odom_thread, NULL, HandleOdomData, this);
  // pthread_create(&handle_radar_thtead, NULL, HandleLaserData, this);
  SLAM_INFO("当前建图算法为Cartographer\n");
}

MappingManagerImpl::MappingManagerImpl(
    const KartoMappingConfig &config,
    std::shared_ptr<DeviceState> device_state) {
  device_state_ = device_state;
  last_odom_data.mclDeltaPosition.mlTimestamp = 0;
  last_radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp = 0;
  is_stop_mapping_ = false;
  mp_MappingModule = new KartoMapping(config);
  pthread_mutex_init(&mutex_pose, nullptr);        //
  pthread_mutex_init(&mutex_radar_data, nullptr);  //
  pthread_mutex_init(&mutex_odom_data, nullptr);   //
  // pthread_create(&handle_odom_thread, NULL, HandleOdomData, this);
  // pthread_create(&handle_radar_thtead, NULL, HandleLaserData, this);
}

MappingManagerImpl::~MappingManagerImpl() {
  if (mp_MappingModule != nullptr) delete mp_MappingModule;
  // pthread_join(handle_odom_thread, NULL);
  // pthread_join(handle_radar_thtead, NULL);
  pthread_mutex_destroy(&mutex_pose);
  pthread_mutex_destroy(&mutex_radar_data);
  pthread_mutex_destroy(&mutex_odom_data);
  pthread_mutex_destroy(
      &mutex_imu_data);  //----------------新加-------------------
}

void MappingManagerImpl::SetConfiguration(const CartoMappingConfig &config) {
  if (mp_MappingModule != nullptr) {
    mp_MappingModule->SetConfiguration((void *)(&config));
  }
}

void MappingManagerImpl::SetDeviceState(
    std::shared_ptr<gomros::message::DeviceState> device_state) {
  device_state_ = device_state;
}

void MappingManagerImpl::SetRadarData(const RadarSensoryMessage &data) {
  int no_data_cnt = 0;
  if (data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp >
      last_radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp) {
    last_radar_data = data;
    if (device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
      mp_MappingModule->HandleLaserData(data);
      if (mp_MappingModule->IsFinishMapping()) {
        device_state_->mcChassisState = SYSTEM_STATE_FREE;
        SLAM_INFO("结束建图!\n");
        is_stop_mapping_ = false;
      }
    }
  } else {
    if ((device_state_->mcChassisState == SYSTEM_STATE_MAPPING)) {
      no_data_cnt++;
      if (no_data_cnt >= 10 * PRINTF_TIME_DURATION) {
        SLAM_WARN("建图模块雷达数据无更新!\n");
        no_data_cnt = 0;
      }
    }
  }
}
void *MappingManagerImpl::HandleLaserData(void *ptr) {
  MappingManagerImpl *lp_this = reinterpret_cast<MappingManagerImpl *>(ptr);
  RadarSensoryMessage radar_data;
  radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp = 0;
  int no_data_cnt = 0;
  while (1) {
    if (lp_this->have_radar_data_)
      break;  //等待雷达数据的到来，have_radar_data_会变成true，就会跳出死循环
    usleep(50000);  //降低循环的频率，防止过度占用CPU内存
  }
  SLAM_INFO("建图模块雷达线程开始处理雷达数据!\n");
  while (1) {
    pthread_mutex_lock(&(lp_this->mutex_radar_data));  //先锁上

    if (radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp !=
        lp_this->last_radar_data.mstruRadarMessage.mstruRadarHeaderData
            .mlTimeStamp) {
      radar_data = lp_this->last_radar_data;
      pthread_mutex_unlock(&(lp_this->mutex_radar_data));  //再解锁

      if (lp_this->device_state_->mcChassisState != SYSTEM_STATE_MAPPING) {
      } else {
        lp_this->mp_MappingModule->HandleLaserData(radar_data);
      }
    } else {
      pthread_mutex_unlock(&(lp_this->mutex_radar_data));
      if ((lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING)) {
        no_data_cnt++;
        if (no_data_cnt >= 10 * PRINTF_TIME_DURATION) {
          SLAM_WARN("建图模块雷达数据无更新!\n");
          no_data_cnt = 0;
        }
      }
    }
    usleep(100000);
  }
}

void MappingManagerImpl::SetOdomData(const OdometerMessage &data) {
  int no_data_cnt = 0;
  if (data.mclDeltaPosition.mlTimestamp >
      last_odom_data.mclDeltaPosition.mlTimestamp) {
    last_odom_data = data;
    if (device_state_->mcChassisState != SYSTEM_STATE_MAPPING) {
    } else {
      // 建图里程计累计
      m_MappingOdom = m_MappingOdom * data.mclDeltaPosition;
      // 数据分发
      mp_MappingModule->HandleOdomData(data);
    }
  } else {
    // pthread_mutex_unlock(&(mutex_odom_data));
    if ((device_state_->mcChassisState == SYSTEM_STATE_MAPPING)) {
      no_data_cnt++;
      if (no_data_cnt >= 50 * PRINTF_TIME_DURATION) {
        SLAM_WARN("建图模块里程计数据无更新!\n");
        no_data_cnt = 0;
      }
    }
  }
  //请注意，在这段代码中没有使用互斥锁，因此假设该方法在单线程环境下被调用，
  //或者在多线程环境下被调用时已经进行了外部的同步控制
}
//这三个线程函数，在其他函数中，并没有被调用
void *MappingManagerImpl::HandleOdomData(void *ptr) {
  MappingManagerImpl *lp_this = reinterpret_cast<MappingManagerImpl *>(ptr);
  OdometerMessage odom_data;  //存储里程计数据的对象
  odom_data.mclDeltaPosition.mlTimestamp = 0;
  int no_data_cnt =
      0;  //初始化一个计数器no_data_cnt，用于记录没有数据更新的次数
  while (1) {
    if (lp_this->have_odom_data_) break;
    usleep(50000);  //暂停 50 毫秒，让出 CPU 时间给其他进程或线程。
  }
  SLAM_INFO("建图模块里程计线程开始处理里程计数据!\n");
  while (1) {
    pthread_mutex_lock(&(
        lp_this
            ->mutex_odom_data));  //加锁，以确保线程安全访问里程计数据，以避免多个线程同时访问和修改数据而导致的竞争条件

    if (odom_data.mclDeltaPosition.mlTimestamp !=
        lp_this->last_odom_data.mclDeltaPosition.mlTimestamp) {
      //检查当前的里程计数据时间戳是否与上次处理的里程计数据时间戳不同，如果是，则表示有新的数据
      odom_data =
          lp_this->last_odom_data;  //将上次处理的里程计数据赋值给odom_data对象

      pthread_mutex_unlock(&(lp_this->mutex_odom_data));

      if (lp_this->device_state_->mcChassisState != SYSTEM_STATE_MAPPING) {
      } else {
        // 将当前的里程计数据累积到m_MappingOdom变量中
        lp_this->m_MappingOdom =
            lp_this->m_MappingOdom * odom_data.mclDeltaPosition;
        // 数据分发
        lp_this->mp_MappingModule->HandleOdomData(odom_data);
      }
    } else {
      //如果里程计数据的时间戳与上次处理的时间戳相同，则表示没有新数据
      pthread_mutex_unlock(&(lp_this->mutex_odom_data));
      if ((lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING)) {
        no_data_cnt++;  //没有新数据，就增加没有数据更新的次数
        if (no_data_cnt >= 50 * PRINTF_TIME_DURATION) {
          SLAM_WARN("建图模块里程计数据无更新!\n");
          no_data_cnt = 0;
        }
      }
    }
    usleep(20000);
  }
}

void MappingManagerImpl::SetImuData(const ImuSensoryMessage &data) {
  int no_data_cnt = 0;
  //时间戳判断
  if (data.time_stamp > last_imu_data.time_stamp) {
    last_imu_data = data;
    if (device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
      // 数据分发
      mp_MappingModule->HandleImuData(data);
    }
  } else {
    // pthread_mutex_unlock(&(mutex_imu_data));
    if ((device_state_->mcChassisState == SYSTEM_STATE_MAPPING)) {
      no_data_cnt++;
      if (no_data_cnt >= 50 * PRINTF_TIME_DURATION) {
        SLAM_WARN("建图模块Imu数据无更新!\n");
        no_data_cnt = 0;
      }
    }
  }
}
//----------------------------------新加（待验证）------------------------
void *MappingManagerImpl::HandlemuData(void *ptr) {
  MappingManagerImpl *lp_this = reinterpret_cast<MappingManagerImpl *>(ptr);
  ImuSensoryMessage imu_data;  //声明
  imu_data.time_stamp = 0;
  int no_data_cnt = 0;
  while (1) {
    if (lp_this->have_imu_data_)
      break;  //等待imu数据的到来，have_imu_data_会变成true，就会跳出死循环
    usleep(50000);  //降低循环的频率，防止过度占用CPU内存
  }
  SLAM_INFO("建图模块Imu线程开始处理imu数据!\n");
  while (1) {
    pthread_mutex_lock(&(lp_this->mutex_imu_data));  //先锁上

    if (imu_data.time_stamp != lp_this->last_imu_data.time_stamp) {
      imu_data = lp_this->last_imu_data;
      pthread_mutex_unlock(&(lp_this->mutex_imu_data));  //再解锁

      if (lp_this->device_state_->mcChassisState != SYSTEM_STATE_MAPPING) {
      } else {
        lp_this->mp_MappingModule->HandleImuData(imu_data);
      }
    } else {
      pthread_mutex_unlock(&(lp_this->mutex_imu_data));
      if ((lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING)) {
        no_data_cnt++;
        if (no_data_cnt >= 10 * PRINTF_TIME_DURATION) {
          SLAM_WARN("建图模块Imu数据无更新!\n");
          no_data_cnt = 0;
        }  //-------------------这一部分为什么要写两遍？？因为跟上面的AddImuData实现的功能实现的功能一样，这种是通过线程函数的方式------------------
      }
    }
    usleep(100000);
  }
}

bool MappingManagerImpl::StartMapping() {
  //加打印信息
  if (device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    SLAM_ERROR("建图指令错误，当前正在开始建图中......\n");
    return false;
  } else if (device_state_->mcChassisState == SYSTEM_STATE_LOCATING ||
             device_state_->mcChassisState == SYSTEM_STATE_INIT_POSE) {
    SLAM_ERROR("建图指令错误，当前正在定位中，不能建图......\n");
    return false;
  }

  // 重置建图里程计
  device_state_->mcChassisState = SYSTEM_STATE_MAPPING;
  m_MappingOdom.mfX = 0;
  m_MappingOdom.mfY = 0;
  m_MappingOdom.mfTheta = 0;
  // 构建建图对象
  mp_MappingModule->StartMapping();
  //加打印信息
  return true;
}

void MappingManagerImpl::StopMapping(std::string map_name) {
  if (is_stop_mapping_) {
    SLAM_ERROR("当前正在结束建图中......\n");
  } else {
    is_stop_mapping_ = true;
    usleep(20000);
    mp_MappingModule->StopMapping(map_name);
  }
}

gomros::message::Position MappingManagerImpl::GetPosition() {
  pthread_mutex_lock(&mutex_pose);
  Position pose = m_MappingOdom;
  pthread_mutex_unlock(&mutex_pose);
  return pose;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
