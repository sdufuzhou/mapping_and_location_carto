#include "include/mapping_lib/carto_mapping.h"
#include <fstream>
#include <string>
#include <vector>
#include "common_lib/time_utils.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
//------传入开发人员配置的参数-------
CartoMapping::CartoMapping(const CartoMappingConfig &config) {
  // 在构造函数中，给CartoMapping类成员变量config_赋初值
  config_ = config;
  // 传感器选择
  use_odom_ = config_.use_odom_data;
  use_imu_ = config_.use_imu_data;

  // 建图模式选择
  offline_mapping_ = config_.mapping_pattern;

  // 转换为弧度
  laser_min_angle_ = config_.mapping_start_angle / 180.0 * M_PI;
  laser_max_angle_ = config_.mapping_end_angle / 180.0 * M_PI;
  laser_resolution_ = config_.laser_resolution / 180.0 * M_PI;

  //  这两个用的是carto里自带的参数
  laser_max_range_ = config_.max_range;
  laser_min_range_ = config_.min_range;
}
CartoMapping::~CartoMapping() {}

void CartoMapping::SetConfiguration(void *config) {
  config_ = *(reinterpret_cast<CartoMappingConfig *>(config));
  laser_min_angle_ = config_.mapping_start_angle / 180 * M_PI;  // 转换为弧度
  laser_max_angle_ = config_.mapping_end_angle / 180 * M_PI;
  laser_resolution_ = config_.laser_resolution / 180 * M_PI;
  // 这两个用的是carto里自带的参数
  laser_max_range_ = config_.max_range;
  laser_min_range_ = config_.min_range;
}
void CartoMapping::StartMapping() {
  // 共同的
  finish_mapping_ = false;
  add_data_ = true;
  first_laser_ = true;
  // 首先创建一个map_builder_对象
  // 在线直接在这创建,因为线程函数中在线的话是直接添加传感器数据，必须有map_builder_对象
  if (!offline_mapping_) {
    CreatCartoModule();
  }
  // SLAM_DEBUG("创建三个处理数据(包含在线或者离线)线程\n"),以前版本
  // 调用
  std::thread th(std::bind(&CartoMapping::HandleTimeQueue, this));
  th.detach();
}
void CartoMapping::StopMapping(std::string map_name) {
  // 更新标志位

  add_data_ = false;  // 此时也不再往list容器中添加数据了
  // 设置地图名称为给定的 map_name
  map_name_ = map_name;
  // 离线和在线都要用到的线程
  std::thread th(std::bind(&CartoMapping::FinishMapping, this));
  th.detach();
}
void CartoMapping::FinishMapping() {
  // SLAM_DEBUG("等待结束建图\n");
  std::unique_lock<std::mutex> lck(stop_mapping_mtx_);
  stop_mapping_cond_.wait(lck, [this] { return handle_radar_finish_; });
  lck.unlock();
  SLAM_DEBUG("开始结束建图\n");
  // 同时整个while循环，又阻塞程序的向下运行，直到各种传感器数据都处理结束才会往下运行
  if (offline_mapping_) {
    CreatCartoModule();
    SLAM_DEBUG("创建三个读数据线程\n");
    SLAM_INFO("#######等待从文件中读取数据完毕！！！！\n");
    std::thread th(std::bind(&CartoMapping::ReadRadarData, this));
    th.join();
    if (use_odom_) {
      std::thread th(std::bind(&CartoMapping::ReadOdomData, this));
      th.join();
    }
    if (use_imu_) {
      std::thread th(std::bind(&CartoMapping::ReadImuData, this));
      th.join();
    }
    usleep(1000000);
    SLAM_DEBUG("数据从文件读取完毕\n");
    SLAM_DEBUG("odom zise %d,radar size %d\n", odom_list_from_file_.size(),
               lidar_list_from_file_.size());

    while (!odom_list_from_file_.empty() || !lidar_list_from_file_.empty()) {
      if (!odom_list_from_file_.empty() && !lidar_list_from_file_.empty()) {
        if (odom_list_from_file_.front().mlTimestamp <=
            lidar_list_from_file_.front()
                .mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp) {
          SLAM_WARN("添加一帧里程计数据\n");
          // std::thread
          // th(std::bind(&CartoMapping::OdomDataConversionAndAddFunc,
          //                          this, odom_list_from_file_.front()));
          // th.join();
          OdomDataConversionAndAddFunc(odom_list_from_file_.front());
          odom_list_from_file_.pop_front();
          LOG_WARN << "剩余" << odom_list_from_file_.size() << "帧里程计数据";
        } else {
          SLAM_WARN("添加一帧雷达数据\n");
          // std::thread
          // th(std::bind(&CartoMapping::LaserDataConversionAndAddFunc,
          //                          this, lidar_list_from_file_.front()));
          // th.join();
          LaserDataConversionAndAddFunc(lidar_list_from_file_.front());
          lidar_list_from_file_.pop_front();
          LOG_WARN << "剩余" << lidar_list_from_file_.size() << "帧雷达数据";
        }
      } else if (!odom_list_from_file_.empty()) {
        SLAM_WARN("添加一帧里程计数据\n");
        // std::thread th(std::bind(&CartoMapping::OdomDataConversionAndAddFunc,
        //                          this, odom_list_from_file_.front()));
        // th.join();
        OdomDataConversionAndAddFunc(odom_list_from_file_.front());
        odom_list_from_file_.pop_front();
        LOG_WARN << "剩余" << odom_list_from_file_.size() << "帧里程计数据";
      } else if (!lidar_list_from_file_.empty()) {
        SLAM_WARN("添加一帧雷达数据\n");
        // std::thread
        // th(std::bind(&CartoMapping::LaserDataConversionAndAddFunc,
        //                          this, lidar_list_from_file_.front()));
        // th.join();
        LaserDataConversionAndAddFunc(lidar_list_from_file_.front());
        lidar_list_from_file_.pop_front();
        LOG_WARN << "剩余" << lidar_list_from_file_.size() << "帧雷达数据";
      }
      // usleep(100000);
    }
  }
  sleep(2);  // 不影响在线
  StopAndOptimize();
  usleep(3000000);
  PaintMap();
  finish_mapping_ = true;
  lidar_list_from_file_.clear();
  odom_list_from_file_.clear();
  SLAM_DEBUG("结束FinishMapping线程函数\n");
}

bool CartoMapping::IsFinishMapping() { return finish_mapping_; }

void CartoMapping::HandleLaserData(const RadarSensoryMessage &data) {
  if (first_laser_) {
    first_laser_ = false;
    laser_resolution_ = fmod(laser_resolution_ + 5 * M_PI, 2 * M_PI) - M_PI;
    laser_resolution_ = fabs(laser_resolution_);
    angle_increment_ =
        data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament;
    angle_increment_ = fmod(angle_increment_ + 5 * M_PI, 2 * M_PI) - M_PI;
    angle_increment_ = fabs(angle_increment_);
    //---------激光分辨率（我们自己配置的）与雷达分辨率的比值,比值为1，代表需要雷达的所有数据点------
    stepIncreament_ = std::round(laser_resolution_ / angle_increment_);
    float float_stepIncreament = laser_resolution_ / angle_increment_;
    SLAM_DEBUG("stepIncreament_  :%d\n", stepIncreament_);

    if ((laser_resolution_ - angle_increment_) < -1e-4) {
      stepIncreament_ = 1;
      laser_resolution_ = angle_increment_;
      SLAM_ERROR("建图模块配置文件角度分辨率小于雷达数据分辨率\n");
    }
    if (fabs(float_stepIncreament - stepIncreament_) > 1e-4) {
      // laser_resolution_ = angle_increment_ * stepIncreament_;
      stepIncreament_ = 1;
      laser_resolution_ = angle_increment_;
      SLAM_WARN(
          "建图模块配置文件角度分辨率必须是雷达数据分辨率整数倍,实际为%f, "
          "取整之后为%d, 使用雷达原始分辨率!!!!!!!!!!!!!!\n",
          float_stepIncreament, stepIncreament_);
    }
    SLAM_DEBUG("mstruRadarHeaderData.mfAngleMin max :%f %f",
               data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin,
               data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax);

    if (laser_min_angle_ <
        data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) {
      laser_min_angle_ = data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin;
      SLAM_ERROR("建图模块配置文件雷达最小使用角度小于雷达数据最小角度\n");
    }
    if (laser_max_angle_ >
        data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax) {
      laser_max_angle_ = data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax;
      SLAM_ERROR("建图模块配置文件雷达最大使用角度大于雷达数据最大角度\n");
    }
    laser_min_angle_ = fmod(laser_min_angle_ + 5 * M_PI, 2 * M_PI) - M_PI;
    laser_max_angle_ = fmod(laser_max_angle_ + 5 * M_PI, 2 * M_PI) - M_PI;
    if (laser_min_range_ <
        data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMin) {
      laser_min_range_ = data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMin;
      SLAM_ERROR("建图模块配置文件雷达最小使用距离小于雷达数据最小距离\n");
    }
    if (laser_max_range_ >
        data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMax) {
      laser_max_range_ = data.mstruRadarMessage.mstruRadarHeaderData.mfRangeMax;
      SLAM_ERROR("建图模块配置文件雷达最大使用距离大于雷达数据最大距离\n");
    }
    SLAM_DEBUG(
        "建图模块配置雷达角度分辨率为%f, 起始角度为%f, 终止角度为%f, "
        "最小距离为%f, 最大距离为%f\n",
        laser_resolution_, laser_min_angle_, laser_max_angle_, laser_min_range_,
        laser_max_range_);
  }
  if (add_data_) {
    std::unique_lock<std::mutex> lidar_locker(raw_lidar_list_mtx_);
    lidar_list_.push_back(data);
    lidar_locker.unlock();
    time_queue_.insert(
        std::make_pair(data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp,
                       SensorType::Lidar));
  }
  if (offline_mapping_) {
  }
}
void CartoMapping::HandleOdomData(const OdometerMessage &data) {
  if (add_data_ && use_odom_) {
    std::unique_lock<std::mutex> lock(raw_odom_list_mtx_);
    time_queue_.insert(
        std::make_pair(data.mclDeltaPosition.mlTimestamp, SensorType::Odom));
    odom_list_.push_back(data);
    lock.unlock();
  }
}
void CartoMapping::HandleImuData(const ImuSensoryMessage &data) {
  if (add_data_ && use_imu_) {
    std::unique_lock<std::mutex> locker(raw_lidar_list_mtx_);
    imu_lists_.push_back(data);
    locker.unlock();
  }
}

void CartoMapping::HandleTimeQueue() {
  // 创建激光数据离线存储文件
  system("mkdir -p  data");  // 如果目录不存在，则创建该目录
  // 创建一个记录雷达数据的空文件，往文件里写入雷达数据（针对于离线建图）
  std::string str = " ";
  std::ofstream ofs_laser("./data/laser_data.rawmap");  //./代表当前工作目录下
  // 如果文件不存在，则会创建新文件；如果文件已存在，则会清空文件内容。
  ofs_laser << str;
  ofs_laser.close();
  // 返回该文件的文件指针，后面用该指针对文件进行读写操作
  RadarDataFile_ = fopen("./data/laser_data.rawmap", "w+");
  usleep(1000);
  radar_file_open_ = true;
  std::unique_lock<std::mutex> locker(stop_mapping_mtx_);
  handle_radar_finish_ = false;
  locker.unlock();

  // 创建odom离线存储数据文件
  std::ofstream ofs_odom("./data/odom_data.rawmap");
  ofs_odom << str;
  ofs_odom.close();
  OdomDataFile_ = fopen("./data/odom_data.rawmap", "w+");
  usleep(1500000);
  Position Wheel_Odom;
  odom_file_open_ = true;
  handle_odom_finish_ = false;

  // 创建IMU离线存储数据文件
  std::ofstream ofs_imu("./data/imu_data.rawmap");
  ofs_imu << str;
  ofs_imu.close();
  ImuDataFile_ = fopen("./data/imu_data.rawmap", "w+");
  usleep(1500000);
  imu_file_open_ = true;
  handle_imu_finish_ = false;
  record_index_radar_ = 0;
  record_index_odom_ = 0;
  record_index_imu_ = 0;

  // 上锁
  std::unique_lock<std::mutex> time_locker(time_queue_mtx_);
  while (!time_queue_.empty() || add_data_) {
    auto time_item = *time_queue_.begin();
    time_queue_.erase(time_queue_.begin());
    time_locker.unlock();
    if (time_item.second == SensorType::Lidar) {
      std::unique_lock<std::mutex> lidar_locker(raw_lidar_list_mtx_);
      RadarSensoryMessage laser_data = lidar_list_.front();
      lidar_list_.pop_front();
      lidar_locker.unlock();
      if (!offline_mapping_) {
        LaserDataConversionAndAddFunc(laser_data);
      } else {
        record_index_radar_++;
        SaveLaserDataToFile(laser_data);
      }
    } else if (time_item.second == SensorType::Odom) {
      std::unique_lock<std::mutex> odom_locker(raw_odom_list_mtx_);
      OdometerMessage odom_data = odom_list_.front();
      odom_list_.pop_front();
      odom_locker.unlock();
      // 构建轮式里程计在tracking_frame下的位姿所对应的变换矩阵
      Eigen::Matrix3d transformation_matrix_wheel;
      float x = config_.wheel_odom_position_x;
      float y = config_.wheel_odom_position_y;
      float theta = config_.wheel_odom_position_theta;
      transformation_matrix_wheel << std::cos(theta), -std::sin(theta), x,
          std::sin(theta), std::cos(theta), y, 0, 0, 1;
      // 计算此时 wheel_odom在local下的位姿
      Wheel_Odom = Wheel_Odom * odom_data.mclDeltaPosition;
      // 位姿的表示形式（x,y,theta），可以直接乘以增量，*运算符事先已经重载过的
      float x_ = Wheel_Odom.mfX;
      float y_ = Wheel_Odom.mfY;
      float theta_ = Wheel_Odom.mfTheta;
      // 构建wheel_odom在local下的位姿对应的变换矩阵
      Eigen::Matrix3d transformation_matrix;
      transformation_matrix << std::cos(theta_), -std::sin(theta_), x_,
          std::sin(theta_), std::cos(theta_), y_, 0, 0, 1;
      // 求解这帧里程计数据的时间戳下所对应的tracking_frame在local坐标系下的位姿
      Eigen::Matrix3d local_to_tracking =
          transformation_matrix * transformation_matrix_wheel.inverse();
      // pose不写入转换函数中，便于离线写入文件
      Position pose(odom_data.mclDeltaPosition.mlTimestamp,
                    local_to_tracking(0, 2), local_to_tracking(1, 2),
                    atan2(local_to_tracking(1, 0), local_to_tracking(0, 0)));
      if (!offline_mapping_) {
        // std::thread th(
        //     std::bind(&CartoMapping::OdomDataConversionAndAddFunc, this,
        //     pose));
        // th.join();
        OdomDataConversionAndAddFunc(pose);  // 直接调和注释的功能相同
      } else {
        record_index_odom_++;
        SaveOdomDataToFile(pose);
      }
    } else {
      // imu枷锁
      std::unique_lock<std::mutex> imu_locker(raw_imu_list_mtx_);
      ImuSensoryMessage imu_data = imu_lists_.front();
      imu_lists_.pop_front();
      // imu解锁
      imu_locker.unlock();
      if (!offline_mapping_) {
        // std::thread th(
        //     std::bind(&CartoMapping::ImuDataConversionAndAddFunc,this,
        //     imu_data));
        // th.join();
        ImuDataConversionAndAddFunc(imu_data);
      } else {
        record_index_imu_++;
        SaveImuDataToFile(imu_data);
      }
    }
    time_locker.lock();
  }
  // 标志位更新
  //  laser
  fclose(RadarDataFile_);
  radar_file_open_ = false;
  locker.lock();
  handle_radar_finish_ = true;
  stop_mapping_cond_.notify_all();
  locker.unlock();
  // odom
  fclose(OdomDataFile_);
  odom_file_open_ = false;
  handle_odom_finish_ = true;
  // imu
  fclose(ImuDataFile_);
  imu_file_open_ = false;
  handle_imu_finish_ = true;
}

void CartoMapping::SaveLaserDataToFile(RadarSensoryMessage &Laser_data) {
  // 写入index、时间戳、
  int start_i =
      fabs(laser_min_angle_ -
           Laser_data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
      angle_increment_;
  double ft =
      start_i *
      Laser_data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament *
      1000000;
  uint64_t lut = (uint64_t)ft;
  // 计算实际使用的一帧激光数据的第一个点的时间戳
  uint64_t real_time =
      Laser_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp + lut;
  int record_cnt =
      fabs((laser_max_angle_ - laser_min_angle_) / laser_resolution_) + 1;
  fprintf(RadarDataFile_, "%d %d %ld %f %f %f %f", record_index_radar_,
          record_cnt, real_time,
          Laser_data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament *
              stepIncreament_,
          laser_min_angle_, laser_max_angle_, laser_resolution_);
  int end_i = start_i + (record_cnt - 1) * stepIncreament_;
  for (int i = start_i; i <= end_i; i += stepIncreament_) {
    double xp =
        Laser_data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](0);
    double yp =
        Laser_data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](1);
    double laser_len = sqrt(xp * xp + yp * yp);
    fprintf(RadarDataFile_, " %f", laser_len);
  }
  fprintf(RadarDataFile_, "\n");  // 换行
}
void CartoMapping::SaveOdomDataToFile(Position &pose) {
  // 往文件中写入的数据都是经过处理后的数据，比如坐标变换后的
  //  写入index、时间戳
  fprintf(OdomDataFile_, "%d %ld ", record_index_odom_, pose.mlTimestamp);
  //------写入x，y，theta------
  fprintf(OdomDataFile_, "%f %f %f", pose.mfX, pose.mfY, pose.mfTheta);
  fprintf(OdomDataFile_, "\n");  // 换行
}
void CartoMapping::SaveImuDataToFile(ImuSensoryMessage &imu_data) {
  // 写入index、时间戳
  fprintf(ImuDataFile_, "%d %ld ", record_index_imu_, imu_data.time_stamp);
  //%ld这里不用改，虽然时间戳的类型并不是ld,但是因为最终getline拿到的都是字符串,一串数字形成的字符串
  //------写入绕z轴的旋转角速度与向前加速度------
  fprintf(ImuDataFile_, "%f %f ", imu_data.z_omega,
          imu_data.forward_linear_accel);
  // 写入z向角度
  fprintf(ImuDataFile_, "%f", imu_data.z_angle);
  fprintf(ImuDataFile_, "\n");  // 换行
}

void *CartoMapping::HandleLaser(void *ptr) {
  pthread_detach(pthread_self());
  CartoMapping *p = reinterpret_cast<CartoMapping *>(ptr);
  // 创建一个记录雷达数据的空文件，往文件里写入雷达数据（针对于离线建图）
  std::string str = " ";
  std::ofstream ofs("./data/laser_data.rawmap");  //./代表当前工作目录下
  // 如果文件不存在，则会创建新文件；如果文件已存在，则会清空文件内容。
  ofs << str;   // 将空字符串 str 写入文件 ofs，即写入空格
  ofs.close();  // 关闭文件流 ofs
  system("mkdir -p  data");
  // 如果目录不存在，则创建该目录。这里使用 -p
  // 参数是为了确保在创建目录时能够创建所有需要的上级目录
  // 返回该文件的文件指针，后面用该指针对文件进行读写操作
  p->RadarDataFile_ = fopen("./data/laser_data.rawmap", "w+");
  usleep(1000);
  // laser
  p->radar_file_open_ = true;
  p->handle_radar_finish_ = false;
  int record_index = 0;
  //----------循环会持续进行，直到激光雷达数据列表为空且add_data_标志为假------
  while (!p->lidar_list_.empty() || p->add_data_) {
    // 判断雷达数据列表非空才进行处理数据,防止如果取数据比放数据快时,程序报错
    if (!p->lidar_list_.empty()) {  // 保证有数据才可以拿
      // 使用互斥锁保护雷达数据列表的操作
      std::unique_lock<std::mutex> locker(p->raw_lidar_list_mtx_);
      RadarSensoryMessage Laser_data = p->lidar_list_.front();
      // 取数据，取数据的速度可能比往list中放快
      p->lidar_list_.pop_front();  // 是为了跳出循环
      locker.unlock();
      // 在线
      if (!p->offline_mapping_) {
        // p->LaserDataConversionAndAddFunc(Laser_data);
        std::thread th(std::bind(&CartoMapping::LaserDataConversionAndAddFunc,
                                 p, Laser_data));
        th.detach();
      }
      // 离线就是把从容器中取出的Laser_data写入文件中（未进行数据格式转换的）
      else {
        // 写入index、时间戳、
        int start_i =
            fabs(p->laser_min_angle_ -
                 Laser_data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
            p->angle_increment_;
        double ft =
            start_i *
            Laser_data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament *
            1000000;
        uint64_t lut = (uint64_t)ft;
        // 计算实际使用的激光数据的时间戳
        uint64_t real_time =
            Laser_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp + lut;
        int record_cnt = fabs((p->laser_max_angle_ - p->laser_min_angle_) /
                              p->laser_resolution_) +
                         1;
        fprintf(
            p->RadarDataFile_, "%d %d %ld %f %f %f %f", record_index,
            record_cnt, real_time,
            Laser_data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament *
                p->stepIncreament_,
            p->laser_min_angle_, p->laser_max_angle_, p->laser_resolution_);
        int end_i = start_i + (record_cnt - 1) * p->stepIncreament_;
        for (int i = start_i; i <= end_i; i += p->stepIncreament_) {
          double xp =
              Laser_data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](0);
          double yp =
              Laser_data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](1);
          double laser_len = sqrt(xp * xp + yp * yp);
          fprintf(p->RadarDataFile_, " %f", laser_len);
        }
        fprintf(p->RadarDataFile_, "\n");  // 换行
        record_index++;
      }
    }
    usleep(1000);
  }
  fclose(p->RadarDataFile_);
  p->radar_file_open_ = false;
  p->handle_radar_finish_ = true;
}
void CartoMapping::LaserDataConversionAndAddFunc(RadarSensoryMessage &data) {
  if (trajectory_id_ < 0) {
    return;
  }
  if (last_data_time_ == 0 ||
      last_data_time_ <
          data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp) {
    // SLAM_DEBUG ("开始添加一次雷达数据\n");
    uint64_t t1 = gomros::common::GetCurrentTime_us();
    trajectory_builder_->AddSensorData("range0", ToCartoPointCloud(data));
    // SLAM_DEBUG ("添加一次雷达数据成功\n");
    uint64_t t2 = gomros::common::GetCurrentTime_us();
    SLAM_INFO("添加一帧雷达数据耗时：%lu\n", t2 - t1);
    last_data_time_ = data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
  } else {
    SLAM_DEBUG("上一帧时间和当前时间:%lu     %lu\n", last_data_time_,
               data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
    SLAM_WARN("laser timeout.....\n");
  }
}
void *CartoMapping::HandleOdom(void *ptr) {
  pthread_detach(pthread_self());
  CartoMapping *p = reinterpret_cast<CartoMapping *>(ptr);
  // 创建一个记录里程计数据的空文件，往文件里写入里程计数据（针对于离线建图）
  std::string str = " ";
  std::ofstream ofs("./data/odom_data.rawmap");
  // 如果文件不存在，则会创建新文件；如果文件已存在，则会清空文件内容。
  ofs << str;
  ofs.close();
  system("mkdir -p  data");
  // 如果目录不存在，则创建该目录。这里使用 -p
  // 参数是为了确保在创建目录时能够创建所有需要的上级目录
  p->OdomDataFile_ = fopen("./data/odom_data.rawmap", "w+");
  // 返回该文件的文件指针，后面用该指针对文件进行读写操作
  usleep(1500000);  // 等待文件打开完成
  // odom
  p->odom_file_open_ = true;
  p->handle_odom_finish_ = false;
  int record_index = 0;
  Position Wheel_Odom;
  while (!p->odom_list_.empty() || p->add_data_) {
    if (!p->odom_list_.empty()) {
      // 使用互斥锁保护里程计数据列表的操作
      OdometerMessage odom_data = p->odom_list_.front();
      p->odom_list_.pop_front();

      // 构建轮式里程计在tracking_frame下的位姿所对应的变换矩阵
      Eigen::Matrix3d transformation_matrix_wheel;
      float x = p->config_.wheel_odom_position_x;
      float y = p->config_.wheel_odom_position_y;
      float theta = p->config_.wheel_odom_position_theta;
      transformation_matrix_wheel << std::cos(theta), -std::sin(theta), x,
          std::sin(theta), std::cos(theta), y, 0, 0, 1;
      // 计算此时 wheel_odom在local下的位姿
      Wheel_Odom = Wheel_Odom * odom_data.mclDeltaPosition;
      // 位姿的表示形式（x,y,theta），可以直接乘以增量，*运算符事先已经重载过的
      float x_ = Wheel_Odom.mfX;
      float y_ = Wheel_Odom.mfY;
      float theta_ = Wheel_Odom.mfTheta;
      // 构建wheel_odom在local下的位姿对应的变换矩阵
      Eigen::Matrix3d transformation_matrix;
      transformation_matrix << std::cos(theta_), -std::sin(theta_), x_,
          std::sin(theta_), std::cos(theta_), y_, 0, 0, 1;
      // 求解这帧里程计数据的时间戳下所对应的tracking_frame在local坐标系下的位姿
      Eigen::Matrix3d local_to_tracking =
          transformation_matrix * transformation_matrix_wheel.inverse();
      // pose不写入转换函数中，便于离线写入文件
      Position pose(odom_data.mclDeltaPosition.mlTimestamp,
                    local_to_tracking(0, 2), local_to_tracking(1, 2),
                    atan2(local_to_tracking(1, 0), local_to_tracking(0, 0)));
      // 在线
      if (!p->offline_mapping_ && p->use_odom_) {
        // 封装成一个函数
        // p->OdomDataConversionAndAddFunc(pose);
        std::thread th(
            std::bind(&CartoMapping::OdomDataConversionAndAddFunc, p, pose));
        th.detach();
      }
      // 离线
      else {
        // 往文件中写入的数据都是经过处理后的数据，比如坐标变换后的
        //  写入index、时间戳
        fprintf(p->OdomDataFile_, "%d %ld ", record_index,
                odom_data.mclDeltaPosition.mlTimestamp);
        //------写入x，y，theta------
        fprintf(p->OdomDataFile_, "%f %f %f", pose.mfX, pose.mfY, pose.mfTheta);
        fprintf(p->OdomDataFile_, "\n");  // 换行
        record_index++;
      }
    }
    usleep(1000);
  }
  fclose(p->OdomDataFile_);
  p->odom_file_open_ = false;
  p->handle_odom_finish_ = true;
}
void CartoMapping::OdomDataConversionAndAddFunc(Position &pose) {
  if (trajectory_id_ < 0) {
    return;
  }
  if (last_data_time_ == 0 || last_data_time_ < pose.mlTimestamp) {
    cartographer::sensor::OdometryData tmp{ToCartoTime(pose.mlTimestamp),
                                           ToCartoRigid3d(pose)};
    uint64_t t1 = gomros::common::GetCurrentTime_us();
    trajectory_builder_->AddSensorData("odom0", tmp);
    uint64_t t2 = gomros::common::GetCurrentTime_us();
    last_data_time_ = pose.mlTimestamp;
    SLAM_WARN("添加一帧里程计数据耗时%lu\n", t2 - t1);
  } else {
    SLAM_DEBUG("上一帧时间和当前时间:%lu %lu\n", last_data_time_,
               pose.mlTimestamp);
    SLAM_WARN("odom timeout.....\n");
  }
}

void *CartoMapping::HandleImu(void *ptr) {
  pthread_detach(pthread_self());
  CartoMapping *p = reinterpret_cast<CartoMapping *>(ptr);
  // 创建一个记录Imu数据的空文件，往文件里写入Imu数据（针对于离线建图）
  std::string str = " ";
  std::ofstream ofs("./data/imu_data.rawmap");  //./代表当前工作目录下
  // 如果文件不存在，则会创建新文件；如果文件已存在，则会清空文件内容。
  ofs << str;   // 将空字符串 str 写入文件 ofs，即写入空格
  ofs.close();  // 关闭文件流 ofs
  system("mkdir -p  data");
  // 如果目录不存在，则创建该目录。这里使用 -p
  // 参数是为了确保在创建目录时能够创建所有需要的上级目录
  // 返回该文件的文件指针，后面用该指针对文件进行读写操作
  p->ImuDataFile_ = fopen("./data/imu_data.rawmap", "w+");
  usleep(1500000);  // 等待文件打开完成
  // imu
  p->imu_file_open_ = true;
  p->handle_imu_finish_ = false;
  int record_index = 0;
  while (!p->imu_lists_.empty() || p->add_data_) {
    if (!p->imu_lists_.empty()) {
      ImuSensoryMessage imu_data = p->imu_lists_.front();
      p->imu_lists_.pop_front();
      // 在线
      if (!p->offline_mapping_ && p->use_imu_) {
        // 封装成函数
        p->ImuDataConversionAndAddFunc(imu_data);
      }
      // 离线
      else {
        // 写入index、时间戳
        fprintf(p->ImuDataFile_, "%d %ld ", record_index, imu_data.time_stamp);
        //%ld这里不用改，虽然时间戳的类型并不是ld,但是因为最终getline拿到的都是字符串,一串数字形成的字符串
        //------写入绕z轴的旋转角速度与向前加速度------
        fprintf(p->ImuDataFile_, "%f %f ", imu_data.z_omega,
                imu_data.forward_linear_accel);
        // 写入z向角度
        fprintf(p->ImuDataFile_, "%f", imu_data.z_angle);
        fprintf(p->ImuDataFile_, "\n");  // 换行
        record_index++;
      }
    }
    usleep(1000);
  }
  fclose(p->ImuDataFile_);
  p->imu_file_open_ = false;
  p->handle_imu_finish_ = true;
}
void CartoMapping::ImuDataConversionAndAddFunc(ImuSensoryMessage &data) {
  if (trajectory_id_ < 0) {
    return;
  }
  if (latest_imu_timestamp_ < 0 || latest_imu_timestamp_ < data.time_stamp) {
    trajectory_builder_->AddSensorData("imu0", ToCartoImu(data));
    latest_imu_timestamp_ = data.time_stamp;
  } else {
    SLAM_WARN("imu timeout.....\n");
  }
}

void CartoMapping::StopAndOptimize() {
  std::unique_lock<std::mutex> locker(paint_mutex_);
  SLAM_DEBUG("进入StopAndOptimize函数\n");
  if (trajectory_id_ < 0) {
    return;
  }
  // 结束trajectory
  map_builder_->FinishTrajectory(trajectory_id_);
  SLAM_DEBUG("结束FinishTrajectory\n");
  trajectory_id_ = -1;
  usleep(1000000);
  // 运行最终的全局优化
  map_builder_->pose_graph()->RunFinalOptimization();
  SLAM_DEBUG("RunFinalOptimization\n");
  locker.unlock();
}

void CartoMapping::PaintMap() {
  // std::unique_lock<std::mutex> locker;
  SLAM_DEBUG("进入PaintMap函数\n");
  using cartographer::io::SubmapSlice;
  using cartographer::mapping::SubmapId;
  gomros::message::MapInfo info;

  std::unique_lock<std::mutex> locker(paint_mutex_);

  double resolution = 0.05;
  // 获取所有子图的位姿
  std::map<SubmapId, SubmapSlice> submap_slices;
  auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  // for 循环遍历 submap_poses 容器中的每一个元素（每一个元素又是一个map容器）
  for (const auto &submap_id_pose : submap_poses) {
    SLAM_DEBUG("进入PaintMap函数for循环\n");
    SubmapId submap_id = submap_id_pose.id;  // 键
    cartographer::transform::Rigid3d pose = submap_id_pose.data.pose;
    int version = submap_id_pose.data.version;
    // 查询子图内容
    // response_proto用于存储转换后的 Protobuf 消息
    cartographer::mapping::proto::SubmapQuery::Response response_proto;
    // 该函数的作用是将子图转换为 Protobuf 消息的示例,并将结果存储在
    //  response_proto 对象中
    const std::string error =
        map_builder_->SubmapToProto(submap_id, &response_proto);
    // 如果转换成功，error是一个空字符串
    if (!error.empty()) {
      //------------修改-------------
      SLAM_INFO("%s", error.c_str());
      // 第二个参数是占位符，后面的参数为实际传入的参数，可以有多个
      locker.unlock();
      return;
    }
    int query_version = response_proto.submap_version();

    if (response_proto.textures_size() == 0) {
      // INFO_LOG的实现跟C语言中的printf函数一致
      SLAM_INFO("Responds of submap query is empty for submap :  %d",
                submap_id.submap_index);
      continue;
    }
    // 提取第一个Texture
    auto first_texture = response_proto.textures().begin();
    std::string cells = first_texture->cells();  // 压缩后的地图栅格数据
    int width = first_texture->width();          // 地图的宽
    int height = first_texture->height();        // 地图的高
    double resolution = first_texture->resolution();
    cartographer::transform::Rigid3d slice_pose =
        cartographer::transform::ToRigid3(first_texture->slice_pose());
    // 将地图栅格数据进行解压
    auto pixels = cartographer::io::UnpackTextureData(cells, width, height);
    // 填写SubmapSlice
    SubmapSlice &submap_slice = submap_slices[submap_id];
    submap_slice.pose = pose;
    submap_slice.metadata_version = version;
    submap_slice.version = query_version;
    submap_slice.width = width;
    submap_slice.height = height;
    submap_slice.slice_pose = slice_pose;
    submap_slice.resolution = resolution;
    submap_slice.cairo_data.clear();
    // 指向新创建的图像的指针
    submap_slice.surface =
        cartographer::io::DrawTexture(pixels.intensity, pixels.alpha, width,
                                      height, &submap_slice.cairo_data);
  }  // for (const auto& submap_id_pose : submap_poses)
  locker.unlock();
  SLAM_INFO("Get and draw %d %s", submap_slices.size(), "submaps");
  // 确认地图名
  map_name_ = map_name_ + ".smap";
  // 使用Submap绘制地图()
  auto painted_slices = PaintSubmapSlices(submap_slices, resolution);
  int width = cairo_image_surface_get_width(painted_slices.surface.get());
  int height = cairo_image_surface_get_height(painted_slices.surface.get());
  info.miMapWidth = width;
  info.miMapHeight = height;
  info.mdOriginXInWorld = -painted_slices.origin.x() * resolution;
  info.mdOriginYInWorld = (-height + painted_slices.origin.y()) * resolution;
  info.mdResolution = resolution;
  uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
      cairo_image_surface_get_data(painted_slices.surface.get()));

  // 写成Json格式的地图数据，后面用于加载这张地图，进行重定位
  // 具体方法：取出栅格地图每个占用栅格的坐标进行保存
  SLAM_DEBUG("开始转换地图格式\n");
  Json::Value map_json;
  Json::Value map_header;
  Json::Value point_append;
  Json::Value atring_add;
  map_header["mapType"] = "2D-Map";
  map_header["mapName"] = map_name_;
  atring_add["x"] = info.mdOriginXInWorld;
  atring_add["y"] = info.mdOriginYInWorld;
  map_header["minPos"] = atring_add;
  atring_add["x"] = info.mdOriginXInWorld + info.mdResolution * info.miMapWidth;
  atring_add["y"] =
      info.mdOriginYInWorld + info.mdResolution * info.miMapHeight;
  map_header["maxPos"] = atring_add;
  map_header["resolution"] = resolution;
  map_header["version"] = "1.0.6";
  map_json["header"] = map_header;
  int index_cnt = 0;

  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      Json::Value point_xy;
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      if (value > 50) {
        point_xy["x"] = info.mdOriginXInWorld + x * resolution;
        point_xy["y"] = info.mdOriginYInWorld + y * resolution;
        point_append[index_cnt] = point_xy;
        index_cnt++;
      }
    }
  }
  SLAM_INFO("Paint map with width: %d %s %d %s %d\n", width, "height:", height,
            "resolution:", resolution);
  map_json["normalPosList"] = point_append;

  // 创建文件夹并可以指定路径
  system("mkdir -p map");
  // std::string path =
  // "/home/fuzhou/svn/x86_branch_refactor/plugins/mapping_and_location_carto_copy/__build/test/map";

  // // 创建目录
  // int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::string full_map_data_path = config_.map_data_file_path + "/" + map_name_;

  // json_save(full_map_data_path.c_str(),map_json);(下面的代码就是json_save的实现)
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(map_json);
  str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
  str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
  str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
  std::ofstream ofs(
      full_map_data_path.c_str(),
      std::ios::
          trunc);  // 第一个参数代表文件路径，第二个参数表示以什么样的方式打开
  if (!ofs.is_open()) {
    SLAM_ERROR("文件打开失败");
    // system ("mkdir %s",config_.map_data_file_path.c_str());
    return;
  }
  ofs << str;
  ofs.close();
  SLAM_DEBUG("结束PaintMap函数\n");
}
void CartoMapping::CreatCartoModule() {
  // 传入的config_在构造函数中已经赋值，这里是两个函数的调用。
  auto map_builder_options = GetMapBuilderOptions(config_);
  auto trajectory_builder_options = GetTrajectoryBuilderOptions(config_);
  // 创建map_builder_
  map_builder_ =
      absl::make_unique<cartographer::mapping::MapBuilder>(map_builder_options);
  // 创建trajectory
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> sensor_ids;
  //----写死,雷达话题一般都不会改变，激光雷达是一定要使用的,不必判断------
  sensor_ids.insert(SensorId{SensorType::RANGE, "range0"});
  SLAM_DEBUG("###############use_imu_:%d use_odom_:%d\n", use_imu_, use_odom_);
  if (use_imu_) {
    sensor_ids.insert(SensorId{SensorType::IMU, "imu0"});
    SLAM_DEBUG("###############使用imu传感器\n");
  }
  if (use_odom_) {
    sensor_ids.insert(SensorId{SensorType::ODOMETRY, "odom0"});
    SLAM_DEBUG("###############使用odom传感器\n");
  }
  trajectory_id_ = map_builder_->AddTrajectoryBuilder(
      sensor_ids, trajectory_builder_options,
      [this](const int trajectory_id_, const ::cartographer::common::Time time,
             const cartographer::transform::Rigid3d local_pose,
             cartographer::sensor::RangeData range_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {});
  SLAM_INFO("Added trajectory with ID '%d'", trajectory_id_);
  // 获取trajectory_builder,实例化
  trajectory_builder_ = map_builder_->GetTrajectoryBuilder(trajectory_id_);
  if (!trajectory_builder_) {
    // LOG(ERROR) << "Get TrajectoryBuilder Failed";
    SLAM_ERROR("Get TrajectoryBuilder Failed");
  }
}
cartographer::sensor::TimedPointCloudData CartoMapping::ToCartoPointCloud(
    RadarSensoryMessage &data) {
  float raw_laser_resolution =
      data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament;
  int index_step = (int)(laser_resolution_ / raw_laser_resolution);
  // 计算实际使用的一帧雷达数据点的总数
  int point_num =
      fabs((laser_max_angle_ - laser_min_angle_) / laser_resolution_) + 1;
  cartographer::sensor::TimedPointCloud carto_data;
  carto_data.reserve(point_num);

  // 实际的点与点之间的时间间隔
  double timestep =
      data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament * index_step;
  // Cartographer设置一帧数据最后一个点的时间戳为0；
  float minus_time = -(point_num - 1) * timestep;
  SLAM_DEBUG("stepIncreament_  :%d  point_num :%d\n", stepIncreament_,
             point_num);
  // 计算配置后的激光数据第一个点的索引
  int start_i = fabs(laser_min_angle_ -
                     data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
                raw_laser_resolution;

  // 获取激光数据对应的雷达在机器人坐标系中的坐标
  double radar_x = config_.radar_position_x;
  double radar_y = config_.radar_position_y;
  double radar_theta = config_.radar_position_theta;
  LOG_INFO << "radar_x: " << radar_x << "radar_y: " << radar_y
           << "radar_theta:" << radar_theta;
  // 存储每一个点的3D位置坐标（Cartographer需要的是每一个激光点在tracking_frame下的坐标）以及时间戳，tmp_point为TimedRangefinderPoint结构体实例
  uint64_t t1 = gomros::common::GetCurrentTime_us();
  for (int i = 0; i < point_num; i++) {
    if (i == point_num - 1) {
      minus_time = 0.;
    }
    float x = data.mstruRadarMessage.mstruSingleLayerData
                  .mvPoints[start_i + i * index_step](0);
    float y = data.mstruRadarMessage.mstruSingleLayerData
                  .mvPoints[start_i + i * index_step](1);
    cartographer::sensor::TimedRangefinderPoint tmp_point{
        Eigen::Vector3f(
            x * std::cos(radar_theta) - y * std::sin(radar_theta) + radar_x,
            x * std::sin(radar_theta) + y * std::cos(radar_theta) + radar_y,
            0.),
        minus_time};  // 不要忘记➕平移向量
    carto_data.push_back(tmp_point);
    minus_time += timestep;
  }
  uint64_t t2 = gomros::common::GetCurrentTime_us();
  SLAM_INFO("转换一帧雷达数据中for循环的耗时：%lu\n", t2 - t1);

  double dt = ((start_i *
                data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament) +
               timestep * (point_num - 1)) *
              1000000;
  uint64_t lut = (uint64_t)dt;
  cartographer::common::Time time = ToCartoTime(
      data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp + lut);
  int64_t time2 = cartographer::common::ToUniversal(time);
  SLAM_INFO("carto时间为 %ld\n", time2);
  double t = (timestep * (point_num - 1)) * 1000000;
  SLAM_DEBUG("##########单帧时间增量 %f %f %f %d\n", t, timestep,
             data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament,
             stepIncreament_);
  return cartographer::sensor::TimedPointCloudData{
      time, Eigen::Vector3f(radar_x, radar_y, radar_theta),
      carto_data};  // 强度值取为空
}
cartographer::transform::Rigid3d CartoMapping::ToCartoRigid3d(Position &pose) {
  Eigen::Matrix<double, 3, 1> translation;
  translation << pose.mfX, pose.mfY, 0;
  // 求用四元数表示的旋转
  Eigen::Quaterniond quaternion;
  Eigen::Quaterniond rollAngle(
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));  // 分别转换成四元数
  Eigen::Quaterniond pitchAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond yawAngle(
      Eigen::AngleAxisd(pose.mfTheta, Eigen::Vector3d::UnitZ()));
  quaternion = yawAngle * pitchAngle * rollAngle;  // 分别表示三个四元数相乘
  return cartographer::transform::Rigid3d{translation, quaternion};
}
cartographer::sensor::ImuData CartoMapping::ToCartoImu(
    ImuSensoryMessage &data) {
  return cartographer::sensor::ImuData{
      ToCartoTime(data.time_stamp),  // 时间戳，单位：us
      Eigen::Vector3d(0, 0, 9.8),
      // carto里，上面三个分量分别代表重力加速度在x,y,z方向上的分量，当机器人水平运动时,Z轴与重力加速度方向重合，
      // 在x,y方向上的投影都为0，在z方向上的分量为本身，
      // 绕Z轴的旋转角速度，单位rad/s, 逆时针为正
      Eigen::Vector3d(0., 0., -data.z_omega)};
}
cartographer::common::Time CartoMapping::ToCartoTime(uint64_t timestamp_us) {
  return cartographer::common::FromUniversal(timestamp_us * 10);
  // carto里时间戳的单位是ns
}

std::vector<std::string> CartoMapping::SplitCString(std::string &str,
                                                    std::string delimit) {
  std::vector<std::string> result;
  size_t pos = str.find(delimit);
  str += delimit;  // 将分隔符加入到最后一个位置，方便分割最后一位
  while (pos != std::string::npos) {
    result.push_back(str.substr(0, pos));
    str = str.substr(
        pos +
        1);  // substr的第一个参数为起始位置，第二个参数为复制长度，默认为string::npos到最后一个位置
    pos = str.find(delimit);
  }
  return result;
}

void CartoMapping::ReadRadarData() {
  read_radar_finish_ = false;
  std::ifstream rawMapFileName(
      "/home/fuzhou/svn/x86_branch_refactor/plugins/"
      "mapping_and_location_carto_copy/data/laser_data.rawmap");
  usleep(100000);
  int index = 0;
  if (rawMapFileName.is_open()) {
    std::string line;
    // getline()是string流的函数，只能用于string类型的输入操作
    while (getline(rawMapFileName, line)) {
      index++;
      std::vector<std::string> laser_str = SplitCString(line, " ");
      int point_num = stoi(laser_str[1]);
      RadarSensoryMessage laser_data;
      uint64_t time_stamp = stoul(laser_str[2]);
      laser_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp =
          time_stamp;
      float time_increament = stof(laser_str[3]);
      SLAM_ERROR("time_increament:%f \n", time_increament);
      laser_data.mstruRadarMessage.mstruRadarHeaderData.mfTimeIncreament =
          time_increament;

      float angle_min = stof(laser_str[4]);
      laser_min_angle_ = angle_min;
      laser_data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin = angle_min;

      float angle_max = stof(laser_str[5]);
      laser_max_angle_ = angle_max;
      laser_data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax = angle_max;

      float angle_resolution = stof(laser_str[6]);
      laser_resolution_ = angle_resolution;
      laser_data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament =
          angle_resolution;
      // 开始读一行数据，索引从7开始的所有激光点的距离数据
      uint64_t t1 = gomros::common::GetCurrentTime_us();
      for (int i = 0; i < point_num; i++) {
        float len = stof(laser_str[i + 7]);
        double x = cos((angle_min + i * angle_resolution)) * len;
        double y = sin((angle_min + i * angle_resolution)) * len;
        laser_data.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(
            Eigen::Vector2d(x, y));
        // 使用empalce_back可以直接传入x,y.而不需要进行构造一个Vector2d类型的值
      }
      uint64_t t2 = gomros::common::GetCurrentTime_us();
      SLAM_INFO("从文件中读取第%d帧雷达数据耗时：%lu\n", index, t2 - t1);
      // usleep(100000);
      lidar_list_from_file_.push_back(laser_data);
    }
  } else {
    SLAM_ERROR("Failed to open file.\n");
  }
  read_radar_finish_ = true;
  rawMapFileName.close();
  SLAM_INFO("结束radar离线数据读取线程\n");
}
void CartoMapping::ReadOdomData() {
  read_odom_finish_ = false;  // 更新标志位
  std::string line;
  float pose_x = 0.0;
  float pose_y = 0.0;
  float pose_theta = 0.0;
  uint64_t pose_time = 0;
  std::ifstream rawMapFileName(
      "/home/fuzhou/svn/x86_branch_refactor/plugins/"
      "mapping_and_location_carto_copy/data/odom_data.rawmap");
  usleep(100000);
  Position last_pose;
  int i = 0;
  if (rawMapFileName.is_open()) {
    while (getline(rawMapFileName, line)) {
      i++;
      std::vector<std::string> pose_str = SplitCString(line, " ");
      pose_time = std::stoul(pose_str[1]);  // C++的函数
      // SLAM_INFO("从文件中读取第%d帧雷达数据耗时：%lu\n", index, t2 - t1);
      pose_x = std::stof(pose_str[2]);
      pose_y = std::stof(pose_str[3]);
      pose_theta = std::stof(pose_str[4]);
      Position pose(pose_time, pose_x, pose_y, pose_theta);
      last_pose.mlTimestamp = pose.mlTimestamp;
      odom_list_from_file_.push_back(pose);
      // DEBUG_LOG << "读取第" << i << "帧里程计数据";
      SLAM_DEBUG("读取第%d帧里程计数据\n", i);
      // usleep(50000);
    }
  } else {
    SLAM_ERROR("Failed to open file.\n");
  }
  read_odom_finish_ = true;
  SLAM_INFO("结束odom离线数据读取线程");
}
void CartoMapping::ReadImuData() {
  read_imu_finish_ = false;  // 更新标志位
  std::string line;
  uint64_t imu_time = 123456789;
  float imu_z_angular_v = 0.0;
  float imu_z_angle = 0.0;
  float imu_forword_acce = 0.0;
  std::ifstream rawMapFileName("./data/imu_data.rawmap");
  usleep(100000);
  if (rawMapFileName.is_open()) {
    while (getline(rawMapFileName, line)) {
      std::vector<std::string> imu_str = SplitCString(line, " ");
      imu_time = std::stoul(imu_str[1]);
      imu_z_angular_v = std::stof(imu_str[2]);
      imu_forword_acce = std::stof(imu_str[3]);
      imu_z_angle = std::stof(imu_str[4]);
      ImuSensoryMessage imu_data = {imu_z_angular_v, imu_forword_acce,
                                    imu_z_angle, imu_time};
      ImuDataConversionAndAddFunc(imu_data);
      // usleep(10000);
    }
  } else {
    SLAM_ERROR("Failed to open file.\n");
  }
  read_imu_finish_ = true;
  SLAM_INFO("结束imu离线数据读取线程\n");
}
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
