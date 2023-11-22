/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-25 10:30:26
 * @LastEditTime: 2023-04-03 14:12:07
 * @Author: lcfc-desktop
 */
#include "include/location_lib/grid_location.h"

#include <fstream>

#include "common_lib/time_utils.h"
#include "include/location_lib/scan_matching/scan_matching.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

/**
 * @brief
 *
 * @param z
 * @return double
 */
double normalize(double z) { return atan2(sin(z), cos(z)); }

/**
 * @brief
 *
 * @param ang_a
 * @param ang_b
 * @return double
 */
double angle_diff(double ang_a, double ang_b) {
  double d1_ang, d2_ang;
  ang_a = normalize(ang_a);
  ang_b = normalize(ang_b);
  d1_ang = ang_a - ang_b;
  d2_ang = 2 * M_PI - fabs(d1_ang);
  if (d1_ang > 0) d2_ang *= -1.0;
  if (fabs(d1_ang) < fabs(d2_ang))
    return (d1_ang);
  else
    return (d2_ang);
}

/**
 * @brief 初始均布粒子
 *
 * @param arg
 * @return pf_vector_t
 */
amcl::pf_vector_t uniformPoseGenerator(void *arg) {
  amcl::map_t *map = reinterpret_cast<amcl::map_t *>(arg);

  double min_x, max_x, min_y, max_y;

  min_x = map->origin_x - (map->size_x * map->scale) / 2.0;
  max_x = map->origin_x + (map->size_x * map->scale) / 2.0;
  min_y = map->origin_y - (map->size_y * map->scale) / 2.0;
  max_y = map->origin_y + (map->size_y * map->scale) / 2.0;

  amcl::pf_vector_t p;
  int ii = 0;
  for (;;) {
    ii++;
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);

    if (MAP_VALID(map, i, j) &&
        (map->cells[MAP_INDEX(map, i, j)].occ_state == -1))
      break;
  }

  return p;
}

/**
 * @brief Construct a new Grid Location:: Grid Location object
 *
 */
GridLocation::GridLocation(const GridLocationConfig &config) {
  m_LocationConfig = config;
  first_laser_ = true;
  stop_locate_ = false;

  pthread_mutex_init(&listen_pose_mutex, nullptr);

  pthread_mutex_init(&mutex_pose, nullptr);
  pthread_mutex_init(&mutex_odom_list, nullptr);

  pthread_mutex_init(&mutex_pose_raw, nullptr);
  pthread_mutex_init(&mutex_odom_list_raw, nullptr);
  pthread_mutex_init(&mutex_finish_locate, nullptr);
  m_MapLoadSuccess = false;
  laser_min_angle_ = m_LocationConfig.mapping_start_angle / 180 * M_PI;
  laser_max_angle_ = m_LocationConfig.mapping_end_angle / 180 * M_PI;
  laser_resolution_ = m_LocationConfig.laser_resolution / 180 * M_PI;
  laser_max_range_ = m_LocationConfig.laser_max_range;
  laser_min_range_ = m_LocationConfig.laser_min_range;
  ParamInitial();

  m_RobotStopCnt = 128;
}

/**
 * @brief Destroy the Grid Location:: Grid Location object
 *
 */
GridLocation::~GridLocation() {
  pthread_mutex_destroy(&listen_pose_mutex);
  pthread_mutex_destroy(&mutex_pose);
  pthread_mutex_destroy(&mutex_odom_list);
  pthread_mutex_destroy(&mutex_pose_raw);
  pthread_mutex_destroy(&mutex_odom_list_raw);
  pthread_mutex_destroy(&mutex_finish_locate);

  if (m_pMap != nullptr) {
    map_free(m_pMap);
    m_pMap = nullptr;
  }
  if (m_pPf != nullptr) {
    pf_free(m_pPf);
    m_pPf = nullptr;
  }
  if (m_pOdom != nullptr) {
    delete m_pOdom;
    m_pOdom = nullptr;
  }
  if (m_pLaser != nullptr) {
    delete m_pLaser;
    m_pLaser = nullptr;
  }
}

void GridLocation::SetConfiguration(void *config) {
  m_LocationConfig = *(reinterpret_cast<GridLocationConfig *>(config));
}

void GridLocation::SetMapData(MapSharedPointer grid_map_) {
  p_grid_map_ = grid_map_;
}

gomros::message::Position GridLocation::GetCurrentPosition() {
  pthread_mutex_lock(&mutex_pose);
  Position current_pose = current_posotion_;
  pthread_mutex_unlock(&mutex_pose);
  return current_pose;
}

bool GridLocation::IsFinishLocate() {
  pthread_mutex_lock(&mutex_finish_locate);
  bool finish_locate = finish_location_;
  pthread_mutex_unlock(&mutex_finish_locate);
  return finish_locate;
}
/**
 * @brief
 *
 * @param data
 */
void GridLocation::HandleLaserData(const RadarSensoryMessage &data) {
  // Position odom_temp;
  m_RadarPose = ExpolateCurrentPositionTimeRaw(
      data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
  m_LastRadarData = data.mstruRadarMessage;
  if (first_laser_) {
    first_laser_ = false;
    laser_resolution_ = fmod(laser_resolution_ + 5 * M_PI, 2 * M_PI) - M_PI;
    laser_resolution_ = fabs(laser_resolution_);
    angle_increment_ = m_LastRadarData.mstruRadarHeaderData.mfAngleIncreament;
    angle_increment_ = fmod(angle_increment_ + 5 * M_PI, 2 * M_PI) - M_PI;
    angle_increment_ = fabs(angle_increment_);
    stepIncreament_ = std::round(laser_resolution_ / angle_increment_);
    float float_stepIncreament = laser_resolution_ / angle_increment_;
    if ((laser_resolution_ - angle_increment_) < -1e-4) {
      stepIncreament_ = 1;
      laser_resolution_ = angle_increment_;
      SLAM_ERROR("定位模块配置文件角度分辨率小于雷达数据分辨率\n");
    }
    if (fabs(float_stepIncreament - stepIncreament_) > 1e-4) {
      laser_resolution_ = angle_increment_ * stepIncreament_;
      SLAM_ERROR(
                "定位模块配置文件角度分辨率必须是雷达数据分辨率整数倍\n");
    }
    if (laser_min_angle_ < m_LastRadarData.mstruRadarHeaderData.mfAngleMin) {
      laser_min_angle_ = m_LastRadarData.mstruRadarHeaderData.mfAngleMin;
      SLAM_ERROR(
                "定位模块配置文件雷达最小使用角度小于雷达数据最小角度\n");
    }
    if (laser_max_angle_ > m_LastRadarData.mstruRadarHeaderData.mfAngleMax) {
      laser_max_angle_ = m_LastRadarData.mstruRadarHeaderData.mfAngleMax;
      SLAM_ERROR(
                "定位模块配置文件雷达最大使用角度大于雷达数据最大角度\n");
    }
    laser_min_angle_ = fmod(laser_min_angle_ + 5 * M_PI, 2 * M_PI) - M_PI;
    laser_max_angle_ = fmod(laser_max_angle_ + 5 * M_PI, 2 * M_PI) - M_PI;
    if (laser_min_range_ < m_LastRadarData.mstruRadarHeaderData.mfRangeMin) {
      laser_min_range_ = m_LastRadarData.mstruRadarHeaderData.mfRangeMin;
      SLAM_ERROR(
                "定位模块配置文件雷达最小使用距离小于雷达数据最小距离\n");
    }
    if (laser_max_range_ > m_LastRadarData.mstruRadarHeaderData.mfRangeMax) {
      laser_max_range_ = m_LastRadarData.mstruRadarHeaderData.mfRangeMax;
      SLAM_ERROR(
                "定位模块配置文件雷达最大使用距离大于雷达数据最大距离\n");
    }
    SLAM_DEBUG(
              "定位模块配置雷达角度分辨率为%f, 起始角度为%f, 终止角度为%f, "
              "最小距离为%f, 最大距离为%f\n",
              laser_resolution_, laser_min_angle_, laser_max_angle_,
              laser_min_range_, laser_max_range_);
  }
  pthread_mutex_lock(&mutex_finish_locate);
  bool finish_locate = finish_location_;
  pthread_mutex_unlock(&mutex_finish_locate);
  if (finish_locate) {  // 如果初始定位已经完成，启动实时定位线程
    if (m_RobotStopCnt < 70 && !stop_locate_) {
      // if (!stop_locate_) {
      // 启动粒子滤波器
      ParticleFilterRun(true);
      // ParticleFilterRun(false);
      // 雷达点云数据转换,准备scan_matching
      if (m_LocationConfig.use_scan_matching) {
        RadarSensoryMessage tmp;
        int count = 0;
        for (const auto &point :
             data.mstruRadarMessage.mstruSingleLayerData.mvPoints) {
          double x = cos(m_RadarPosition.mfTheta) * point(0) -
                     sin(m_RadarPosition.mfTheta) * point(1) +
                     m_RadarPosition.mfX;
          double y = sin(m_RadarPosition.mfTheta) * point(0) +
                     cos(m_RadarPosition.mfTheta) * point(1) +
                     m_RadarPosition.mfY;
          tmp.add_point(
              x, y,
              data.mstruRadarMessage.mstruSingleLayerData.mvIntensities[count]);
          count++;
        }
        int record_cnt =
            fabs((laser_max_angle_ - laser_min_angle_) / laser_resolution_);
        RadarSensoryMessage dataDisposed;
        dataDisposed.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp =
            data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
        int range_count =
            fabs((data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax -
                  data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
                 data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament);
        int start_i =
            fabs(laser_min_angle_ -
                 data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
            angle_increment_;
        record_cnt = record_cnt * stepIncreament_;
        for (int i = start_i; i <= (start_i + record_cnt); i += stepIncreament_)
          dataDisposed.add_point(
              tmp.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](0),
              tmp.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](1),
              tmp.mstruRadarMessage.mstruSingleLayerData.mvIntensities[i]);
        // scan_matching
        Position pose_scan = DoScanMatching(dataDisposed, m_LocatePose);
        UpdateLastPosition(pose_scan);
      } else {
        // UpdateLastPosition(m_LocatePose);
      }
    }
  }

  m_LaserRecvCnt++;
  if (m_LaserRecvCnt >= 10) {
    m_LaserRecvCnt = 6;
  }
}

/**
 * @brief
 *
 * @param data
 */
void GridLocation::HandleOdomData(const message::OdometerMessage &data) {
  message::Position pose = data.mclDeltaPosition;
  AddOdomDataRaw(pose);
  AddOdomData(pose);
  // if (fabs(data.mfAngularSpeed) <= 0.05 && fabs(data.mfLinearSpeed) <= 0.01)
  // {
  if (fabs(pose.mfX) <= 0.001 && fabs(pose.mfY) <= 0.001 &&
      fabs(pose.mfTheta) <= 0.001) {
    if (m_RobotStopCnt < 65535) {
      m_RobotStopCnt++;
    }
  } else {
    m_RobotStopCnt = 0;
  }
  if (m_OdomList.size() > 200) FlushOdomData();
  if (m_OdomListRaw.size() > 200) FlushOdomDataRaw();
}

/**
 * @brief
 *
 * @param data
 * @param pose
 * @return Position
 */
Position GridLocation::DoScanMatching(const RadarSensoryMessage &data,
                                      Position pose) {
  assert(p_grid_map_);
  m_DoingScanMatching = true;
  float min_dist = m_LocationConfig.scan_matching_min_distance;
  float max_dist = m_LocationConfig.scan_matching_max_distance;
  ScanMatchingOptions opt;
  opt.occupied_space_cost_factor = m_LocationConfig.occupied_space_cost_factor;
  opt.translation_delta_cost_factor =
      m_LocationConfig.translation_delta_cost_factor;
  opt.rotation_delta_cost_factor = m_LocationConfig.rotation_delta_cost_factor;
  opt.num_threads = m_LocationConfig.num_threads;
  opt.max_num_iterations = m_LocationConfig.max_num_iterations;
  Position optimism;
  optimism = ScanMatching(pose, p_grid_map_.get(), data, opt);
  optimism.mlTimestamp =
      data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
  m_DoingScanMatching = false;

  return optimism;
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param theta
 */
void GridLocation::SetInitPose(float x, float y, float theta) {
  m_InitPoseX = x;
  m_InitPoseY = y;
  m_InitPoseTheta = theta;
  initial_posotion_.mfX = m_InitPoseX;
  initial_posotion_.mfY = m_InitPoseY;
  initial_posotion_.mfTheta = m_InitPoseTheta;
  initial_posotion_.mlTimestamp = gomros::common::GetCurrentTime_us();
  FlushOdomData();
  FlushOdomDataRaw();
  current_posotion_ = initial_posotion_;
  m_CurrentPoseRaw = initial_posotion_;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool GridLocation::StartGlobalLocating(bool relocate) {
  m_relocate_ = relocate;
  pthread_create(&m_GlobalLocatingThread, nullptr, GlobalLocatingThreadFunction,
                 this);
  return true;
}

/**
 * @brief
 *
 * @param param
 * @return void*
 */
void *GridLocation::GlobalLocatingThreadFunction(void *param) {
  GridLocation *ptr = reinterpret_cast<GridLocation *>(param);
  pthread_detach(pthread_self());
  ptr->MainParticleRun();
  return nullptr;
}

/**
 * @brief 主粒子滤波运行
 *
 */
void GridLocation::MainParticleRun() {
  pthread_mutex_lock(&mutex_finish_locate);
  finish_location_ = false;
  pthread_mutex_unlock(&mutex_finish_locate);
  sleep(1);
  if (ini_location_continue_) {
    ini_location_continue_ = false;
    usleep(200000);
  }
  stop_locate_ = false;
  usleep(500000);
  freeMapDependentMemory();
  m_LaserRecvCnt = 0;
  m_pMap = DoMapOpen();
  SLAM_INFO("AMCL地图加载完毕\n");
  JhInitialPf();
  SLAM_INFO("定位线程初始化完毕\n");
  // 开机执行初始化一次，初始化粒子滤波器，odom，激光传感器
  bool b_ini_located = false;  // 初始定位初始化变量
  char force_update;           // 定时强制触发粒子定位

  uint64_t init_time = gomros::common::GetCurrentTime_us();
  uint64_t real_time = init_time;
  double delta_t;
  int LOCATE_TIME_EXCEED = 70 * 1000;
  m_LocateStep = 1;
  if (m_relocate_) {
    SLAM_INFO("全局初始化定位线程运行开始\n");
    // 初始定位是否成功
    bool ini_location_success = false;
    ini_location_continue_ = true;
    while (ini_location_continue_) {
      // 机器人初始定位超时判断,start_global_locating中为0
      // 在处理激光数据的函数里，从6到10变化
      // 猜测前5帧
      if (m_LaserRecvCnt <= 5) {
        // printf("m_LaserRecvCnt = %d......\n", m_LaserRecvCnt);
        usleep(100000);
        continue;
      }
      if (b_ini_located == false) {
        b_ini_located = true;
        init_time = gomros::common::GetCurrentTime_us();
        delta_t = 0;  // 清零delta_t
      }
      real_time = gomros::common::GetCurrentTime_us();
      delta_t = static_cast<double>(real_time - init_time) / 1000;  // 单位ms
      // printf("delta_t = %f......\n", delta_t);
      if (delta_t > LOCATE_TIME_EXCEED) {
        // 超时强制定位结束
        delta_t = 0;
        SLAM_INFO(
            "定位超时，初始定位失败\n最大置信度的位姿点为x = %f, y = %f, theta "
            "= %f 度"
            ", 得分 "
            "= "
            "%f\n",
            m_AmclPose.v[0], m_AmclPose.v[1], m_AmclPose.v[2] / M_PI * 180,
            m_LocateScore);
        ini_location_continue_ = false;
        ini_location_success = false;
      } else {
        ParticleFilterRun(true);  // 强制启动粒子滤波器
        ini_location_success = InitialLocation();
      }
      usleep(20000);
    }
    bool finish_location = ini_location_success;
    if (!finish_location) {
      SLAM_ERROR("初始化定位失败\n");
    } else {
      SLAM_INFO("初始化定位成功\n");
      FlushOdomDataRaw();
      FlushOdomData();
      pthread_mutex_lock(&mutex_pose_raw);
      m_CurrentPoseRaw = current_posotion_;
      m_LastOdomPose.v[0] = current_posotion_.mfX;
      m_LastOdomPose.v[1] = current_posotion_.mfY;
      m_LastOdomPose.v[2] = current_posotion_.mfTheta;
      pthread_mutex_unlock(&mutex_pose_raw);
    }
    pthread_mutex_lock(&mutex_finish_locate);
    finish_location_ = finish_location;
    pthread_mutex_unlock(&mutex_finish_locate);
    SLAM_INFO("全局初始化定位线程运行结束\n");
  } else {
    SLAM_INFO("################不启用全局初始化定位\n");
    FlushOdomData();
    pthread_mutex_lock(&mutex_finish_locate);
    finish_location_ = true;
    pthread_mutex_unlock(&mutex_finish_locate);
  }
}

/**
 * @brief 机器人初始定位线程函数
 *
 * @return true
 * @return false
 */
bool GridLocation::InitialLocation() {
  float particle_radius = 0.2;
  if (ini_location_continue_) {  // 定位步骤激活
    if (m_LocateScore > 0.90) {
      amcl::pf_vector_t pf_init_pose_mean = amcl::pf_vector_zero();
      amcl::pf_matrix_t pf_init_pose_cov = amcl::pf_matrix_zero();
      // 使用得到的位置重新初始化粒子群
      if (m_LocateStep < 3) {  // 启动二次重定位步骤
        usleep(30000);
        m_LocateStep++;
        SLAM_INFO(
            "\n==================\n启动重--%d--撒粒子步骤\n=================="
            "\n",
            m_LocateStep);
        // FlushOdomData();
        // FlushOdomDataRaw();
        pthread_mutex_lock(&mutex_pose);
        // m_CurrentPoseRaw = current_posotion_;
        pf_init_pose_mean.v[0] = current_posotion_.mfX;
        pf_init_pose_mean.v[1] = current_posotion_.mfY;
        pf_init_pose_mean.v[2] = current_posotion_.mfTheta;
        pthread_mutex_unlock(&mutex_pose);
        pf_init_pose_cov.m[0][0] = particle_radius;
        pf_init_pose_cov.m[1][1] = particle_radius;
        pf_init_pose_cov.m[2][2] = 0.1;
        pf_init(m_pPf, pf_init_pose_mean, pf_init_pose_cov);
        m_PfInit = false;
      } else {
        SLAM_INFO(
            "初始定位成功\n最大置信度的位姿点为x = %f, y = %f, a = %f, score "
            "= "
            "%f\n",
            m_AmclPose.v[0], m_AmclPose.v[1], m_AmclPose.v[2] / M_PI * 180,
            m_LocateScore);
        FlushOdomData();
        SLAM_INFO("==========================================定位完成\n");
        ini_location_continue_ = false;
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief 启动粒子滤波器
 *
 * @param fup
 * @return true
 * @return false
 */
bool GridLocation::ParticleFilterRun(bool force_update) {
  pthread_mutex_lock(&listen_pose_mutex);
  // 定义粒子滤波所需局部变量
  amcl::pf_vector_t delta;
  bool resampled = false;
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  double weight = 0.0;
  bool odom_update = false;
  bool laser_update = false;
  char resample_upd = 0;
  amcl::pf_sample_set_t *set;
  delta = amcl::pf_vector_zero();
  amcl::pf_vector_t CuurentOdomPose;
  Position start_pose = m_RadarPose;

  CuurentOdomPose = amcl::pf_vector_zero();
  CuurentOdomPose.v[0] = m_RadarPose.mfX;
  CuurentOdomPose.v[1] = m_RadarPose.mfY;
  CuurentOdomPose.v[2] = m_RadarPose.mfTheta;
  uint64_t pre_time = gomros::common::GetCurrentTime_us();
  Position pre_amcl = ExpolateCurrentPosition();
  uint64_t cc_time;
  resampled = false;
  // 如果初始化完成，则判断是否更新
  if (m_PfInit) {
    // 当前里程计与上一里程计的差值
    delta.v[0] = CuurentOdomPose.v[0] - m_LastOdomPose.v[0];
    delta.v[1] = CuurentOdomPose.v[1] - m_LastOdomPose.v[1];
    delta.v[2] = angle_diff(CuurentOdomPose.v[2], m_LastOdomPose.v[2]);
    // 如果里程计位移增量足够大就更新里程计数据
    odom_update = fabs(delta.v[0]) > m_DThresh ||
                  fabs(delta.v[1]) > m_DThresh || fabs(delta.v[2]) > m_AThresh;
    // 是否强制更新
    if (force_update) odom_update = true;
    // 是否更新里程计
    if (odom_update) {
      // 更新里程计的同时更新雷达
      laser_update = true;
      amcl::AMCLOdomData odata;
      odata.pose = CuurentOdomPose;
      odata.delta = delta;
      // Use the action data to update the filter
      m_pOdom->UpdateAction(m_pPf,
                            reinterpret_cast<amcl::AMCLSensorData *>(&odata));
    }
  } else {
    // 没有初始化就强制更新雷达数据
    m_PfInit = true;
    // Pose at last filter update
    m_LastOdomPose = CuurentOdomPose;
    // Filter is now initialized
    // Should update sensor data
    laser_update = true;
    resampled = true;
    m_ResampleCount = 0;
  }
  // 是否更新雷达数据
  if (laser_update) {
    amcl::AMCLLaserData ldata;
    ldata.sensor = m_pLaser;
    int record_cnt =
        fabs((laser_max_angle_ - laser_min_angle_) / laser_resolution_);
    ldata.range_count = record_cnt + 1;  // 用于定位的激光束数目
    int range_count = fabs(
        (m_LastRadarData.mstruRadarHeaderData.mfAngleMax -
         m_LastRadarData.mstruRadarHeaderData.mfAngleMin) /
        m_LastRadarData.mstruRadarHeaderData.mfAngleIncreament);  // 雷达数据
    int start_i = fabs(laser_min_angle_ -
                       m_LastRadarData.mstruRadarHeaderData.mfAngleMin) /
                  angle_increment_;
    record_cnt = record_cnt * stepIncreament_;
    float angle_increment_step =
        static_cast<float>(stepIncreament_ * angle_increment_);
    ldata.range_max = laser_max_range_;
    ldata.ranges = new double[ldata.range_count][2];
    assert(ldata.ranges);
    // 整理激光数据
    int j = 0;
    for (int i = start_i; i <= (start_i + record_cnt); i += stepIncreament_) {
      double xp = m_LastRadarData.mstruSingleLayerData.mvPoints[i](0);
      double yp = m_LastRadarData.mstruSingleLayerData.mvPoints[i](1);
      double laser_len = sqrt(xp * xp + yp * yp);
      if (laser_len < laser_min_range_) {
        ldata.ranges[j][0] = laser_max_range_ + 1;
      } else {
        ldata.ranges[j][0] = laser_len;
      }
      ldata.ranges[j][1] =
          laser_min_angle_ + (j * angle_increment_step);  // 角度
      j++;
    }
    m_pLaser->UpdateSensor(m_pPf,
                           reinterpret_cast<amcl::AMCLSensorData *>(&ldata));
    laser_update = false;

    // 保存上一步的位置为激光相对于odo的位置
    // Resample the particles
    m_LastOdomPose = CuurentOdomPose;
    // 是否达到重采样间隔
    if ((++m_ResampleCount % m_ResampleInterval) == 0) {
      m_ResampleCount = 0;
      pf_update_resample(m_pPf, resample_upd);
      resampled = true;
    }
    set = m_pPf->sets + m_pPf->current_set;
    if (resampled) {
      // Read out the current hypotheses
      // int total_count = 0;
      int score_count = 0;
      std::vector<amcl_hyp_t> hyps;
      hyps.resize(m_pPf->sets[m_pPf->current_set].cluster_count);

      for (int hyp_count = 0;
           hyp_count < m_pPf->sets[m_pPf->current_set].cluster_count;
           hyp_count++) {
        amcl::pf_vector_t pose_mean;
        amcl::pf_matrix_t pose_cov;
        if (!pf_get_cluster_stats(m_pPf, hyp_count, &weight, &pose_mean,
                                  &pose_cov)) {
          SLAM_ERROR("Couldn't get stats on cluster\n");
          break;
        }
        hyps[hyp_count].weight = weight;
        hyps[hyp_count].pf_pose_mean = pose_mean;
        hyps[hyp_count].pf_pose_cov = pose_cov;

        if (hyps[hyp_count].weight > max_weight) {
          max_weight = hyps[hyp_count].weight;
          max_weight_hyp = hyp_count;
        }
      }
      pthread_mutex_lock(&mutex_finish_locate);
      bool finish_location = finish_location_;
      pthread_mutex_unlock(&mutex_finish_locate);
      if (finish_location == false) {
        for (int i = 0; i < set->sample_count; i++) {
          amcl::pf_sample_t *sample = set->samples + i;
          if (DisAmong(hyps[max_weight_hyp].pf_pose_mean, sample->pose, 0.10)) {
            score_count++;
          }
        }
      }
      // 判断是否收敛，判断依据是集中在pf_pose_mean附近的粒子的比例
      hyps[max_weight_hyp].score = static_cast<float>(score_count) /
                                   (static_cast<float>(set->sample_count));
      m_LocateScore = hyps[max_weight_hyp].score;
      if (max_weight > 0.0) {
        // 最高得分的位置值赋值给全局变量LWG
        cc_time = gomros::common::GetCurrentTime_us();
        m_AmclPose.v[0] = hyps[max_weight_hyp].pf_pose_mean.v[0];
        m_AmclPose.v[1] = hyps[max_weight_hyp].pf_pose_mean.v[1];
        m_AmclPose.v[2] = hyps[max_weight_hyp].pf_pose_mean.v[2];
      }

      Position end_amcl = ExpolateCurrentPosition();
      Position delt_pre_end_amcl = end_amcl - pre_amcl;
      int delt_time = end_amcl.mlTimestamp - pre_amcl.mlTimestamp;
      m_AmclrPose.mfX = m_AmclPose.v[0];
      m_AmclrPose.mfY = m_AmclPose.v[1];
      m_AmclrPose.mfTheta = m_AmclPose.v[2];
      m_AmclrPose.mlTimestamp = gomros::common::GetCurrentTime_us();
      // m_AmclrPose.mlTimestamp = gomros::common::GetCurrentTime_us();
      uint64_t cur_time = gomros::common::GetCurrentTime_us();
      Position end_pose = ExpolateCurrentPositionTimeRaw(cur_time);
      // UpdateLastPosition(m_AmclrPose);  // 更新现在的位姿
      pthread_mutex_lock(&mutex_pose);
      current_posotion_ = m_AmclrPose + (end_pose - start_pose);
      // current_posotion_ = m_AmclrPose;
      current_posotion_.mlTimestamp = cur_time;
      m_LocatePose = current_posotion_;
      pthread_mutex_unlock(&mutex_pose);
      UpdateOdomListTimestamp(cur_time);
    }
  }
  pthread_mutex_unlock(&listen_pose_mutex);
}

/**
 * @brief 两个空间点之间的距离
 *
 * @param pos1
 * @param pos2
 * @param dis
 * @return true
 * @return false
 */
bool GridLocation::DisAmong(amcl::pf_vector_t pos1, amcl::pf_vector_t pos2,
                            double dis) {
  double dis_ret;
  double dis_x = pos1.v[0] - pos2.v[0];
  if (fabs(dis_x) > dis) return false;
  double dis_y = pos1.v[1] - pos2.v[1];
  if (fabs(dis_y) > dis) return false;

  double v02 = (dis_x) * (dis_x);
  double v12 = (dis_y) * (dis_y);
  dis_ret = sqrt(v02 + v12);
  if (dis_ret > dis)
    return false;
  else
    return true;
}

/**
 * @brief
 *
 * @param pose_change
 */
void GridLocation::AddOdomDataRaw(const Position &pose_change) {
  uint64_t timestamp = pose_change.mlTimestamp;
  pthread_mutex_lock(&mutex_pose_raw);
  if (timestamp < m_CurrentPoseRaw.mlTimestamp) {
    SLAM_ERROR("##AMCL raw data timestamp is too small  exit\n");
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

/**
 * @brief
 *
 * @param pose_change
 */
void GridLocation::AddOdomData(const Position &pose_change) {
  uint64_t timestamp = pose_change.mlTimestamp;
  pthread_mutex_lock(&mutex_pose);
  if (timestamp < current_posotion_.mlTimestamp) {
    SLAM_ERROR("##amcl realtime data timestamp is too late  exit\n");
  }
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
  pthread_mutex_unlock(&mutex_pose);
}

/**
 * @brief 最新的amcl结果,当前位姿要将后面的位姿变化乘上去
 *
 * @param pose
 */
void GridLocation::UpdateLastPosition(const Position &pose) {
  pthread_mutex_lock(&mutex_pose);
  if (pose.mlTimestamp > current_posotion_.mlTimestamp) {
    current_posotion_ = pose;
    UpdateOdomListTimestamp(pose.mlTimestamp);
  }
  pthread_mutex_unlock(&mutex_pose);
}

/**
 * @brief
 *
 * @param timestamp_us
 */
void GridLocation::UpdateOdomListTimestamp(uint64_t timestamp_us) {
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
 * @brief 将实时位姿更新到最新里程计处
 *
 */
void GridLocation::FlushOdomData() {
  pthread_mutex_lock(&mutex_pose);
  Position pose = current_posotion_;
  pthread_mutex_lock(&mutex_odom_list);

  while (!m_OdomList.empty()) {
    if (m_OdomList.front().mlTimestamp > pose.mlTimestamp) {
      pose = pose * m_OdomList.front();
    }
    m_OdomList.pop_front();
  }
  current_posotion_ = pose;
  pthread_mutex_unlock(&mutex_odom_list);
  pthread_mutex_unlock(&mutex_pose);
}

void GridLocation::StopLocate() { stop_locate_ = true; }

/**
 * @brief 将纯里程计位姿更新到原始里程计数据最新时间出
 *
 */
void GridLocation::FlushOdomDataRaw() {
  pthread_mutex_lock(&mutex_pose_raw);
  Position pose = m_CurrentPoseRaw;
  pthread_mutex_lock(&mutex_odom_list_raw);
  while (!m_OdomListRaw.empty()) {
    pose = pose * m_OdomListRaw.front();
    m_OdomListRaw.pop_front();
  }
  m_CurrentPoseRaw = pose;
  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
}

/**
 * @brief
 *
 * @return Position
 */
Position GridLocation::ExpolateCurrentPosition() {
  Position pose_temp;
  pthread_mutex_lock(&mutex_finish_locate);
  bool finish_location = finish_location_;
  pthread_mutex_unlock(&mutex_finish_locate);
  if (!finish_location) return pose_temp;  // 默认为0 0 0 0
  pthread_mutex_lock(&mutex_pose);
  Position ret = current_posotion_;
  pthread_mutex_lock(&mutex_odom_list);
  for (Position &p : m_OdomList) {
    if (p.mlTimestamp > ret.mlTimestamp) {
      ret = ret * p;
    }
  }
  pthread_mutex_unlock(&mutex_odom_list);
  pthread_mutex_unlock(&mutex_pose);
  return ret;
}

/**
 * @brief
 *
 * @param timestamp
 * @return Position
 */
Position GridLocation::ExpolateCurrentPositionTimeRaw(uint64_t timestamp) {
  pthread_mutex_lock(&mutex_pose_raw);
  Position ret = m_CurrentPoseRaw;
  pthread_mutex_lock(&mutex_odom_list_raw);
  for (Position &p : m_OdomListRaw) {
    if (p.mlTimestamp > ret.mlTimestamp && (p.mlTimestamp <= timestamp)) {
      ret = ret * p;
    }
  }
  m_CurrentPoseRaw = ret;
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

/**
 * @brief
 *
 * @param timestamp
 * @return Position
 */
Position GridLocation::ExpolateCurrentPositionTime(uint64_t timestamp) {
  Position pos_temp;
  pthread_mutex_lock(&mutex_finish_locate);
  bool finish_location = finish_location_;
  pthread_mutex_unlock(&mutex_finish_locate);
  if (!finish_location) return pos_temp;
  pthread_mutex_lock(&mutex_pose);
  Position ret = current_posotion_;
  pthread_mutex_lock(&mutex_odom_list);
  for (Position &p : m_OdomList) {
    if (p.mlTimestamp > ret.mlTimestamp && (p.mlTimestamp <= timestamp)) {
      ret = ret * p;
    }
  }
  current_posotion_ = ret;
  while (!m_OdomList.empty()) {
    if (m_OdomList.front().mlTimestamp > timestamp) {
      break;
    }
    m_OdomList.pop_front();
  }
  pthread_mutex_unlock(&mutex_odom_list);
  pthread_mutex_unlock(&mutex_pose);
  return ret;
}

/**
 * @brief
 *
 */
void GridLocation::freeMapDependentMemory() {
  m_MapLoadSuccess = false;
  if (m_pMap != nullptr) {
    map_free(m_pMap);
    m_pMap = nullptr;
  }
  if (m_pPf != nullptr) {
    pf_free(m_pPf);
    m_pPf = nullptr;
  }

  if (m_pOdom != nullptr) {
    delete m_pOdom;
    m_pOdom = nullptr;
  }

  if (m_pLaser != nullptr) {
    delete m_pLaser;
    m_pLaser = nullptr;
  }
}

amcl::map_t *GridLocation::DoMapOpen() {
  amcl::map_t *map = amcl::map_alloc();
  int index_x, index_y;
  int point_count = 0;
  float max_x, max_y;
  float min_x, min_y;
  float threshold = 0.6;
  int width = p_grid_map_->map_info.miMapWidth;
  int height = p_grid_map_->map_info.miMapHeight;
  map->scale = p_grid_map_->map_info.mdResolution;
  map->size_x = width;
  if (map->size_x <= 0) return map;
  map->size_y = height;
  if (map->size_y <= 0) return map;
  float m_scale = map->scale;  // 缩放因子==分辨率
  min_x = p_grid_map_->map_info.mdOriginXInWorld;
  min_y = p_grid_map_->map_info.mdOriginYInWorld;
  map->origin_x = (map->size_x / 2) * m_scale + min_x;
  map->origin_y = (map->size_y / 2) * m_scale + min_y;
  map->cells = reinterpret_cast<amcl::map_cell_t *>(
      malloc(sizeof(amcl::map_cell_t) * map->size_x * map->size_y));

  char grid_value;
  int xi, yi;
  usleep(300000);
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    map->cells[i].occ_state = -1;  // 先全部初始化为n障碍
  }
  int total_height = height - 1;

  for (index_y = 0; index_y < height; index_y++) {
    for (index_x = 0; index_x < width; index_x++) {
      xi = index_x;
      yi = index_y;
      grid_value =
          p_grid_map_
              ->datas[p_grid_map_->get_info().MapIndexToArraryIndex(xi, yi)];
      if (grid_value > 50 && grid_value < 255) {
        map->cells[(index_y)*width + (index_x)].occ_state = 1;
      } else if (grid_value == 255) {
        map->cells[(index_y)*width + (index_x)].occ_state = -1;
      }
    }
  }
  // fclose(map_data_file);
  m_MapLoadSuccess = true;
  SLAM_INFO("地图已加载完毕>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  return map;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool GridLocation::JhInitialPf() {
  SLAM_INFO("成功加载地图，执行粒子初始化.\n");
  ParticleFilterInitialize();
  // 设定激光的安装位置变量
  amcl::pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = m_RadarPosition.mfX;
  laser_pose_v.v[1] = m_RadarPosition.mfY;
  laser_pose_v.v[2] = m_RadarPosition.mfTheta;
  m_pLaser->SetLaserPose(laser_pose_v);
  return true;
}

/**
 * @brief 创建初始化粒子滤波器
 *
 * @return int
 */
int GridLocation::ParticleFilterInitialize() {
  // Create the particle filter
  m_pPf = amcl::pf_alloc(m_MinParticles, m_MaxParticles, m_RecoveryAlphaSlow,
                         m_RecoveryAlphaFast,
                         (amcl::pf_init_model_fn_t)uniformPoseGenerator,
                         reinterpret_cast<void *>(m_pMap));
  m_pPf->pop_err = m_KldErr;
  m_pPf->pop_z = m_KldZ;
  // Initialize the filter
  amcl::pf_vector_t pf_init_pose_mean = amcl::pf_vector_zero();
  pf_init_pose_mean.v[0] = m_InitPoseX;
  pf_init_pose_mean.v[1] = m_InitPoseY;
  pf_init_pose_mean.v[2] = m_InitPoseTheta;
  SLAM_INFO("pf_init_pose_mean = %f %f %f\n", pf_init_pose_mean.v[0],
            pf_init_pose_mean.v[1], pf_init_pose_mean.v[2]);
  usleep(200000);
  SLAM_INFO("PF--The initial pose x = %f,y = %f,theta = %f\n", m_InitPoseX,
            m_InitPoseY, m_InitPoseTheta);
  amcl::pf_matrix_t pf_init_pose_cov = amcl::pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 0.5;
  pf_init_pose_cov.m[1][1] = 0.5;
  pf_init_pose_cov.m[2][2] = 0.15;
  // 如果初始位姿不为0，则进行局部撒粒子，否则全局撒粒子
  if ((fabs(m_InitPoseX) > fuzzy_1) || (fabs(m_InitPoseY) > fuzzy_1)) {
    pf_init(m_pPf, pf_init_pose_mean, pf_init_pose_cov);
    SLAM_INFO("Initializing with uniform distribution\n");
  } else {
    pf_init_model(m_pPf, (amcl::pf_init_model_fn_t)uniformPoseGenerator,
                  reinterpret_cast<void *>(m_pMap));
    SLAM_INFO("Global initialisation done!\n");
  }
  m_PfInit = false;
  // Instantiate the sensor objects
  // Odometry
  delete m_pOdom;
  m_pOdom = nullptr;
  m_pOdom = new amcl::AMCLOdom();
  assert(m_pOdom);
  if (m_OdomType == 0)
    m_pOdom->SetModelDiff(m_OdomAlpha1, m_OdomAlpha2, m_OdomAlpha3,
                          m_OdomAlpha4);
  else
    m_pOdom->SetModelOmni(m_OdomAlpha1, m_OdomAlpha2, m_OdomAlpha3,
                          m_OdomAlpha4, m_OdomAlpha5);
  // Laser
  delete m_pLaser;
  m_pLaser = nullptr;
  m_pLaser = new amcl::AMCLLaser(m_LaserMaxBeams, m_pMap);
  assert(m_pLaser);
  m_pLaser->SetModelLikelihoodField(m_LaserZHit, m_LaserZRand, m_LaserSigmaHit,
                                    m_LaserLikelihoodMaxDist);
  SLAM_INFO("Done initializing likelihood field model.\n");
  SLAM_INFO(
      "粒子滤波器laser_z_hit=%f,z_rand=%f,sigma_hit=%f>>>>>>>>>>>>>>>>>>\n",
      m_LaserZHit, m_LaserZRand, m_LaserSigmaHit);
  SLAM_INFO("粒子滤波,odom,laser模型初始化函数运行完毕>>>>>>>>>>>>>>>>>>>\n");
  return 1;
}

/**
 * @brief 参数初始化，在构造函数中调用
 *
 */
void GridLocation::ParamInitial() {
  // is_ini_locate_ = m_LocationConfig.is_initial_locate;
  m_MinParticles = 500;   // 允许的粒子数量的最小值，默认100(500)
  m_MaxParticles = 3000;  // 允许的例子数量的最大值，默认5000
  m_KldErr = 0.009;  // 真实分布和估计分布之间的最大误差，默认0.01(0.05)
  // 上标准分位数（1-p），其中p是估计分布上误差小于kld_err的概率，默认0.99
  m_KldZ = 0.99;
  m_DThresh = m_LocationConfig.distance_threhold;  // 粒子滤波触发条件1(0.2)
  m_AThresh = m_LocationConfig.angle_threahold;  // 粒子滤波触发条件2(0.5)
  m_ResampleInterval = m_LocationConfig.resample_interval;  // 重采样间隔(2)
  // 慢速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover
  // 默认0（disable），可能0.001是一个不错的值
  m_RecoveryAlphaSlow = 0.001;
  // 快速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover
  // 默认0（disable），可能0.1是个不错的值
  m_RecoveryAlphaFast = 0.1;
  // 存储上一次估计的位姿和协方差到参数服务器的最大速率
  // 被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。(0.5)
  m_SavePoseRate = 2;

  // 激光模型参数
  // 被考虑的最小扫描范围；参数设置为-1.0时，将会使用激光上报的最小扫描范围
  // m_LaserMinRange = m_LocationConfig.laser_min_angle;
  // 被考虑的最大扫描范围；参数设置为-1.0时，将会使用激光上报的最大扫描范围
  // m_LaserMaxRange = m_LocationConfig.laser_max_range;
  // get_configs()->get_float("radar", "laser_max_range", nullptr);
  // 更新滤波器时，每次扫描中多少个等间距的光束被使用
  // 减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响，太小也会造成信息量少定位不准
  m_LaserMaxBeams = m_LocationConfig.laser_max_beams;
  // (0.5)模型的z_hit部分的混合权值，默认0.95
  // 混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值，
  // 其后laser_sigma_hit为标准偏差的高斯分布的权重
  m_LaserZHit = 0.95;
  // (0.05)模型的z_short部分的混合权值
  // 默认0.1（混合权重2.意外对象权重（类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分
  m_LaserZShort = 0.02;
  // 0.05模型的z_max部分的混合权值，默认0.05（混合权重3.测量失败权重（最大距离时为1，其余为0）
  m_LaserZMax = 0.002;
  m_LaserZRand = 0.05;  // 0.5模型的z_rand部分的混合权值
                        // 默认0.05（混合权重4.随机测量权重
  // 均匀分布（1平均分布到0～最大测量范围）
  m_LaserSigmaHit = 0.05;  // 0.2被用在模型的z_hit部分的高斯模型的标准差
                           // 默认0.2m
  m_LaserLambdaShort = 0.5;  // 0.1模型z_short部分的指数衰减参数
                             // 默认0.1（根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
  m_LaserLikelihoodMaxDist = 0.3;  // 2.0地图上做障碍物膨胀的最大距离
                                   // 用作likelihood_field模型
  m_LaserModelType = amcl::LASER_MODEL_LIKELIHOOD_FIELD;  // 可以是beam
                                                          // likehood_field,
                                                          // likehood_field_prob
  m_DoBeamskip = false;                                   // false
  m_BeamSkipDistance = 0.5;                               // 0.5
  m_BeamSkipThreshold = 0.3;                              // 0.3
  // 里程计模型参数
  // odom_model_t odom_model_type;
  // 可以是"diff", "omni", "diff-corrected",
  // "omni-corrected",后面两
  m_OdomAlpha1 = 0.2;  // 0.2指定由机器人运动部分的旋转分量
                       // 估计的里程计旋转的期望噪声,默认0.2
  m_OdomAlpha2 = 0.2;  // 默认0.2
  m_OdomAlpha3 = 0.1;  // 默认0.2
  m_OdomAlpha4 = 0.1;  // 默认0.2
  m_OdomAlpha5 = 0.1;  // 0.1平移相关的噪声参数

  m_ResampleCount = 0;

  m_RadarPosition.mfX = m_LocationConfig.radar_position_x;
  m_RadarPosition.mfY = m_LocationConfig.radar_position_y;
  m_RadarPosition.mfTheta = m_LocationConfig.radar_position_theta;
  int odomtype = m_LocationConfig.odom_type;
  if (odomtype == 0 || odomtype == 1)
    m_OdomType = odomtype;
  else
    m_OdomType = 0;
  m_OnlyRadarPose = m_LocationConfig.only_radar_pose;
  if (m_OnlyRadarPose < 0) m_OnlyRadarPose = 0;
  m_PfInit = false;
  m_LocateStep = 0;
  pthread_mutex_lock(&mutex_finish_locate);
  finish_location_ = false;
  pthread_mutex_unlock(&mutex_finish_locate);
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
