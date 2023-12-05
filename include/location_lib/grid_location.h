/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-26 15:52:31
 * @LastEditTime: 2023-03-31 22:28:26
 * @Author: lcfc-desktop
 */
#pragma once

#include <assert.h>
#include <jsoncpp/json/json.h>
#include <pthread.h>
#include <memory>

#include <list>
#include <memory>
#include <vector>

#include "common_lib/log.h"
#include "include/location_lib/amcl/amcl_laser.h"
#include "include/location_lib/amcl/amcl_odom.h"
#include "include/location_lib/amcl/map.h"
#include "include/location_lib/amcl/pf.h"
#include "include/location_lib/location_interface.h"
#include "include/common/logger.h"
#include "include/config_struct.h"
#include "message_lib/grid_map.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

#define fuzzy_1 0.001
#define fuzzy_mini 0.01

typedef struct {
  // Total weight (weights sum to 1)
  double weight;
  double score;
  // Mean of pose esimate
  amcl::pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  amcl::pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

double normalize(double z);
double angle_diff(double ang_a, double ang_b);
amcl::pf_vector_t uniformPoseGenerator(void* arg);

class GridLocation : public LocationInterface {
 public:
  typedef std::shared_ptr<gomros::message::SimpleGridMap> MapSharedPointer;
  using Position = gomros::message::Position;
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  using Logger = gomros::common::Logger;

  GridLocation(const GridLocationConfig& config);
  ~GridLocation();

  virtual void HandleOdomData(const message::OdometerMessage& data);

  virtual void HandleLaserData(const RadarSensoryMessage& data);

  virtual void SetInitPose(float x, float y, float theta);

  virtual bool StartGlobalLocating(bool relocate);

  virtual void SetConfiguration(void* config);

  virtual void SetMapData(MapSharedPointer grid_map);

  virtual Position ExpolateCurrentPositionTime(uint64_t timestamp);

  virtual Position GetCurrentPosition();

  virtual bool IsFinishLocate();

  virtual void StopLocate();
 

 private:
  static void* GlobalLocatingThreadFunction(void* ptr);

  void freeMapDependentMemory();

  amcl::map_t* DoMapOpen();

  void MainParticleRun();
  bool InitialLocation();
  bool ParticleFilterRun(bool force_update);

  bool DisAmong(amcl::pf_vector_t pos1, amcl::pf_vector_t pos2, double dis);

  void AddOdomDataRaw(const Position& pose_change);

  void AddOdomData(const Position& pose_change);

  void UpdateLastPosition(const Position& pose);

  void UpdateOdomListTimestamp(uint64_t timestamp_us);

  void FlushOdomData();

  void FlushOdomDataRaw();

  Position ExpolateCurrentPosition();

  Position ExpolateCurrentPositionTimeRaw(uint64_t timestamp);
  
  Position DoScanMatching(const RadarSensoryMessage& data, Position pose);

  bool JhInitialPf();

  int ParticleFilterInitialize();

  void ParamInitial();

  Position m_RadarPosition;
  Position m_CurrentPoseRaw;
  Position current_posotion_;
  Position initial_posotion_;
  Position m_RadarPose;  // 实时位姿
  Position m_LocatePose;  // 供外部调用的位姿
  Position m_AmclrPose;
  std::list<Position> m_OdomList;
  std::list<Position> m_OdomListRaw;

  MapSharedPointer p_grid_map_;  // 栅格地图

  float m_InitPoseX;  // 通过外部传递给amcl算法的位姿

  float m_InitPoseY;

  float m_InitPoseTheta;

  int m_LaserRecvCnt = 10;

  bool stop_locate_ = false;

  // 全局定位状态
  bool ini_location_continue_;

  bool finish_location_;

  float m_LocateScore;

  char m_LocateStep;

  bool m_PfInit;

  bool m_MapLoadSuccess;

  int m_OdomType;

  int m_OnlyRadarPose;  // 仅仅使用雷达定位标识
  
  amcl::pf_t* m_pPf = nullptr;  // 粒子滤波器
  amcl::pf_vector_t m_AmclPose;
  amcl::AMCLOdom* m_pOdom = nullptr;
  amcl::AMCLLaser* m_pLaser = nullptr;
  amcl::map_t* m_pMap = nullptr;              // AMCL格式的地图数据
  amcl::pf_vector_t m_LastOdomPose;           // last odom data
  message::RadarSensoryInfo m_LastRadarData;  // last radar data

  int m_RobotStopCnt;
  bool m_DoingScanMatching;
  bool m_relocate_ = false;

  pthread_mutex_t listen_pose_mutex;
  pthread_mutex_t mutex_pose;
  pthread_mutex_t mutex_odom_list;
  pthread_mutex_t mutex_pose_raw;
  pthread_mutex_t mutex_odom_list_raw;
  pthread_mutex_t mutex_finish_locate;


  // 一些关于粒子滤波的参数
  int m_MinParticles;  // 允许的粒子数量的最小值，默认100(500)
  int m_MaxParticles;  // 允许的例子数量的最大值，默认5000
  float m_KldErr;  // 真实分布和估计分布之间的最大误差，默认0.01(0.05)
  float m_KldZ;  // 上标准分位数（1-p），
                 // 其中p是估计分布上误差小于kld_err的概率，默认0.99
  float m_DThresh;            // 粒子滤波触发条件1(0.2)
  float m_AThresh;            // 粒子滤波触发条件2(0.5)
  int m_ResampleInterval;     // 重采样间隔(2)
  float m_RecoveryAlphaSlow;  // 慢速的平均权重滤波的指数衰减频率
                              // 用作决定什么时候通过增加随机位姿来recover
                              // 默认0（disable），可能0.001是一个不错的值
  float m_RecoveryAlphaFast;  // 快速的平均权重滤波的指数衰减频率
                              // 用作决定什么时候通过增加随机位姿来recover
                              // 默认0（disable），可能0.1是个不错的值
  float
      m_SavePoseRate;  // 存储上一次估计的位姿和协方差到参数服务器的最大速率。
                       // 被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。(0.5)

  // 激光模型参数
  float m_LaserMinRange;  // 被考虑的最小扫描范围；参数设置为-1.0时
                          // 将会使用激光上报的最小扫描范围
  float m_LaserMaxRange;  // 被考虑的最大扫描范围；参数设置为-1.0时
                          // 将会使用激光上报的最大扫描范围
  float
      m_LaserMaxBeams;  // 更新滤波器时，每次扫描中多少个等间距的光束被使用
                        // 减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响
  float
      m_LaserZHit;  // (0.5)模型的z_hit部分的混合权值，默认0.95
                    // (混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值
                    // 其后laser_sigma_hit为标准偏差的高斯分布的权重)
  float
      m_LaserZShort;  // (0.05)模型的z_short部分的混合权值，默认0.1
                      // （混合权重2.意外对象权重
                      // 类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分：
  float m_LaserZMax;  // 0.05模型的z_max部分的混合权值，默认0.05
                      // （混合权重3.测量失败权重（最大距离时为1，其余为0）
  float
      m_LaserZRand;  // 0.5模型的z_rand部分的混合权值，默认0.05
                     // （混合权重4.随机测量权重--均匀分布（1平均分布到0～最大测量范围）
  float m_LaserSigmaHit;  // 0.2被用在模型的z_hit部分的高斯模型的标准差
                          // 默认0.2m
  float
      m_LaserLambdaShort;  // 0.1模型z_short部分的指数衰减参数，默认0.1
                           // （根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
  float m_LaserLikelihoodMaxDist;  // 2.0地图上做障碍物膨胀的最大距离
                                   // 用作likelihood_field模型
  int m_LaserModelType;  // 可以是beam, likehood_field, likehood_field_prob
  bool m_DoBeamskip;     // false
  float m_BeamSkipDistance;   // 0.5
  float m_BeamSkipThreshold;  // 0.3

  // 里程计模型参数
  amcl::odom_model_t
      m_OdomModelType;  // ODOM_MODEL_DIFF//diff模型使用，可以是"diff", "omni",
                        // "diff-corrected", "omni-corrected",后面两
  float m_OdomAlpha1;  // 0.2指定由机器人运动部分的旋转分量
                       // 估计的里程计旋转的期望噪声，默认0.2
  float m_OdomAlpha2;  // 0.2制定由机器人运动部分的平移分量
                       // 估计的里程计旋转的期望噪声，默认0.2
  float m_OdomAlpha3;  // 0.8指定由机器人运动部分的平移分量
                       // 估计的里程计平移的期望噪声，默认0.2
  float m_OdomAlpha4;  // 0.2指定由机器人运动部分的旋转分量
                       // 估计的里程计平移的期望噪声，默认0.2
  float m_OdomAlpha5;  // 0.1平移相关的噪声参数

  int m_ResampleCount;  // 重采样计数
  float laser_min_angle_;
  float laser_max_angle_;
  float laser_resolution_;
  float laser_max_range_;
  float laser_min_range_;
  float angle_increment_;
  int stepIncreament_;
  bool first_laser_ = true;

  pthread_t m_GlobalLocatingThread;

  GridLocationConfig m_LocationConfig;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
