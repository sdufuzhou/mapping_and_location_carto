/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

/**
 * @brief 构造函数
 *
 * @param[in] options
 * @param[in] expected_range_sensor_ids 所有range类型的话题(应该是范围传感器类型)
 */
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),//2d轨迹前端的所有配置
      active_submaps_(options.submaps_options()),//根据子图的相关配置，构建ActiveSubmaps2D对象
      motion_filter_(options_.motion_filter_options()),   //根据运动过滤器的配置，构建 MotionFilter 对象 
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),  //根据real_time_correlative_scan_matcher配置参数，构建
      //scan_matching::RealTimeCorrelativeScanMatcher2D 相关性扫描匹配类对象
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),// 根据ceres_scan_matcher参数，构建scan_matching::CeresScanMatcher2D对象
      range_data_collator_(expected_range_sensor_ids) {}
      //根据订阅的话题，构建RangeDataCollator对象，用于对雷达数据进行时间同步的类

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

/**
 * @brief 先进行点云的旋转与z方向的滤波, 然后再进行体素滤波减少数据量
 *
 * @param[in] transform_to_gravity_aligned_frame
 * 坐标变换：就是后面的gravity_alignment.cast<float>()*
 * range_data_poses.back().inverse()：计算从tracking_frame变换到local_frame原点并且变换后姿态为0的坐标变换
 * @param[in] range_data 传入的点云
 * @return sensor::RangeData 处理后的点云 拷贝
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const 
{ 
  //Step:5将原点位于机器人当前位姿处的点云转成原点位于local坐标系原点处的点云（调用的是TransformRangeData函数）,再进行z轴上的过滤，调用的是CropRangeData函数
  const sensor::RangeData cropped =
      sensor::CropRangeData(/*crop有剪短，裁减之意*/
                            sensor::TransformRangeData(
                            range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(),
                            options_.max_z());  // param: min_z max_z
  // Step: 6 对过滤后的点云进行体素滤波
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns,options_.voxel_filter_size()), // param: voxel_filter_siz
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

/**
 * @brief 进行扫描匹配
 *
 * @param[in] time 点云的时间
 * @param[in] pose_prediction 先验位姿
 * @param[in] filtered_gravity_aligned_point_cloud 匹配用的点云
 * @return std::unique_ptr<transform::Rigid2d> 匹配后的二维位姿
 */

//进行scanMatch,CSM+Ceres（就是理论课程讲的高斯牛顿方法，只不过是基于ceres库实现的）
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
    //当最开始没有地图的时侯，返回的是先验位姿pose_prediction
  }
  // 使用active_submaps_的第一个子图进行匹配
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  // 根据参数决定是否 使用correlative_scan_matching（先进行基于暴力搜索方式的扫描匹配实现）对先验位姿进行校准（建议使用）
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);//把得分加入到度量里
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  // 使用ceres进行扫描匹配（基于优化方式的扫描匹配的实现）
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  // 一些度量
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 返回ceres计算后的位姿
  return pose_observation;
}

/**
 * @brief 处理点云数据, 进行扫描匹配, 将点云写成地图
 *
 * @param[in] sensor_id 点云数据对应的话题名称
 * @param[in] unsynchronized_data 不同步的点云数据
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
 * 匹配后的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  // Step: 1 进行多个雷达点云数据的时间同步,点云的坐标是相对于tracking_frame的
  auto synchronized_data = range_data_collator_.AddRangeData(
      sensor_id, unsynchronized_data); /*传入未同步的点云数据*/
  if (synchronized_data.ranges.empty()) {
    	//通常在多雷达传感器中，expected_sensor_ids_ 数据没有到齐。
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  } 

  //获取已同步的点云数据的时间（正值）
  const common::Time& time =
      synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  // 如果不用imu, 就在雷达这初始化位姿推测器
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO)
        << "Extrapolator not yet initialized.";  //证明还没有完全初始化完毕
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
  /*检查最后一个点的时间是否小于0*/

  // 计算第一个点的时间：第一个激光束的时间戳
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  // 只有在extrapolator_初始化时, GetLastPoseTime()是common::Time::min()
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }


  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(
      synchronized_data.ranges.size());  //为存储坐标变换的容器申请一个空间
  bool warned = false;

  // 预测得到每一个时间点的位姿
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    // 如果该时间比上次预测位姿的时间还要早,说明这个点的时间戳往回走了, 就报错
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      // 一个循环只报一次错
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }

    // Step: 2 预测出每个点的时间戳时刻tracking frame在 local slam坐标系下的位姿

     //计算每一个激光点的位姿
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
        //range_data_poses 表示机器人在 local 坐标系下的位姿
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
    /*空的数据结构的赋值*/
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  //对一帧数据的每个数据点进行处理:这里进行运动畸变去除
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // 获取在tracking frame 下点的坐标
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;

    // 原点的位姿：将点云的origins坐标转到 local slam 坐标系下
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);

    // Step:3运动畸变的去除,将相对于tracking_frame的hit坐标转成相对于local坐标系下的坐标
    //由于雷达自身的运动，还误认为所有点都是相对于最初雷达的位置，导致雷达数据点在转换之后变成了一条斜线，这就叫做运动畸变
    //静止的雷达不会产生运动畸变
    //雷达由于平移产生的运动畸变几乎可以忽略不计，但是由于旋转所产生的运动畸变还是比较明显的，雷达频率越低，所产生的运动畸变也越明显
    //运动畸变去除：
    //求出每个时刻雷达坐标系的位姿与初始位姿的坐标变换，已知某个时刻雷达数据点在运动后的雷达坐标下的位姿，那么根据这个坐标变换，就可以求解出雷达数据点在初始坐标系下的位姿
    //之前是拿运动后的雷达所测量的数据点的坐标，直接误认为是相对于原始坐标系下的坐标，肯定是不对的,就是因为缺少了运动前后雷达坐标系的一个坐标变换
   
   //击中点的位姿
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    // range_data_poses[i]：这个就是雷达坐标系运动前后的坐标变换关系，根据不同时间戳预测出来的

    // 激光束的距离：计算这个点的距离, 这里用的是去畸变之后的点的距离
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    //点的坐标减去原点的坐标，得到的是一个向量
    const float range = delta.norm();  //对这个向量取模长

    // param: min_range max_range
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
        // 在这里可以看到, returns里保存的是local slam下的去畸变之后的点的坐标
      } else {
        // Step: 4 超过max_range时的处理: 用一个距离进行替代, 并放入misses里
        hit_in_local.position =
            origin_in_local +
            // param: missing_data_ray_length, 是个比例, 不是距离
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }  // end for

  // 有一帧有效的数据了
  ++num_accumulated_;  //帧数累加，开始是0

  // param: num_accumulated_range_data 几帧有效的点云数据进行一次扫描匹配
  // accumulated：累加的，累计的
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 计算2帧有效点云数据的的时间差  
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;

    // 重置变量
    num_accumulated_ = 0;  //每次扫描匹配过后，都重置

    // 获取机器人当前的姿态（启动cartographer时的姿态）
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));//位姿推测器通过IMU获得的重力向量
    // Rotation函数的返回值：return Rigid3(Vector::Zero(),rotation)，
    //一个0向量（平移向量），一个用四元数表示的旋转

    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    // note:地图的原点就是这里的origin,不是一个固定值
    // 以点云数据最后一个点的时间戳估计出的坐标变换所包含的平移向量的表示作为这帧点云数据的原点坐标
    accumulated_range_data_.origin = range_data_poses.back().translation();
    return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),  //因为机器人开始姿态的平移向量为0，所以实际上操作的是旋转R
        gravity_alignment, sensor_duration);  // inverse()：T的逆变换
  }
  /*range_data_poses.back().inverse()计算从点云当前原点坐标相对于local坐标系原点的坐标变换*/
  /* gravity_alignment.cast<float>() *range_data_poses.back().inverse()
  代表将当前机器人姿态乘以这个坐标变换，计算从tracking_frame变换到local_frame原点并且变换后姿态为0的坐标变换：
  就是把tracking_frame下所有点的位置给转到相对于local坐标系原点下（这样local坐标系原点的点云就和tracking_frame（运动着的机器人坐标系
  无关了），注意根前面点的坐标变换不同，坐标变换的话，点的位置没有发生变化，还是在原来的坐标系的某个位置处，在tracking_frame的周围
  */
  return nullptr;
}

/**
 * @brief 进行扫描匹配, 将点云写入地图
 *
 * @param[in] time 点云的时间戳
 * @param[in] gravity_aligned_range_data 原点位于local坐标系原点处的点云
 * @param[in] gravity_alignment 机器人当前姿态
 * @param[in] sensor_duration 2帧点云数据的时间差
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) {
  // 如果处理完点云之后数据为空, 就报错. 使用单线雷达时不要设置min_z
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 进行位姿的预测, 先验位姿
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // 将三维位姿（x,y,z,r,p,y）先旋转到姿态为0, 再取xy坐标将三维位姿转成二维位姿(x,y,yaw)
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());//乘以后者转换为二维

  // Step: 7 对 returns点云 进行自适应体素滤波，返回的点云的数据类型是PointCloud
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 扫描匹配, 进行点云与submap的匹配
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);

  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  // 将二维坐标旋转回之前的姿态（三维状态）
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment; // 将二维坐标旋转回之前的姿态
  // 校准位姿估计器
  extrapolator_->AddPose(time, pose_estimate);

  // Step: 8 将 原点位于local坐标系原点处的点云 变换成 原点位于扫描匹配后的位姿处（点云应该在的位置）的点云（最后插入到submap里）
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));

  // 将校正后的雷达数据写入submap
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

  // 计算耗时
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  // 计算cpu耗时
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;

  // 返回结果
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

/**
 * @brief 将处理后的雷达数据写入submap
 *
 * @param[in] time 点云的时间
 * @param[in] range_data_in_local 校正后的点云
 * @param[in] filtered_gravity_aligned_point_cloud 自适应体素滤波后的点云
 * @param[in] pose_estimate 扫描匹配后的三维位姿
 * @param[in] gravity_alignment
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 如果移动距离过小, 或者时间过短, 不进行地图的更新
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 将点云数据写入到submap中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  // 生成InsertionResult格式的数据进行返回
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,  // 这里存的是体素滤波后的点云,
                                                 // 不是校准后的点云
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// 将IMU数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

// 将里程计数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 如果Extrapolator没有初始化就进行初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 如果已经初始化过了就直接返回
  if (extrapolator_ != nullptr) {
    return;
  }

  // 注意 use_imu_based为true就会报错
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.

  // 初始化位姿推测器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(
          options_.pose_extrapolator_options()
              .constant_velocity()
              .pose_queue_duration()),  // 0.001s
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());  // 10
  // 添加初始位姿
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());//(初始位姿：0，0，0，0，0，0)
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
