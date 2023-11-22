/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// 时间同步前的点云
struct TimedPointCloudData {
  common::Time time;        // 点云最后一个点的时间
  Eigen::Vector3f origin;   // tracking_frame_到雷达坐标系的坐标变换为原点（坐标变换T转换为（x,y,theta））
  TimedPointCloud ranges;   // 数据点的集合, 每个数据点包含xyz与time, time是负的（ 将点云从雷达坐标系下转到tracking_frame坐标系下后的点云）
  // 'intensities' has to be same size as 'ranges', or empty.
  std::vector<float> intensities; // 空的
};

// 时间同步后的点云
struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;   // 带时间戳的单个数据点的坐标 xyz
    float intensity;                    // 强度值
    size_t origin_index;                // 属于第几个origins的点
  };
  common::Time time;                    // 点云的时间
  // ranges中点云涉及哪些坐标系，origins存储着这些坐标系原点在tracking_frame_坐标系下的坐标。
  // 针对单一、有序场景，传感器只涉及一个坐标系，像“laser”，所以origins.size()总是1，值是TimedPointCloudData.origin。
  std::vector<Eigen::Vector3f> origins; // 点云原点坐标的集合，
  std::vector<RangeMeasurement> ranges; // 数据点的集合
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
