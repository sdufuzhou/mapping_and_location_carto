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

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {

/**
 * @brief 添加轨迹以生成排序的传感器输出, 每个topic设置一个回调函数
 *
 * @param[in] trajectory_id 新生成的轨迹的id
 * @param[in] expected_sensor_ids 需要排序的topic名字的集合
 * @param[in] callback 2个参数的回调函数,实际是CollatedTrajectoryBuilder::HandleCollatedSensorData()函数
 */
void Collator::AddTrajectory(
    const int trajectory_id,//轨迹id
    const absl::flat_hash_set<std::string>& expected_sensor_ids,//订阅的话题名字集合
    const Callback& callback) {
  /*对所有传感器的topic进行遍历，每一个topic生成一个queue_key*/
  for (const auto& sensor_id : expected_sensor_ids) {
    //为每个话题构建一个QueueKey，其包含了trajectory_id与订阅该话题名字
    const auto queue_key = QueueKey{trajectory_id, sensor_id};//键
    //为每条轨迹的话题都添加一一对应的数据队列
    queue_.AddQueue(queue_key,//把trajectory_id与话题名字共同看作key
                    // void(std::unique_ptr<Data> data) 带了个默认参数sensor_id
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });//值
    //把同一trajectory_id的queue_key都存放在一个vector中
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

// 将 trajectory_id 标记为完成
void Collator::FinishTrajectory(const int trajectory_id) {
  //传入一个trajectory_id，然后对该id下的std::vector<QueueKey>进行遍历
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

// 向数据队列中添加 传感器数据
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}
// 将所有数据队列标记为已完成,分派所有剩下的传感器数据
// 只能调用一次, 在 Flush 之后不能再调用 AddSensorData()
void Collator::Flush() { queue_.Flush(); }

// 返回在 CollatorInterface 解锁之前需要更多数据的轨迹的 ID
// 对于不等待特定轨迹的实现, 返回 'nullopt'
absl::optional<int> Collator::GetBlockingTrajectoryId() const {
  return absl::optional<int>(queue_.GetBlocker().trajectory_id);
}
/*
这段代码中，有两个函数的定义：`Flush()` 和 `GetBlockingTrajectoryId()`。

`Flush()` 函数的作用是将传感器数据队列中的数据全部清空。这里的 `queue_` 是一个传感器数据队列，通过调用 `Flush()` 函数可以清空队列中的数据，以便进行下一轮的数据处理。

`GetBlockingTrajectoryId()` 函数返回一个 `absl::optional<int>` 类型的值，表示可能存在的阻塞轨迹的 ID。

  在这里，它通过访问 `queue_` 中的 `Blocker` 对象获取阻塞轨迹的 ID，并将其封装到 `absl::optional` 类型中返回。如果没有阻塞轨迹，则返回的 `absl::optional` 对象为空。

综合起来，这两个函数的作用是对传感器数据队列进行操作，其中 `Flush()` 清空队列，而 `GetBlockingTrajectoryId()` 返回可能存在的阻塞轨迹的 ID。
*/

}  // namespace sensor
}  // namespace cartographer
