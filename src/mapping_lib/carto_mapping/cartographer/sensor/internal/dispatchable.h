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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
 public:
 //构造函数，同时会调用父类的构造函数。对参数data赋值给成员变量data_
  Dispatchable(const std::string& sensor_id, const DataType& data)
      : Data(sensor_id), data_(data) {}
  //重写父类函数，直接返回 data_.time 即可(表示DataType类型的数据,必须含有成员变量time)
  common::Time GetTime() const override { return data_.time; }

  // 调用传入的trajectory_builder的AddSensorData()，这里的trajectory_builder为GlobalTrajectoryBuilder类对象指针
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface* const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }
  //返回成员变量data_的一个引用
  const DataType& data() const { return data_; }

 private:
  const DataType data_;//构造函数初始化列表进行赋值，之后便不可再进行更改
};

// c++11: template <typename DataType>
// 函数模板的调用使用 实参推演 来进行
// 类模板 模板形参的类型必须在类名后的尖括号中明确指定, 不能使用实参推演
// 在类外声明一个 函数模板, 使用 实参推演 的方式来使得
// 类模板可以自动适应不同的数据类型

// 根据传入的data的数据类型,自动推断DataType,
// 实现一个函数处理不同类型的传感器数据
template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string& sensor_id, const DataType& data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}
//那么为什么要在 Dispatchable 的外面写这样一个函数呢？而不把他写在 class Dispatchable 之中呢？
//主要时因为类成员模板函数不具备自动推导模板参数的能力，而非类模板成员函数即普通函数是具备这个能力的

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
