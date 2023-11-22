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

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/strings/substitute.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace transform {

template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }

  /* c++11: .template 调用模板类的模板成员函数前必须加template关键字的情况
    当要使用模板类中的模板函数时, 如果同时满足下面两个条件:
      1.如果模板类的模板参数是不确定类型时（int和非模板类等是确定类型）
      2.显式提供模板函数的模板参数, 不管模板参数是确定类型还是不确定类型
    则，需要在模板函数前加template关键字
   */
  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());
  }

  // T = [R t] T^-1 = [R^-1  -R^-1 * t]
  //     [0 1]        [0         1    ] 
  // R是旋转矩阵, 特殊正交群, 所以R^-1 = R^T
  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();//只有四元数表示的旋转才存在求共轭一说
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  std::string DebugString() const {
    return absl::Substitute("{ t: [$0, $1], r: [$2] }", translation().x(),
                            translation().y(), rotation().angle());
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

// lhs是全局坐标系下的位姿, rhs是全局坐标系下的坐姿变动量
// lhs.rotation() * rhs.translation() + lhs.translation() 的意思是
// 将 rhs 转换成 lhs自身坐标系下的位姿变动量 再与lhs的坐标相加
// 得到 lhs 在全局坐标系下的新的位姿
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>; //使用了Eigen库中的Matrix类来定义一个Vector别名，使其代表一个3x1的矩阵，其中数据类型为FloatType
  using Quaternion = Eigen::Quaternion<FloatType>;//用Quaternion代替表示Eigen中的四元数
  using AngleAxis = Eigen::AngleAxis<FloatType>;//用AngleAxis代替表示Eigen中的轴角

  //默认构造函数,对平移translation_与旋转rotation_两个变量通过初始化列表进行初始化,全为0
  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  //构造函数重载,传入一个向量表示的平移translation, 与四元数表示的旋转进行初始化
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  //构造函数重载,传入一个向量表示的平移translation, 与轴角表示的旋转
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  //声明该为静态函数，该函数可以通过Rigid3::Rotation()直接进行调用，
  //而非必须创建实例之后才能调用,理解为python中的类函数,注意其没有this指针
  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));//最终都是以四元数的格式存储的
    //函数内部通过调用另一个构造函数来创建一个由平移向量和四元数构成的Rigid3对象
    //整个函数的作用是将旋转矩阵和平移向量合成为一个Rigid3变换矩阵
  }

 //该为重载函数，作用与上一函数一样，就是根据传入的参数创建一个Rigid3实例返回,
  //该实例平移初始值都为0, 旋转使用传入的参数进行表示
  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }
  //根据传入的参数创建一个Rigid3实例返回,
  //该实例平移为传入的vector,旋转初始化全为0
  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }
 //根据以数组形式传入的四元数旋转rotation，以及平移translation构建一个实例
  static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                           const std::array<FloatType, 3>& translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                               rotation[2], rotation[3]));
  }
  //创建一个初始化全为0的Rigid3实例
  static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

   //该函数主要实现数据的类型转换,把原来的数据类型转化为OtherType
  template <typename OtherType> 
  Rigid3<OtherType> cast() const {
    //.template的用法比较简单，因为cast<OtherType>() 为 Eigen::Matrix 实例对象的
    //模板函数，所以使用.template声明，告诉编译器,接下来要调用的是一个类中实现的模板函数。
    //如果直接调用 translation_.cast<OtherType>() 会报错如下：
    //error: expected primary-expression before ‘>’ token，
    //简单的说就是编译器弄不清楚translation_.cast后面'<'是解析成模板还是解析成小于符号
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  //const修饰返回值，表示返回值不能被修改，只能赋值给其他变量
  //const修饰函数体，或者花括号，表示函数体或者花括号中，都是常量操作，
  //且其中只能调用使用const修饰的函数。另外这里返回的变量为引用类型
  const Vector& translation() const { return translation_; }//该函数的作用是返回当前 Rigid3 对象的平移向量
  const Quaternion& rotation() const { return rotation_; }

  // T = [R t] T^-1 = [R^-1  -R^-1 * t]
  //     [0 1]        [0         1    ] 
  // R是旋转矩阵, 特殊正交群, 所以R^-1 = R^T
  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();//共轭,等价于旋转矩阵求逆
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);//返回欧式变换群的逆
  }

  std::string DebugString() const {//absl::Substitute 是一个高效的字符串替换函数，用于调试信息的打印
    return absl::Substitute("{ t: [$0, $1, $2], q: [$3, $4, $5, $6] }",
                            translation().x(), translation().y(),
                            translation().z(), rotation().w(), rotation().x(),
                            rotation().y(), rotation().z());
  }

//检测这些数据是否有效，如平移的xyz不能为nan，四元数各个元素平方和为1
  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
           /*这里的norm是四元数的模，即四元数的长度，通常表示为|q|，计算公式为：
          |q| = sqrt(qw^2 + qx^2 + qy^2 + qz^2)
          其中，qw、qx、qy、qz分别为四元数的标量部分和三个虚部分。
          因为旋转部分必须是一个单位四元数，即|q|=1，所以这里用1与四元数norm的差值来判断是否接近于1
          */
  }

 private:
  Vector translation_; //平移私有成员变量
  Quaternion rotation_;//旋转私有成员变量
};

// lhs是全局坐标系下的位姿, rhs是全局坐标系下的位姿变动量
// lhs.rotation() * rhs.translation() + lhs.translation() 的意思是
// 将 rhs 转换成 lhs自身坐标系下的位姿变动量 再与lhs的坐标相加
// 得到 lhs 在全局坐标系下的新的位姿，

//实现模板类Rigid3的 '*' 操作，该操作为两个 Rigid3 实例进行 '*' 运算
template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());//参数上,全都是平移在前，旋转在后
}

//该函数的功能为对一个3维点进行欧式变换
//p_new = R*p + t 
template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.  //实现cout打印与输出功能
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.

//把欧拉角转换成四元数
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
