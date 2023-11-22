/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-26 19:31:23
 * @LastEditTime: 2022-10-01 16:17:43
 * @Author: lcfc-desktop
 */
#include "include/location_lib/scan_matching/scan_matching.h"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <stdio.h>

#include <Eigen/Core>
#include <cmath>
#include <iostream>

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(double scaling_factor,
                              std::vector<Eigen::Vector2d> point_cloud,
                              GlobalCoordinateMap* grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {
    info = grid_->get_info();
  }
  ~OccupiedSpaceCostFunction2D() {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);
    int size = point_cloud_.size();
    GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);

    for (int i = 0; i < size; i++) {
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                         (T(point_cloud_[i].y())), T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (world[0] - info.mdOriginXInWorld) / info.mdResolution,
          (world[1] - info.mdOriginXInWorld) / info.mdResolution, &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  double scaling_factor_;
  std::vector<Eigen::Vector2d> point_cloud_;
  GlobalCoordinateMap* grid_;
  MapInfo info;
};

class TranslationDeltaCostFunctor2D {
 public:
  TranslationDeltaCostFunctor2D(double scaling_factor, double x, double y)
      : scaling_factor_(scaling_factor), x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
};

class RotationDeltaCostFunctor2D {
 public:
  RotationDeltaCostFunctor2D(double scaling_factor, double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

Position ScanMatching(const Position& init, GlobalCoordinateMap* map,
                      const message::RadarSensoryMessage& range_data,
                      const ScanMatchingOptions& option) {
  double opt_buf[3];
  opt_buf[0] = init.mfX;
  opt_buf[1] = init.mfY;
  opt_buf[2] = init.mfTheta;

  ceres::Solver::Options options;
  options.max_num_iterations = option.max_num_iterations;
  options.num_threads = option.num_threads;
  options.use_nonmonotonic_steps = option.use_nonmonotonic_steps;
  ceres::Problem problem;

  ceres::CostFunction* occupied_function =
      new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                      ceres::DYNAMIC /* residuals */,
                                      3 /* pose variables */>(
          new OccupiedSpaceCostFunction2D(
              option.occupied_space_cost_factor,
              range_data.mstruRadarMessage.mstruSingleLayerData.mvPoints, map),
          range_data.mstruRadarMessage.mstruSingleLayerData.mvPoints.size());
  problem.AddResidualBlock(occupied_function, nullptr /* loss function */,
                           opt_buf);

  ceres::CostFunction* trans_delta_function = new ceres::AutoDiffCostFunction<
      TranslationDeltaCostFunctor2D, 2 /* residuals */, 3 /* pose variables */>(
      new TranslationDeltaCostFunctor2D(option.translation_delta_cost_factor,
                                        init.mfX, init.mfY));
  problem.AddResidualBlock(trans_delta_function, nullptr /* loss function */,
                           opt_buf);

  ceres::CostFunction* rotation_delta_function =
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor2D,
                                      1 /* residuals */,
                                      3 /* pose variables */>(
          new RotationDeltaCostFunctor2D(option.rotation_delta_cost_factor,
                                         init.mfTheta));
  problem.AddResidualBlock(rotation_delta_function, nullptr /* loss function */,
                           opt_buf);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << "\n";
  // printf("From (%.3f, %.3f, %.3f)\n", init.x, init.y, init.theta);
  // printf("To   (%.3f, %.3f, %.3f)\n", opt_buf[0], opt_buf[1], opt_buf[2]);

  return Position(init.mlTimestamp, opt_buf[0], opt_buf[1], opt_buf[2]);
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
