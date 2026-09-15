#pragma once

#include <frc/EigenCore.h>

#include <array>

#include "funkit/math/collection.h"
#include "funkit/math/filter.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::odometry {

class PoseEstimator {
public:
  PoseEstimator(
      pdcsu::util::math::uVec<pdcsu::units::inch_t, 2> initial_position,
      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> initial_vel);

  PoseEstimator()
      : PoseEstimator({pdcsu::units::inch_t{0}, pdcsu::units::inch_t{0}},
            {pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}}) {}

  std::array<double, 2> position() { return {state_[0], state_[1]}; }

  std::array<double, 2> velocity_ips() { return {state_[2], state_[3]}; }

  void Update();

  void SetProcessNoise(double pose_process_var, double velocity_process_var);

  void AddVisionMeasurement(std::array<double, 2> pos, double variance);

  void AddOdometryMeasurement(std::array<double, 2> difPos, double variance);

  void SetPoint(std::array<double, 2> point);

  double getVariance();

  void Zero();

private:
  static constexpr double kDefaultPoseProcessVar = 1e-4;
  static constexpr double kDefaultVelocityProcessVar = 0.03;

  Eigen::Matrix<double, 4, 1> state_;
  funkit::math::LinearKalmanFilter<4> filter;

  Eigen::Matrix<double, 2, 4> Ho;
  double pose_process_var_ = kDefaultPoseProcessVar;
  double velocity_process_var_ = kDefaultVelocityProcessVar;
};

}  // namespace funkit::robot::swerve::odometry
