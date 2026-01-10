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
      pdcsu::util::math::uVec<pdcsu::units::foot_t, 2> initial_position,
      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> initial_vel,
      pdcsu::util::math::uVec<pdcsu::units::fps2_t, 2> initial_accl);

  PoseEstimator() {
    PoseEstimator({pdcsu::units::foot_t{0}, pdcsu::units::foot_t{0}},
        {pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}},
        {pdcsu::units::fps2_t{0}, pdcsu::units::fps2_t{0}});
  };

  std::array<double, 2> position() { return {state_[0], state_[1]}; }

  std::array<double, 2> velocity() { return {state_[2], state_[3]}; }

  void Update(double pVar, double vVar, double aVar);

  void AddAccelerationMeasurement(std::array<double, 2> accl);

  void AddVisionMeasurement(std::array<double, 2> pos, double variance);

  void AddOdometryMeasurement(std::array<double, 2> difPos, double variance);

  void SetPoint(std::array<double, 2> point);

  double getVariance();

  void Zero();

private:
  Eigen::Matrix<double, 6, 1> state_;
  funkit::math::LinearKalmanFilter<6> filter;

  Eigen::Matrix<double, 2, 6> Hv;

  Eigen::Matrix<double, 2, 6> Ha;
  Eigen::Matrix<double, 2, 1> Vara;

  Eigen::Matrix<double, 2, 6> Ho;
  Eigen::Matrix<double, 2, 1> Varo;
};

}  // namespace funkit
