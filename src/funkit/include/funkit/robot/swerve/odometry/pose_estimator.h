#pragma once

#include <frc/EigenCore.h>

#include <array>

#include "funkit/math/collection.h"
#include "funkit/math/filter.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::odometry {

/**
 * PoseEstimator
 *
 * A class that applies linear algebraic math and kalman filters to estimate
 * robot pose.
 */
class PoseEstimator {
public:
  PoseEstimator(
      pdcsu::util::math::uVec<pdcsu::units::foot_t, 2> initial_position,
      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> initial_vel);

  PoseEstimator() {
    PoseEstimator({pdcsu::units::foot_t{0}, pdcsu::units::foot_t{0}},
        {pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}});
  };

  // Getter method for position determined by PoseEstimator
  std::array<double, 2> position() { return {state_[0], state_[1]}; }

  // Getter method for velocity determined by PoseEstimator
  std::array<double, 2> velocity() { return {state_[2], state_[3]}; }

  // Updates/predicts the state of a dynamic system
  void Update();

  /**
   * AddVisionMeasurement()
   *
   * Combines a state measured from a vision system into the state estimate
   * @param pos - the position determined from a vision measurement
   * @param variance - the variance/uncertainty of the vision measurement
   */
  void AddVisionMeasurement(std::array<double, 2> pos, double variance);

  /**
   * AddOdometryMeasurement()
   *
   * Combines an odometry derived velocity measurement into the state estimate
   * @param difPos - the difference in position determined from the odometry
   * measurement
   * @param variance - the variance/uncertainty of the odometry measurement
   */
  void AddOdometryMeasurement(std::array<double, 2> difPos, double variance);

  // Sets a trusted estimate of the state with provided position.
  // Velocity/latency stay the same.
  void SetPoint(std::array<double, 2> point);

  // Returns the average variance of position
  double getVariance();

  // Returns the latency of the state in seconds
  double getLatency();

  // Zeros the position. Velocity/latency stay the same.
  void Zero();

private:
  static constexpr double kLatencyMinSec = 0.0;
  static constexpr double kLatencyMaxSec = 0.0;
  static constexpr double kLatencyInitialSec = 0.0;
  static constexpr double kLatencyProcessVar = 1e-7;
  static constexpr double kPoseProcessVar = 1e-4;
  static constexpr double kVelocityProcessVar = 0.03;
  static constexpr double kMinSpeedForLatencyObsFps = 0.5;
  static constexpr double kMaxLatencyStepSec = 0.008;

  void ClampLatency();

  Eigen::Matrix<double, 5, 1> state_;
  funkit::math::LinearKalmanFilter<5> filter;

  Eigen::Matrix<double, 2, 5> Ho;
  Eigen::Matrix<double, 2, 1> Varo;
};

}  // namespace funkit::robot::swerve::odometry
