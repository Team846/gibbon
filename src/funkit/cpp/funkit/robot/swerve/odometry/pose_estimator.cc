#include "funkit/robot/swerve/odometry/pose_estimator.h"

#include <algorithm>

#include "funkit/robot/GenericRobot.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve::odometry {

PoseEstimator::PoseEstimator(
    pdcsu::util::math::uVec<pdcsu::units::foot_t, 2> initial_position,
    pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> initial_vel) {
  state_ = Eigen::Matrix<double, 5, 1>({{initial_position[0].value()},
      {initial_position[1].value()}, {initial_vel[0].value()},
      {initial_vel[1].value()}, {kLatencyInitialSec}});
  double dt =
      pdcsu::units::second_t{funkit::robot::GenericRobot::kPeriod}.value();
  filter = funkit::math::LinearKalmanFilter<5>(
      state_, Eigen::Matrix<double, 5, 5>({{1, 0, dt, 0, 0}, {0, 1, 0, dt, 0},
                  {0, 0, 1, 0, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 0, 1}}));

  Ho = Eigen::Matrix<double, 2, 5>({{0, 0, 1, 0, 0}, {0, 0, 0, 1, 0}});
  Varo = Eigen::Matrix<double, 2, 1>({{1.0}, {1.0}});
}

void PoseEstimator::ClampLatency() {
  state_(4, 0) = std::clamp(state_(4, 0), kLatencyMinSec, kLatencyMaxSec);
}

void PoseEstimator::Update() {
  filter.Predict(Eigen::Matrix<double, 5, 1>(
      {{kPoseProcessVar}, {kPoseProcessVar}, {kVelocityProcessVar},
          {kVelocityProcessVar}, {kLatencyProcessVar}})
                     .asDiagonal());
  state_ = filter.getEstimate();
  ClampLatency();
  filter.setState(state_);
}

void PoseEstimator::AddVisionMeasurement(
    std::array<double, 2> pos, double variance) {
  double L = state_(4, 0);
  double vx = state_(2, 0);
  double vy = state_(3, 0);
  Eigen::Matrix<double, 2, 5> Hv;
  Hv << 1, 0, -L, 0, -vx, 0, 1, 0, -L, -vy;
  Eigen::Matrix<double, 2, 1> z_obs;
  z_obs << pos[0], pos[1];
  filter.Update(
      Hv, z_obs, Eigen::Matrix<double, 2, 1>({{variance}, {variance}}));
  state_ = filter.getEstimate();
  ClampLatency();
  filter.setState(state_);
}

void PoseEstimator::AddOdometryMeasurement(
    std::array<double, 2> difPos, double variance) {
  double dt =
      pdcsu::units::second_t{funkit::robot::GenericRobot::kPeriod}.value();
  filter.Update(Ho,
      Eigen::Matrix<double, 2, 1>({{difPos[0] / dt}, {difPos[1] / dt}}),
      Eigen::Matrix<double, 2, 1>({{variance}, {variance}}));
  state_ = filter.getEstimate();
}

void PoseEstimator::SetPoint(std::array<double, 2> point) {
  state_ = Eigen::Matrix<double, 5, 1>({{point[0]}, {point[1]},
      {state_.coeff(2, 0)}, {state_.coeff(3, 0)}, {state_.coeff(4, 0)}});
  filter.setSureEstimate(state_);
}

void PoseEstimator::Zero() { SetPoint({0.0, 0.0}); }

double PoseEstimator::getVariance() {
  Eigen::Matrix<double, 5, 5> cov = filter.getCoVar();
  return (cov.coeff(0, 0) + cov.coeff(1, 1)) / 2;
}

double PoseEstimator::getLatency() { return state_(4, 0); }

}  // namespace funkit::robot::swerve::odometry
