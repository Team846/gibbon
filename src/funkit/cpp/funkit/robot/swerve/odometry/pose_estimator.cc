#include "funkit/robot/swerve/odometry/pose_estimator.h"

#include <frc/DriverStation.h>

#include "funkit/robot/GenericRobot.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve::odometry {

PoseEstimator::PoseEstimator(
    pdcsu::util::math::uVec<pdcsu::units::foot_t, 2> initial_position,
    pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> initial_vel,
    pdcsu::util::math::uVec<pdcsu::units::fps2_t, 2> initial_accl) {
  state_ = {initial_position[0].value(), initial_position[1].value(),
      initial_vel[0].value(), initial_vel[1].value(), initial_accl[0].value(),
      initial_accl[1].value()};
  double dt =
      pdcsu::units::second_t{funkit::robot::GenericRobot::kPeriod}.value();
  filter = funkit::math::LinearKalmanFilter<6>(state_,
      Eigen::Matrix<double, 6, 6>({{1, 0, dt, 0, dt * dt / 2, 0},
          {0, 1, 0, dt, 0, dt * dt / 2}, {0, 0, 1, 0, dt, 0},
          {0, 0, 0, 1, 0, dt}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}}));
  Hv = Eigen::Matrix<double, 2, 6>({{1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}});

  Ha = Eigen::Matrix<double, 2, 6>({{0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}});
  Vara = Eigen::Matrix<double, 2, 1>({{1.0}, {1.0}});

  Ho = Eigen::Matrix<double, 2, 6>({{0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}});
  Varo = Eigen::Matrix<double, 2, 1>({{1.0}, {1.0}});
}

void PoseEstimator::Update(double pVar, double vVar, double aVar) {
  filter.Predict(Eigen::Matrix<double, 6, 1>(
      {{pVar}, {pVar}, {vVar}, {vVar}, {aVar},
          {aVar}}).asDiagonal());
  state_ = filter.getEstimate();
}

void PoseEstimator::AddVisionMeasurement(
    std::array<double, 2> pos, double variance) {
  filter.Update(Hv, Eigen::Matrix<double, 2, 1>({{pos[0]}, {pos[1]}}),
      Eigen::Matrix<double, 2, 1>({{variance, variance}}));
  state_ = filter.getEstimate();
}

void PoseEstimator::AddAccelerationMeasurement(std::array<double, 2> accl) {
  filter.Update(Ha, Eigen::Matrix<double, 2, 1>({{accl[0]}, {accl[1]}}), Vara);
  state_ = filter.getEstimate();
}

void PoseEstimator::AddOdometryMeasurement(
    std::array<double, 2> difPos, double variance) {
  double dt =
      pdcsu::units::second_t{funkit::robot::GenericRobot::kPeriod}.value();
  filter.Update(Ho,
      Eigen::Matrix<double, 2, 1>({{difPos[0] / dt}, {difPos[1] / dt}}),
      Eigen::Matrix<double, 2, 1>({{variance, variance}}));
  state_ = filter.getEstimate();
}

void PoseEstimator::SetPoint(std::array<double, 2> point) {
  state_ =
      Eigen::Matrix<double, 6, 1>({{point[0]}, {point[1]}, {state_.coeff(2, 0)},
          {state_.coeff(3, 0)}, {state_.coeff(4, 0)}, {state_.coeff(5, 0)}});
  filter.setSureEstimate(state_);
}

void PoseEstimator::Zero() { SetPoint({0.0, 0.0}); }

double PoseEstimator::getVariance() {
  Eigen::Matrix<double, 6, 6> cov = filter.getCoVar();
  return (cov.coeff(0, 0) + cov.coeff(1, 1)) / 2;
}

}  // namespace funkit::robot::swerve::odometry