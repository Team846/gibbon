#include "funkit/robot/swerve/odometry/pose_estimator.h"

#include "funkit/robot/GenericRobot.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve::odometry {

PoseEstimator::PoseEstimator(
    pdcsu::util::math::uVec<pdcsu::units::inch_t, 2> initial_position,
    pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> initial_vel) {
  state_ = Eigen::Matrix<double, 4, 1>(
      {{initial_position[0].value()}, {initial_position[1].value()},
          {initial_vel[0].value() * 12.0}, {initial_vel[1].value() * 12.0}});
  double dt =
      pdcsu::units::second_t{funkit::robot::GenericRobot::kPeriod}.value();
  filter = funkit::math::LinearKalmanFilter<4>(
      state_, Eigen::Matrix<double, 4, 4>(
                  {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}}));

  Ho = Eigen::Matrix<double, 2, 4>({{0, 0, 1, 0}, {0, 0, 0, 1}});
}

void PoseEstimator::SetProcessNoise(
    double pose_process_var, double velocity_process_var) {
  pose_process_var_ = pose_process_var;
  velocity_process_var_ = velocity_process_var;
}

void PoseEstimator::Update() {
  filter.Predict(Eigen::Matrix<double, 4, 1>(
      {{pose_process_var_}, {pose_process_var_}, {velocity_process_var_},
          {velocity_process_var_}})
          .asDiagonal());
  state_ = filter.getEstimate();
}

void PoseEstimator::AddVisionMeasurement(
    std::array<double, 2> pos, double variance) {
  Eigen::Matrix<double, 2, 4> Hv;
  Hv << 1, 0, 0, 0, 0, 1, 0, 0;
  Eigen::Matrix<double, 2, 1> z_obs;
  z_obs << pos[0], pos[1];
  filter.Update(
      Hv, z_obs, Eigen::Matrix<double, 2, 1>({{variance}, {variance}}));
  state_ = filter.getEstimate();
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
  state_ = Eigen::Matrix<double, 4, 1>(
      {{point[0]}, {point[1]}, {state_.coeff(2, 0)}, {state_.coeff(3, 0)}});
  filter.setSureEstimate(state_);
}

void PoseEstimator::Zero() { SetPoint({0.0, 0.0}); }

double PoseEstimator::getVariance() {
  Eigen::Matrix<double, 4, 4> cov = filter.getCoVar();
  return (cov.coeff(0, 0) + cov.coeff(1, 1)) / 2;
}

}  // namespace funkit::robot::swerve::odometry
