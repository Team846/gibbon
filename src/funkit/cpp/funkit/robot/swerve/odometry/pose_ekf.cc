#include "funkit/robot/swerve/odometry/pose_ekf.h"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>

namespace funkit::robot::swerve::odometry {

PoseEKF::PoseEKF() : x_{Eigen::Vector3d::Zero()} {
  P_ = Eigen::Vector3d{kUnknownPositionVar, kUnknownPositionVar,
      kResetHeadingVar}
           .asDiagonal();
}

void PoseEKF::Predict(
    Vector2D delta, double position_var, double heading_var_deg2) {
  const double dx = delta[0].value();
  const double dy = delta[1].value();
  x_(0) += dx;
  x_(1) += dy;
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = dy;
  F(1, 2) = -dx;
  P_ = F * P_ * F.transpose();
  P_.diagonal() += Eigen::Vector3d{
      position_var, position_var, heading_var_deg2 * kDeg2ToRad2};
}

bool PoseEKF::Correct(const funkit::robot::calculators::TagObservation& obs,
    const PoseEKFTagParams& params) {
  const double theta =
      pdcsu::units::radian_t{obs.gyro_bearing_at_capture}.value() + x_(2);
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  const double cx = obs.cam_offset[0].value();
  const double cy = obs.cam_offset[1].value();
  const double cfx = cx * c + cy * s;
  const double cfy = -cx * s + cy * c;
  const double dx = obs.tag_pos[0].value() -
                    (x_(0) - obs.odom_since_capture[0].value()) - cfx;
  const double dy = obs.tag_pos[1].value() -
                    (x_(1) - obs.odom_since_capture[1].value()) - cfy;
  const double r2 = dx * dx + dy * dy;
  if (r2 < 1.0) { return false; }
  const double r = std::sqrt(r2);

  const double tx_pred = std::atan2(dx, dy) - theta -
                         pdcsu::units::radian_t{obs.yaw_offset}.value();
  const Eigen::Vector2d nu{obs.distance.value() - r,
      std::remainder(
          pdcsu::units::radian_t{obs.tx}.value() - tx_pred, 2.0 * kPi)};

  Eigen::Matrix<double, 2, 3> H;
  H << -dx / r, -dy / r, (dy * cfx - dx * cfy) / r, -dy / r2, dx / r2,
      -(dx * cfx + dy * cfy) / r2 - 1.0;

  const double meas_r = std::max(obs.distance.value(), 1.0);
  const Eigen::Matrix2d R =
      Eigen::Vector2d{obs.variance * params.range_var_scale,
          obs.variance * params.bearing_var_scale / (meas_r * meas_r)}
          .asDiagonal();

  Eigen::Matrix2d S = H * P_ * H.transpose() + R;
  if (nu.dot(S.inverse() * nu) > params.gate_chi2) {
    if (++rejected_streak_ < params.gate_release_count) { return false; }
    const double err2 = nu(0) * nu(0) + meas_r * meas_r * nu(1) * nu(1);
    P_.diagonal() += Eigen::Vector3d{err2, err2, kReleaseHeadingVar};
    S = H * P_ * H.transpose() + R;
    rejected_streak_ = 0;
  }
  rejected_streak_ = std::max(rejected_streak_ - 1, 0);

  const Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
  x_ += K * nu;
  const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  return true;
}

void PoseEKF::SetPosition(Vector2D position) {
  x_(0) = position[0].value();
  x_(1) = position[1].value();
  P_.topRows<2>().setZero();
  P_.leftCols<2>().setZero();
  P_(0, 0) = kResetPositionVar;
  P_(1, 1) = kResetPositionVar;
}

void PoseEKF::SetHeadingOffset(pdcsu::units::degree_t offset) {
  x_(2) = pdcsu::units::radian_t{offset}.value();
  P_.row(2).setZero();
  P_.col(2).setZero();
  P_(2, 2) = kResetHeadingVar;
}

Vector2D PoseEKF::position() const {
  return Vector2D{pdcsu::units::inch_t{x_(0)}, pdcsu::units::inch_t{x_(1)}};
}

pdcsu::units::degree_t PoseEKF::heading_offset() const {
  return pdcsu::units::radian_t{x_(2)};
}

pdcsu::units::degree_t PoseEKF::heading_stddev() const {
  return pdcsu::units::radian_t{std::sqrt(std::max(P_(2, 2), 0.0))};
}

double PoseEKF::position_variance() const {
  return (P_(0, 0) + P_(1, 1)) / 2.0;
}

}  // namespace funkit::robot::swerve::odometry
