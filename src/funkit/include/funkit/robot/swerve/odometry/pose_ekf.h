#pragma once

#include <Eigen/Core>

#include "funkit/math/constants.h"
#include "funkit/robot/calculators/AprilTagCalculator.h"
#include "funkit/robot/swerve/odometry/swerve_pose.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve::odometry {

struct PoseEKFTagParams {
  double range_var_scale = 1.0;
  double bearing_var_scale = 1.0;
  double gate_chi2 = 13.8;
  int gate_release_count = 10;
};

class PoseEKF {
public:
  PoseEKF();

  void Predict(Vector2D delta, double position_var, double heading_var_deg2);

  bool Correct(const funkit::robot::calculators::TagObservation& obs,
      const PoseEKFTagParams& params);

  void SetPosition(Vector2D position);
  void SetHeadingOffset(pdcsu::units::degree_t offset);

  Vector2D position() const;
  pdcsu::units::degree_t heading_offset() const;
  pdcsu::units::degree_t heading_stddev() const;
  double position_variance() const;

private:
  static constexpr double kPi = funkit::math::constants::pi;
  static constexpr double kDeg2ToRad2 = (kPi / 180.0) * (kPi / 180.0);
  static constexpr double kUnknownPositionVar = 1e6;
  static constexpr double kResetPositionVar = 1.0;
  static constexpr double kResetHeadingVar = 4.0 * kDeg2ToRad2;
  static constexpr double kReleaseHeadingVar = 9.0 * kDeg2ToRad2;

  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
  int rejected_streak_ = 0;
};

}  // namespace funkit::robot::swerve::odometry
