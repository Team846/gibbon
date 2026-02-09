#pragma once

#include <networktables/NetworkTable.h>

#include <map>
#include <memory>

#include "funkit/math/calculator.h"
#include "funkit/robot/swerve/odometry/swerve_pose.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::calculators {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

struct ATCalculatorInput {
  funkit::robot::swerve::odometry::SwervePose pose;
  funkit::robot::swerve::odometry::SwervePose old_pose;
  pdcsu::units::degps_t angular_velocity;

  double aprilVarianceCoeff;
  double triangularVarianceCoeff;
  std::map<size_t, pdcsu::units::second_t> fudge_latency;
  pdcsu::units::second_t bearing_latency;
};

struct ATCalculatorOutput {
  Vector2D pos;
  double variance;
};

struct AprilTagData {
  pdcsu::units::inch_t x_pos;
  pdcsu::units::inch_t y_pos;
};

struct AprilTagCameraConfig {
  size_t camera_id;
  pdcsu::units::inch_t x_offset;
  pdcsu::units::inch_t y_offset;
};

struct AprilTagCamera {
  AprilTagCameraConfig config;
  std::shared_ptr<nt::NetworkTable> table;
  bool equiv_turret = false;
};

struct TurretTagCameraConfig {
  size_t camera_id;
  pdcsu::units::inch_t turret_x_offset;
  pdcsu::units::inch_t turret_y_offset;
  pdcsu::units::inch_t x_offset;
  pdcsu::units::inch_t y_offset;
};

struct TurretTagCamera {
  TurretTagCameraConfig config;
  std::shared_ptr<nt::NetworkTable> table;
};

struct ATCalculatorConstants {
  std::map<size_t, AprilTagData> tag_locations;
  std::vector<AprilTagCamera> cameras;
  std::optional<TurretTagCamera> turret_camera;
};

class AprilTagCalculator : public funkit::math::Calculator<ATCalculatorInput,
                               ATCalculatorOutput, ATCalculatorConstants> {
public:
  AprilTagCalculator() {};

  ATCalculatorOutput calculate(ATCalculatorInput input) override;

  static pdcsu::units::degree_t turret_angle;
  static pdcsu::units::degps_t turret_vel;

  static inch_t view_turret_off_x;
  static inch_t view_turret_off_y;
  static degree_t view_full_turret_angle;

private:
  Vector2D correction;
};
}