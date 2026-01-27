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

struct Line {
  Vector2D point;
  pdcsu::units::degree_t angle;

  [[nodiscard]] Vector2D intersect(const Line& other) const {
    double x_val = (point[1].value() - other.point[1].value() +
                       std::tan(pdcsu::units::radian_t{other.angle}.value()) *
                           other.point[0].value() -
                       std::tan(pdcsu::units::radian_t{angle}.value()) *
                           point[0].value()) /
                   (std::tan(pdcsu::units::radian_t{other.angle}.value()) -
                       std::tan(pdcsu::units::radian_t{angle}.value()));
    double y_val = std::tan(pdcsu::units::radian_t{angle}.value()) *
                       (x_val - point[0].value()) +
                   point[1].value();
    return {pdcsu::units::inch_t{x_val}, pdcsu::units::inch_t{y_val}};
  }

  void translate(const Vector2D& addend) { point += addend; }
};

struct ATCalculatorInput {
  funkit::robot::swerve::odometry::SwervePose pose;
  funkit::robot::swerve::odometry::SwervePose old_pose;
  pdcsu::units::degps_t angular_velocity;

  double aprilVarianceCoeff;
  double triangularVarianceCoeff;
  std::map<int, pdcsu::units::second_t> fudge_latency;
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
};

struct ATCalculatorConstants {
  std::map<int, AprilTagData> tag_locations;
  std::vector<AprilTagCamera> cameras;
};

class AprilTagCalculator : public funkit::math::Calculator<ATCalculatorInput,
                               ATCalculatorOutput, ATCalculatorConstants> {
public:
  AprilTagCalculator() {};

  ATCalculatorOutput calculate(ATCalculatorInput input) override;

private:
  Vector2D correction;
  Vector2D getPos(pdcsu::units::degree_t bearing, pdcsu::units::degree_t theta,
      pdcsu::units::inch_t distance, int tag,
      const AprilTagCameraConfig& config);
};
}