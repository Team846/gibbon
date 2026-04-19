#pragma once

#include <networktables/NetworkTable.h>

#include <deque>
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
  pdcsu::units::degps_t angular_velocity;

  double aprilVarianceCoeff;
  double triangularVarianceCoeff;
  std::map<size_t, pdcsu::units::second_t> fudge_latency;
};

struct ATCalculatorOutput {
  Vector2D pos;
  double variance;
  pdcsu::units::degree_t bearing_from_tags;
  bool bearing_from_tags_valid;
  bool camera_disconnect = false;
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

/**
 * AprilTagCalculator
 * 
 * A class that inherits from Calculator. 
 * Uses AprilTags to determine position and bearing of robot, and angle of turret.
 */
class AprilTagCalculator : public funkit::math::Calculator<ATCalculatorInput,
                               ATCalculatorOutput, ATCalculatorConstants> {
public:
  AprilTagCalculator() {};

  /**
   * calculate()
   * 
   * Uses AprilTags to create fused/estimated positions. 
   * Using odometry history and variances compensates for latencies and account for uncertainty. 
   * @param input - An ATCalculatorInput with current readings and calculations of the robot
   * @return The calculated ATCalculatorOutput 
   */
  ATCalculatorOutput calculate(ATCalculatorInput input) override;

  static pdcsu::units::degree_t turret_angle;
  static pdcsu::units::degps_t turret_vel;

  static inch_t view_turret_off_x;
  static inch_t view_turret_off_y;
  static degree_t view_full_turret_angle;

  struct HistoryEntry {
    pdcsu::units::second_t time;
    Vector2D position;
    pdcsu::units::degree_t bearing;
    pdcsu::units::degree_t turret_angle;
  };
  std::deque<HistoryEntry> odom_history_{};
  static constexpr size_t kMaxHistorySize = 500;

  /**
   * AddToHistory() 
   * 
   * Adds a record of the most recent positions to odom_history. Pops the oldest records if maximum count is exceeded. 
   * @param time - the most recent time
   * @param position - the most recent position
   * @param bearing - the bearing of the robot
   * @param turret_angle - the most recent turret angle
   */
  void AddToHistory(pdcsu::units::second_t time, Vector2D position,
      pdcsu::units::degree_t bearing, pdcsu::units::degree_t turret_angle);


  // Finds the estimated position at a given time.
  Vector2D InterpolatePosition(pdcsu::units::second_t time) const;

  // Finds the estimated bearing at a given time.
  pdcsu::units::degree_t InterpolateRobotBearing(
      pdcsu::units::second_t time) const;

  // Finds the estimated turret angle at a given time.
  pdcsu::units::degree_t InterpolateTurretAngle(
      pdcsu::units::second_t time) const;

  Vector2D correction;
};
}