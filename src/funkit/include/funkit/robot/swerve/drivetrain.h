#pragma once

#include <frc/smartdashboard/Field2d.h>

#include <array>
#include <memory>

#include "ctre/phoenix6/Pigeon2.hpp"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/robot/calculators/AprilTagCalculator.h"
#include "funkit/robot/swerve/control/swerve_ol_calculator.h"
#include "funkit/robot/swerve/odometry/pose_estimator.h"
#include "funkit/robot/swerve/odometry/swerve_odometry_calculator.h"
#include "funkit/robot/swerve/odometry/swerve_pose.h"
#include "funkit/robot/swerve/path_logger.h"
#include "funkit/robot/swerve/swerve_module.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve {
class SwerveModuleSubsystem;
}

namespace funkit::robot::swerve {

/*
DrivetrainConfigs

Contains all configs related to the specific drivetrain in use.
*/
using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

struct DrivetrainConfigs {
  int pigeon_CAN_id;

  SwerveModuleCommonConfig module_common_config;
  std::array<SwerveModuleUniqueConfig, 4> module_unique_configs;

  pdcsu::units::inch_t wheelbase_horizontal_dim;
  pdcsu::units::inch_t wheelbase_forward_dim;

  pdcsu::units::fps_t max_speed;

  std::vector<pdcsu::units::inch_t> camera_x_offsets;
  std::vector<pdcsu::units::inch_t> camera_y_offsets;
  size_t cams;

  std::map<int, funkit::robot::calculators::AprilTagData> april_locations;
  pdcsu::units::fps2_t max_accel;
};

struct DrivetrainReadings {
  funkit::robot::swerve::odometry::SwervePose pose;
  Vector2D april_point;
  funkit::robot::swerve::odometry::SwervePose estimated_pose;
  pdcsu::units::degps_t yaw_rate;
  pdcsu::units::fps2_t acceleration;
  int see_tag_counter;
};

struct DrivetrainTarget {
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity;
  pdcsu::units::degps_t angular_velocity;
  pdcsu::units::fps2_t accel_clamp = pdcsu::units::fps2_t{-1};
  bool cut_excess_steering = false;
};

/*
DrivetrainSubsystem

A generic class to control a 4-module Kraken x60 swerve drive with CANCoders.
*/
class DrivetrainSubsystem
    : public funkit::robot::GenericSubsystem<DrivetrainReadings,
          DrivetrainTarget> {
public:
  DrivetrainSubsystem(DrivetrainConfigs configs);

  void Setup() override;

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroBearing();

  void SetBearing(pdcsu::units::degree_t bearing);
  void SetPosition(Vector2D position);
  void SetOdomBearing(pdcsu::units::degree_t odom_bearing);

  void SetCANCoderOffsets();

  pdcsu::units::degps_t ApplyBearingPID(pdcsu::units::degree_t target_bearing);

  /**
   * Start recording the robot's path.
   *
   * @param filename The base filename to save the path to (without extension).
   */
  void StartPathRecording(const std::string& filename);

  /**
   * Stop recording the robot's path and save it to a file.
   *
   * @return True if the file was saved successfully.
   */
  bool StopPathRecording();

  /**
   * Check if path recording is active.
   *
   * @return True if recording, false otherwise.
   */
  bool IsPathRecording() const;

  /**
   * Set a field object pose with PDCSU units (handles conversion to WPILib).
   *
   * @param name The name of the field object.
   * @param position Position in PDCSU units (inch_t).
   * @param rotation Rotation in PDCSU units (degree_t).
   */
  void SetFieldObjectPose(const std::string& name,
      pdcsu::util::math::uVec<pdcsu::units::inch_t, 2> position,
      pdcsu::units::degree_t rotation);

private:
  DrivetrainReadings ReadFromHardware() override;

  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> compensateForSteerLag(
      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> uncompensated);

  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> accelClampHelper(
      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity,
      pdcsu::units::fps2_t accel_clamp);

  void WriteVelocitiesHelper(
      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity,
      pdcsu::units::degps_t angular_velocity, bool cut_excess_steering,
      pdcsu::units::fps_t speed_limit);
  void WriteToHardware(DrivetrainTarget target) override;

  DrivetrainConfigs configs_;
  std::array<std::unique_ptr<SwerveModuleSubsystem>, 4> modules_;

  ctre::phoenix6::hardware::Pigeon2 pigeon_;

  funkit::robot::swerve::odometry::SwerveOdometryCalculator odometry_;
  funkit::robot::swerve::control::SwerveOpenLoopCalculator ol_calculator_;
  funkit::robot::calculators::AprilTagCalculator tag_pos_calculator;
  funkit::robot::swerve::odometry::PoseEstimator pose_estimator{
      {pdcsu::units::foot_t{0}, pdcsu::units::foot_t{0}},
      {pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}},
      {pdcsu::units::fps2_t{0}, pdcsu::units::fps2_t{0}}};

  // Path logger for recording odometry data
  PathLogger path_logger_;

  bool first_loop = true;

  pdcsu::units::degree_t bearing_offset_ = pdcsu::units::degree_t{0};

  int see_tag_counter_ = 100001;

  pdcsu::units::degps_t cached_max_omega_cut_{pdcsu::units::degps_t{0}};
  pdcsu::units::fps_t cached_max_speed_{pdcsu::units::fps_t{0}};
  pdcsu::units::second_t cached_steer_lag_{pdcsu::units::second_t{0}};
  pdcsu::units::second_t cached_bearing_latency_{pdcsu::units::second_t{0}};
  double cached_odom_fudge_factor_ = 0.0;
  double cached_odom_variance_ = 0.0;
  double cached_pose_variance_ = 0.0;
  double cached_velocity_variance_ = 0.0;
  double cached_accel_variance_ = 0.0;
  bool cached_pose_override_ = false;
  double cached_april_variance_coeff_ = 0.0;
  double cached_triangular_variance_coeff_ = 0.0;
  pdcsu::units::ms_t cached_fudge_latency1_{pdcsu::units::ms_t{0}};
  pdcsu::units::ms_t cached_fudge_latency2_{pdcsu::units::ms_t{0}};
  pdcsu::units::ms_t cached_april_bearing_latency_{pdcsu::units::ms_t{0}};

  frc::Field2d MainField_;
};

}  // namespace funkit::robot::swerve