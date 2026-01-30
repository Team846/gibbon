#include "subsystems/abstract/gpd.h"

#include <frc/DriverStation.h>

#include <vector>

#include "funkit/wpilib/time.h"
#include "pdcsu_units.h"

GPDSubsystem::GPDSubsystem(
    funkit::robot::swerve::DrivetrainSubsystem* drivetrain)
    : funkit::robot::GenericSubsystem<GPDReadings, GPDTarget>{"GPD"},
      drivetrain_{drivetrain} {
  RegisterPreference("intake_to_cam_y", inch_t{0});
  RegisterPreference("intake_to_cam_x", inch_t{-13});
  RegisterPreference("cam_h_angle", degree_t{0});
  RegisterPreference("kS", 0.005);
}

GPDTarget GPDSubsystem::ZeroTarget() const { return GPDTarget{}; }

bool GPDSubsystem::VerifyHardware() { return true; }

void GPDSubsystem::Setup() {}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings{};

  funkit::robot::swerve::DrivetrainReadings drivetrain_readings =
      drivetrain_->GetReadings();

  std::vector<double> optimal_point =
      gpdTable->GetNumberArray("optimal_target", {});

  auto latency_entry = gpdTable->GetEntry("tl");
  auto nt_delay =
      funkit::wpilib::CurrentFPGATime() - ms_t(latency_entry.GetLastChange());
  Graph("nt_delay", second_t(nt_delay));
  second_t latency{second_t{latency_entry.GetDouble(0.005)} + nt_delay};
  Graph("latency", latency);

  double distance = optimal_point[0];
  double theta_x = optimal_point[1];

  auto vel_x_ =
      inch_t{drivetrain_readings.estimated_pose.velocity[0].value() * 12.0};
  auto vel_y_ =
      inch_t{drivetrain_readings.estimated_pose.velocity[1].value() * 12.0};
  auto yaw_rate_deg_per_sec = degps_t{drivetrain_readings.yaw_rate.value()};

  auto bearing_angle =
      drivetrain_readings.estimated_pose.bearing -  // fxi
      degree_t{yaw_rate_deg_per_sec.value() * latency.value()} +
      degree_t{theta_x};

  funkit::math::Vector2D vel_comp{inch_t{vel_x_.value() * latency.value()},
      inch_t{vel_y_.value() * latency.value()}};

  funkit::math::Vector2D cam_offset{
      GetPreferenceValue_unit_type<inch_t>("intake_to_cam_x"),
      GetPreferenceValue_unit_type<inch_t>("intake_to_cam_y")};

  funkit::math::Vector2D raw_pos =
      funkit::math::Vector2D{inch_t{distance}, bearing_angle, true} +
      drivetrain_readings.estimated_pose.position - vel_comp +
      cam_offset.rotate(drivetrain_readings.estimated_pose.bearing, true);

  readings.optimal_pos = raw_pos;
  readings.has_target = true;

  Graph("target_x", readings.optimal_pos[0]);
  Graph("target_y", readings.optimal_pos[1]);

  // gp_spin_ += degree_t{5};
  // drivetrain_->SetFieldObjectPose("optimal_gp", readings.optimal_pos,
  // gp_spin_);

  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}