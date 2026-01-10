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

  RegisterPreference("max_gp_diff", inch_t{10});
  RegisterPreference("use_diff_thresh", inch_t{40});
}

GPDTarget GPDSubsystem::ZeroTarget() const { return GPDTarget{}; }

bool GPDSubsystem::VerifyHardware() { return true; }

void GPDSubsystem::Setup() {}

std::pair<funkit::math::Vector2D, bool> GPDSubsystem::getBestGP(
    const std::vector<funkit::math::Vector2D> algae) {
  if (algae.size() == 0U) { return {{inch_t{0}, inch_t{0}}, false}; }

  funkit::math::Vector2D closest_algae;

  auto robot_pose = drivetrain_->GetReadings().pose;

  if (robot_pose.velocity.magnitude().value() >= 2.0) {
    degree_t min_angle{degree_t{180}};
    for (size_t i = 0; i < algae.size(); i++) {
      funkit::math::Vector2D this_algae = algae.at(i);
      degree_t angle = robot_pose.velocity.angleTo(this_algae, true);

      if (angle.value() < min_angle.value()) {
        min_angle = angle;
        closest_algae = this_algae;
      }
    }
  } else {
    inch_t min_dist{inch_t{1000}};
    for (size_t i = 0; i < algae.size(); i++) {
      funkit::math::Vector2D this_algae = algae.at(i);
      inch_t dist = (this_algae - robot_pose.position).magnitude();

      if (dist.value() < min_dist.value()) {
        min_dist = dist;
        closest_algae = this_algae;
      }
    }
  }

  return {closest_algae, true};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings;
  funkit::robot::swerve::DrivetrainReadings drivetrain_readings =
      drivetrain_->GetReadings();

  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});

  auto latency_entry = gpdTable->GetEntry("tl");
  auto nt_delay =
      funkit::wpilib::CurrentFPGATime() - ms_t(latency_entry.GetLastChange());
  Graph("nt_delay", second_t(nt_delay));
  second_t latency{second_t{latency_entry.GetDouble(0.005)} + nt_delay};
  Graph("latency", latency);

  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});

  readings.gamepieces.clear();

  for (size_t i = 0; i < distances.size() && i < theta_x.size(); ++i) {
    auto vel_x_inches_per_sec =
        inch_t{drivetrain_readings.estimated_pose.velocity[0].value() * 12.0} /
        second_t{1.0};
    auto vel_y_inches_per_sec =
        inch_t{drivetrain_readings.estimated_pose.velocity[1].value() * 12.0} /
        second_t{1.0};
    auto yaw_rate_deg_per_sec = degps_t{drivetrain_readings.yaw_rate.value()};
    auto bearing_angle =
        drivetrain_readings.estimated_pose.bearing -
        degree_t{yaw_rate_deg_per_sec.value() * latency.value()} +
        degree_t{theta_x[i]} +
        GetPreferenceValue_unit_type<degree_t>("cam_h_angle");

    funkit::math::Vector2D vel_comp{
        inch_t{vel_x_inches_per_sec.value() * latency.value()},
        inch_t{vel_y_inches_per_sec.value() * latency.value()}};

    funkit::math::Vector2D cam_offset{
        GetPreferenceValue_unit_type<inch_t>("intake_to_cam_x"),
        GetPreferenceValue_unit_type<inch_t>("intake_to_cam_y")};

    readings.gamepieces.push_back(
        funkit::math::Vector2D{inch_t{distances[i]}, bearing_angle, true} +
        drivetrain_readings.estimated_pose.position - vel_comp +
        cam_offset.rotate(drivetrain_readings.estimated_pose.bearing));
  }

  int num_gps = readings.gamepieces.size();

  gp_spin_ += degree_t{5};

  for (int i = 0; i < std::min(20, num_gps); i++) {
    drivetrain_->SetFieldObjectPose(
        std::to_string(i), readings.gamepieces[i], gp_spin_);
  }
  for (int i = std::min(20, num_gps); i < 20; i++) {
    drivetrain_->SetFieldObjectPose(
        std::to_string(i), {inch_t{100 * 12}, inch_t{100 * 12}}, degree_t{0});
  }

  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}