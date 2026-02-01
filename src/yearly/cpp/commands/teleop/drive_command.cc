#include "commands/teleop/drive_command.h"

#include <utility>

#include "calculators/AntiTippingCalculator.h"
#include "funkit/math/fieldpoints.h"

DriveCommand::DriveCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, DriveCommand>{
          container, "drive_command"} {
  AddRequirements({&container_.drivetrain_});
}

void DriveCommand::OnInit() {}

void DriveCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  funkit::robot::swerve::DrivetrainTarget target{};

  double translate_x = funkit::math::HorizontalDeadband(
      ci_readings_.translate_x,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double translate_y = funkit::math::HorizontalDeadband(
      ci_readings_.translate_y,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double rotation = funkit::math::HorizontalDeadband(ci_readings_.rotation,
      container_.control_input_.GetPreferenceValue_double("rotation_deadband"),
      1, container_.control_input_.GetPreferenceValue_int("rotation_exponent"),
      1);

  fps_t max_speed =
      container_.drivetrain_.GetPreferenceValue_unit_type<fps_t>("max_speed");
  degps_t max_omega =
      container_.drivetrain_.GetPreferenceValue_unit_type<degps_t>("max_omega");

  target.velocity = {fps_t{translate_x * max_speed.value()},
      fps_t{translate_y * max_speed.value()}};

  Graph("target_velocity_x", target.velocity[0]);
  Graph("target_velocity_y", target.velocity[1]);

  //   auto delta_dir =
  //       (target.velocity -
  //       container_.drivetrain_.GetReadings().pose.velocity);

  //   Graph("delta_dir_x", delta_dir[0]);
  //   Graph("delta_dir_y", delta_dir[1]);

  // auto accel_limited = AntiTippingCalculator::LimitAcceleration(
  //     delta_dir, container_.drivetrain_.GetReadings().pose.bearing);

  // Graph("limited_accel_x", accel_limited[0]);
  // Graph("limited_accel_y", accel_limited[1]);

  // target.velocity[0] =
  //     1_fps * rampRateLimiter_x_.limit(target.velocity[0].to<double>(),
  //                 accel_limited[0].to<double>());
  // target.velocity[1] =
  //     1_fps * rampRateLimiter_y_.limit(target.velocity[1].to<double>(),
  //                 accel_limited[1].to<double>());

  target.angular_velocity = degps_t{rotation * max_omega.value()};

  bool isBlue = (frc::DriverStation::GetAlliance() ==
                 frc::DriverStation::Alliance::kBlue);

  if (isBlue) target.velocity = target.velocity.rotate(degree_t{180});

  GPDReadings gpd_readings = container_.GPD_.GetReadings();
  if (ci_readings_.gpd_drive_button && target.velocity.magnitude() > 1.0_fps_) {
    radian_t x_off = gpd_readings.optimal_pos - target.velocity.angle(true);
    fps_t x_off_comp = target.velocity.magnitude() / 2.0 *
                       std::tanh(x_off.value() * u_abs(x_off).value() * 400.0);
    degree_t dir_xoff = target.velocity.angle(true) + 90_deg_;
    ema_comp_gpd_ = 0.02 * x_off_comp + 0.98 * ema_comp_gpd_;
    Graph("x_off_comp", x_off_comp);
    if (u_abs(x_off_comp) > 1_fps_) {
      target.velocity = target.velocity + pdcsu::util::math::uVec<fps_t, 2>{
        ema_comp_gpd_, dir_xoff, true};
    } else {
        target.velocity = target.velocity + pdcsu::util::math::uVec<fps_t, 2>{
            ema_comp_gpd_ * u_abs(ema_comp_gpd_) / 1_fps_, dir_xoff, true};
    }
    Graph("ema_comp_gpd", ema_comp_gpd_);
  } else if (ci_readings_.gpd_drive_button ) {
    ema_comp_gpd_ = 0.95 * ema_comp_gpd_;
  } else {
    ema_comp_gpd_ = 0.0_fps_;
  }

  container_.drivetrain_.SetTarget({target});
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }