#include "subsystems/abstract/control_input.h"

ControlInputSubsystem::ControlInputSubsystem(
    funkit::robot::swerve::DrivetrainSubsystem* drivetrain_ss)
    : funkit::robot::GenericSubsystem<ControlInputReadings,
          ControlInputTarget>{"control_input"},
      drivetrain_ss_{drivetrain_ss} {}

void ControlInputSubsystem::Setup() {
  RegisterPreference("trigger_threshold", 0.3);

  RegisterPreference("translation_deadband", 0.07);
  RegisterPreference("translation_exponent", 2);

  RegisterPreference("rotation_deadband", 0.07);
  RegisterPreference("rotation_exponent", 2);

  RegisterPreference("op_deadband", 0.35);
}

ControlInputTarget ControlInputSubsystem::ZeroTarget() const {
  ControlInputTarget target;
  target.driver_rumble = false;
  target.operator_rumble = false;
  return target;
}

bool ControlInputSubsystem::VerifyHardware() { return true; }

ControlInputReadings ControlInputSubsystem::ReadFromHardware() {
  ControlInputReadings readings = UpdateWithInput();

  if (readings.zero_bearing != previous_readings_.zero_bearing) {
    Log("ControlInput [Drivetrain Zeroing] state changed to {}",
        readings.zero_bearing ? 1 : 0);
  }

  previous_readings_ = readings;

  return readings;
}

void ControlInputSubsystem::WriteToHardware(ControlInputTarget target) {
  driver_.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
      target.driver_rumble ? 1.0 : 0.0);
  operator_.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
      target.driver_rumble ? 1.0 : 0.0);
}

ControlInputReadings ControlInputSubsystem::UpdateWithInput() {
  ControlInputReadings ci_readings_{};
  auto trigger_threshold = GetPreferenceValue_double("trigger_threshold");
  funkit::robot::XboxReadings dr_readings{driver_, trigger_threshold};
  funkit::robot::XboxReadings op_readings{operator_, trigger_threshold};
  funkit::robot::GenericControllerReadings op_keyboard_readings{
      operator_keyboard_};

  ci_readings_.zero_bearing = dr_readings.back_button;
  ci_readings_.translate_x = dr_readings.left_stick_x;
  ci_readings_.translate_y = dr_readings.left_stick_y;
  ci_readings_.rotation = dr_readings.right_stick_x;

  Graph("translate_x", ci_readings_.translate_x);
  Graph("translate_y", ci_readings_.translate_y);

  ci_readings_.diagonalize_bump = dr_readings.left_bumper;
  ci_readings_.intake = dr_readings.left_trigger;
  ci_readings_.climb_align = dr_readings.right_bumper;
  ci_readings_.point_blank_shot = dr_readings.right_trigger;

  ci_readings_.override_autoshoot = op_readings.left_trigger;
  ci_readings_.force_shoot = op_readings.left_bumper;
  ci_readings_.agitate = op_readings.lsb;

  ci_readings_.hood_trim_cw = op_readings.pov == funkit::robot::XboxPOV::kUp;
  ci_readings_.hood_trim_ccw = op_readings.pov == funkit::robot::XboxPOV::kDown;

  ci_readings_.turret_trim_cw =
      op_readings.pov == funkit::robot::XboxPOV::kLeft;
  ci_readings_.turret_trim_ccw =
      op_readings.pov == funkit::robot::XboxPOV::kRight;

  ci_readings_.pass_mode = op_readings.right_trigger;
  ci_readings_.descend_l1 = op_readings.right_bumper;
  ci_readings_.override_force_assist = op_readings.y_button;
  ci_readings_.evac_storage = op_readings.a_button;
  ci_readings_.rev_dye_rotor = op_readings.left_stick_x < -0.5;
  ci_readings_.home = op_readings.back_button;

  ci_readings_.turret_no_spin = op_keyboard_readings.one_button;

  previous_driver_ = dr_readings;
  previous_operator_ = op_readings;
  previous_operator_keyboard_ = op_keyboard_readings;

  return ci_readings_;
}