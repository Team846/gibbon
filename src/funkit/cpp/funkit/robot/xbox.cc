#include "funkit/robot/xbox.h"

#include <networktables/NetworkTable.h>

namespace funkit::robot {

bool XboxReadingsFromSimDS(
    nt::NetworkTable* xbox_table, double trigger_threshold, XboxReadings* out) {
  if (xbox_table == nullptr || out == nullptr) return false;
  if (!xbox_table->GetEntry("left_stick_x").Exists()) return false;
  out->left_stick_x = xbox_table->GetEntry("left_stick_x").GetDouble(0.0);
  out->left_stick_y = xbox_table->GetEntry("left_stick_y").GetDouble(0.0);
  out->right_stick_x = xbox_table->GetEntry("right_stick_x").GetDouble(0.0);
  out->right_stick_y = xbox_table->GetEntry("right_stick_y").GetDouble(0.0);
  out->left_trigger = xbox_table->GetEntry("left_trigger").GetBoolean(false);
  out->right_trigger = xbox_table->GetEntry("right_trigger").GetBoolean(false);
  out->left_bumper = xbox_table->GetEntry("left_bumper").GetBoolean(false);
  out->right_bumper = xbox_table->GetEntry("right_bumper").GetBoolean(false);
  out->back_button = xbox_table->GetEntry("back_button").GetBoolean(false);
  out->start_button = xbox_table->GetEntry("start_button").GetBoolean(false);
  out->lsb = xbox_table->GetEntry("lsb").GetBoolean(false);
  out->rsb = xbox_table->GetEntry("rsb").GetBoolean(false);
  out->a_button = xbox_table->GetEntry("a_button").GetBoolean(false);
  out->b_button = xbox_table->GetEntry("b_button").GetBoolean(false);
  out->x_button = xbox_table->GetEntry("x_button").GetBoolean(false);
  out->y_button = xbox_table->GetEntry("y_button").GetBoolean(false);
  int pov_angle = static_cast<int>(xbox_table->GetEntry("pov").GetDouble(-1.0));
  out->pov = (pov_angle >= 0 && pov_angle <= 315)
                 ? static_cast<XboxPOV>(pov_angle)
                 : XboxPOV::kNone;
  return true;
}

XboxReadings::XboxReadings(frc::XboxController& xbox, double trigger_threshold)
    : left_stick_x(xbox.GetLeftX()),
      left_stick_y(-xbox.GetLeftY()),  // negated so + is up
      right_stick_x(xbox.GetRightX()),
      right_stick_y(-xbox.GetRightY()),  // negated so + is up
      left_trigger(xbox.GetLeftTriggerAxis() >= trigger_threshold),
      right_trigger(xbox.GetRightTriggerAxis() >= trigger_threshold),
      left_bumper(xbox.GetLeftBumperButton()),
      right_bumper(xbox.GetRightBumperButton()),
      back_button(xbox.GetBackButton()),
      start_button(xbox.GetStartButton()),
      lsb(xbox.GetLeftStickButton()),
      rsb(xbox.GetRightStickButton()),
      a_button(xbox.GetAButton()),
      b_button(xbox.GetBButton()),
      x_button(xbox.GetXButton()),
      y_button(xbox.GetYButton()),
      pov(static_cast<funkit::robot::XboxPOV>(xbox.GetPOV())) {}

}  // namespace funkit::robot