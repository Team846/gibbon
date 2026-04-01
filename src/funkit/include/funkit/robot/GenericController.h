#pragma once

#include <frc/GenericHID.h>

namespace funkit::robot {

/**
 * GenericControllerReadings
 * 
 * Snapshot of raw HID (Human Interface Device) buttons 1–8 from frc::GenericHID::GetRawButton()
 * Default construction leaves members uninitialized unless value-initialized
 */
struct GenericControllerReadings {
  bool one_button;
  bool two_button;
  bool three_button;
  bool four_button;
  bool five_button;
  bool six_button;
  bool seven_button;
  bool eight_button;

  GenericControllerReadings() = default;
  GenericControllerReadings(frc::GenericHID& controller);
};

}  // namespace funkit::robot