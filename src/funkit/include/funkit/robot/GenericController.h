#pragma once

#include <frc/GenericHID.h>

namespace funkit::robot {

/**
 * GenericControllerReadings
 *
 * A struct representing values of buttons 1–8 on a Human Interface Device
 * (HID). Default construction leaves members uninitialized unless
 * value-initialized
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