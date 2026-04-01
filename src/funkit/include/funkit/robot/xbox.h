#pragma once

#include <frc/XboxController.h>

namespace nt {
class NetworkTable;
}

namespace funkit::robot {
/**
 * XboxPOV
 * 
 * An enum that classifies the controller's hat switch into 8 directions plus kNone, depending on its direction.
 */
enum class XboxPOV : int {
  kNone = -1,
  kUp = 0,
  kUpRight = 45,
  kRight = 90,
  kDownRight = 135,
  kDown = 180,
  kDownLeft = 225,
  kLeft = 270,
  kUpLeft = 315
};

/**
 * XboxReadings
 * 
 * A struct holding variables for all xBox inputs.
 */
struct XboxReadings {
  double left_stick_x;   // [-1, 1]
  double left_stick_y;   // [-1, 1]
  double right_stick_x;  // [-1, 1]
  double right_stick_y;  // [-1, 1]

  bool left_trigger;
  bool right_trigger;
  bool left_bumper;
  bool right_bumper;

  bool back_button;
  bool start_button;
  bool lsb;
  bool rsb;

  bool a_button;
  bool b_button;
  bool x_button;
  bool y_button;

  funkit::robot::XboxPOV pov;

  // Default constructor
  XboxReadings() = default;
  // Custom constructor that takes in sa snaopshot of controller state. Uses trigger_threshold to convert analog trigger axes into booleans.
  XboxReadings(frc::XboxController& xbox, double trigger_threshold);
};

/**
 * XboxReadingsFromSimDS()
 * 
 * @param xbox_table - ntable that contains xbox information
 * @param trigger_threshold - threshold to convert analog trigger axes into booleans
 * @param out - readings of xbox controller instance
 * @return True if data was successfully retrieved from NetworkTables
 */ 

bool XboxReadingsFromSimDS(
    nt::NetworkTable* xbox_table, double trigger_threshold, XboxReadings* out);

}  // namespace funkit::robot
