#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

#define ADD_AUTO_VARIANTS(class_name, name_string) \
  AddAuto(std::string(name_string) + "/R/L",       \
      new class_name{container_, false, true});    \
  AddAuto(std::string(name_string) + "/R/R",       \
      new class_name{container_, false, false});   \
  AddAuto(std::string(name_string) + "/B/L",       \
      new class_name{container_, true, true});     \
  AddAuto(std::string(name_string) + "/B/R",       \
      new class_name{container_, true, false});

class CS2Auto : public funkit::robot::GenericCommandGroup<RobotContainer,
                    CS2Auto, frc2::SequentialCommandGroup> {
public:
  CS2Auto(RobotContainer& container, bool is_blue_side, bool is_left_side);
};