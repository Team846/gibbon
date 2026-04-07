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

class CompatibilityAuto
    : public funkit::robot::GenericCommandGroup<RobotContainer,
          CompatibilityAuto, frc2::SequentialCommandGroup> {
public:
  CompatibilityAuto(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class OPAuto : public funkit::robot::GenericCommandGroup<RobotContainer, OPAuto,
                   frc2::SequentialCommandGroup> {
public:
  OPAuto(RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class SafeOPAuto : public funkit::robot::GenericCommandGroup<RobotContainer, SafeOPAuto,
                   frc2::SequentialCommandGroup> {
public:
  SafeOPAuto(RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class Center8 : public funkit::robot::GenericCommandGroup<RobotContainer,
                    Center8, frc2::SequentialCommandGroup> {
public:
  Center8(RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class Center8Depot : public funkit::robot::GenericCommandGroup<RobotContainer,
                    Center8Depot, frc2::SequentialCommandGroup> {
public:
  Center8Depot(RobotContainer& container, bool is_blue_side, bool is_left_side);
};