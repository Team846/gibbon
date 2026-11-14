#pragma once

#include <optional>

#include "funkit/math/fieldpoints.h"
#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

std::optional<funkit::math::FieldPoint> TakeAutoStartPoint();

#define ADD_AUTO_VARIANT(class_name, name, is_blue_side, is_left_side)   \
  AddAuto(name, new class_name{container_, is_blue_side, is_left_side}); \
  if (auto start = TakeAutoStartPoint()) {                               \
    auto_start_points_.insert_or_assign(name, *start);                   \
  }

#define ADD_AUTO_VARIANTS(class_name, name_string)                             \
  ADD_AUTO_VARIANT(class_name, std::string(name_string) + "/R/L", false, true) \
  ADD_AUTO_VARIANT(                                                            \
      class_name, std::string(name_string) + "/R/R", false, false)             \
  ADD_AUTO_VARIANT(class_name, std::string(name_string) + "/B/L", true, true)  \
  ADD_AUTO_VARIANT(class_name, std::string(name_string) + "/B/R", true, false)

class CS2Auto : public funkit::robot::GenericCommandGroup<RobotContainer,
                    CS2Auto, frc2::SequentialCommandGroup> {
public:
  CS2Auto(RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class TrailDepotAuto : public funkit::robot::GenericCommandGroup<RobotContainer,
                           TrailDepotAuto, frc2::SequentialCommandGroup> {
public:
  TrailDepotAuto(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class TrailShiftAuto : public funkit::robot::GenericCommandGroup<RobotContainer,
                           TrailShiftAuto, frc2::SequentialCommandGroup> {
public:
  TrailShiftAuto(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class TrailSafeAuto : public funkit::robot::GenericCommandGroup<RobotContainer,
                          TrailSafeAuto, frc2::SequentialCommandGroup> {
public:
  TrailSafeAuto(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
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

class SafeOPAuto : public funkit::robot::GenericCommandGroup<RobotContainer,
                       SafeOPAuto, frc2::SequentialCommandGroup> {
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