#pragma once

#include "funkit/robot/GenericSubsystem.h"

namespace funkit::robot {

class GenericRobotContainer : public funkit::base::Loggable {
public:
  GenericRobotContainer() : funkit::base::Loggable{"robot_container"} {}

  void RegisterSubsystemGroupA(
      std::initializer_list<std::pair<funkit::robot::SubsystemBase*, bool>>
          subsystems) {
    for (auto& [subsystem, init] : subsystems) {
      if (init) {
        subsystem->Init();
        group_a_subsystems_.push_back(subsystem);
        all_subsystems_.push_back(subsystem);
      }
    }
  }

  void RegisterSubsystemGroupB(
      std::initializer_list<std::pair<funkit::robot::SubsystemBase*, bool>>
          subsystems) {
    for (auto& [subsystem, init] : subsystems) {
      if (init) {
        subsystem->Init();
        group_b_subsystems_.push_back(subsystem);
        all_subsystems_.push_back(subsystem);
      }
    }
  }

  void RegisterSubsystemGroupAB(
      std::initializer_list<std::pair<funkit::robot::SubsystemBase*, bool>>
          subsystems) {
    for (auto& [subsystem, init] : subsystems) {
      if (init) {
        subsystem->Init();
        group_a_subsystems_.push_back(subsystem);
        group_b_subsystems_.push_back(subsystem);
        all_subsystems_.push_back(subsystem);
      }
    }
  }

  void UpdateReadings() {
    if (read_counter % 2 == 0) {
      for (auto subsystem : group_a_subsystems_) {
        subsystem->UpdateReadings();
      }
      GroupAUpdateReadingsExtension();
    } else {
      for (auto subsystem : group_b_subsystems_) {
        subsystem->UpdateReadings();
      }
      GroupBUpdateReadingsExtension();
    }
    read_counter++;
  }

  void UpdateHardware() {
    if (write_counter % 2 == 0) {
      for (auto subsystem : group_a_subsystems_) {
        subsystem->UpdateHardware();
      }
      GroupAUpdateHardwareExtension();
    } else {
      for (auto subsystem : group_b_subsystems_) {
        subsystem->UpdateHardware();
      }
      GroupBUpdateHardwareExtension();
    }
    write_counter++;
  }

  void Setup() {
    for (auto subsystem : all_subsystems_) {
      subsystem->Setup();
    }
  }

  void ZeroTargets() {
    for (auto subsystem : all_subsystems_) {
      subsystem->SetTargetZero();
    }
  }

  void VerifyHardware() {
    for (auto subsystem : all_subsystems_) {
      subsystem->VerifyHardware();
    }
  }

  virtual void GroupAUpdateReadingsExtension() {};
  virtual void GroupBUpdateReadingsExtension() {};
  virtual void GroupAUpdateHardwareExtension() {};
  virtual void GroupBUpdateHardwareExtension() {};

protected:
  std::vector<funkit::robot::SubsystemBase*> all_subsystems_{};
  std::vector<funkit::robot::SubsystemBase*> group_a_subsystems_{};
  std::vector<funkit::robot::SubsystemBase*> group_b_subsystems_{};

  unsigned int read_counter;
  unsigned int write_counter;
};

}  // namespace funkit::robot
