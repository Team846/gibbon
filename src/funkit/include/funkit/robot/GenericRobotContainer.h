#pragma once

#include "funkit/robot/GenericSubsystem.h"

namespace funkit::robot {

/**
 * GenericRobotContainer
 *
 * A class that inherits Loggable and provides functionality to orchestrate
 * registered subsystems. By registering subsystem groups under A, B, and AB,
 * this gives custom control over cycling behavior.
 */
class GenericRobotContainer : public funkit::base::Loggable {
public:
  GenericRobotContainer() : funkit::base::Loggable{"robot_container"} {}

  /**
   * RegisterSubsystemGroupA()
   *
   * @param subsystems - an initializer list that takes in pairs of subsystem
   * pointers and if it should be initialized.
   *
   * Registers subsystems under group A and initializes them.
   * These subsystems will run periodically every 20ms
   */
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

  /**
   * RegisterSubsystemGroupB()
   *
   * @param subsystems - an initializer list that takes in pairs of subsystem
   * pointers and if it should be initialized.
   *
   * Registers subsystems under group B and initializes them.
   * These subsystems will run periodically every 20ms on the alternate periodic
   * cycle from Group A.
   */
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

  /**
   * RegisterSubsystemGroupAB()
   *
   * @param subsystems - an initializer list that takes in pairs of subsystem
   * pointers and if it should be initialized.
   *
   * Registers subsystems under group AB and initializes them
   * These subsystems will run periodically every 10ms.
   */
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

  /**
   * UpdateReadings()
   *
   * Updates readings for all subsystems, depending on their cycle behavior.
   * GroupA and GroupB get updated every 20ms, whereas GroupAB gets updated
   * every 10ms
   */
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

  /**
   * UpdateHardware()
   *
   * Updates hardware for all subsystems, depending on their cycle behavior.
   * GroupA and GroupB get updated every 20ms, whereas GroupAB gets updated
   * every 10ms
   */
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

  // Sets up all subsystems
  void Setup() {
    for (auto subsystem : all_subsystems_) {
      subsystem->Setup();
    }
  }

  // Sets targets to zero for all subsystems
  void ZeroTargets() {
    for (auto subsystem : all_subsystems_) {
      subsystem->SetTargetZero();
    }
  }

  // Verify hardware for all subsystems
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
