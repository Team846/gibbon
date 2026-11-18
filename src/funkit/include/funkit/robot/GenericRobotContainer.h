#pragma once

#include <chrono>
#include <string>
#include <unordered_map>

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
    slowest_subsystem_ = nullptr;
    slowest_subsystem_ms_ = 0.0;
    if (read_counter % 2 == 0) {
      for (auto subsystem : group_a_subsystems_) {
        Timed(subsystem, [subsystem] { subsystem->UpdateReadings(); });
      }
      GroupAUpdateReadingsExtension();
    } else {
      for (auto subsystem : group_b_subsystems_) {
        Timed(subsystem, [subsystem] { subsystem->UpdateReadings(); });
      }
      GroupBUpdateReadingsExtension();
    }
    read_counter++;
  }

  void UpdateHardware() {
    if (write_counter % 2 == 0) {
      for (auto subsystem : group_a_subsystems_) {
        Timed(subsystem, [subsystem] { subsystem->UpdateHardware(); });
      }
      GroupAUpdateHardwareExtension();
    } else {
      for (auto subsystem : group_b_subsystems_) {
        Timed(subsystem, [subsystem] { subsystem->UpdateHardware(); });
      }
      GroupBUpdateHardwareExtension();
    }
    write_counter++;
  }

  std::string SlowestSubsystem() const {
    return slowest_subsystem_ ? slowest_subsystem_->name() : "none";
  }
  double SlowestSubsystemMs() const { return slowest_subsystem_ms_; }

  struct SubsystemTiming {
    double total_ms{0.0};
    double max_ms{0.0};
  };
  std::unordered_map<funkit::robot::SubsystemBase*, SubsystemTiming>&
  SubsystemTimings() {
    return subsystem_timings_;
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

  unsigned int read_counter{0};
  unsigned int write_counter{0};

private:
  template <typename F>
  void Timed(funkit::robot::SubsystemBase* subsystem, F&& update) {
    auto start = std::chrono::steady_clock::now();
    update();
    double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start)
                    .count();
    if (ms > slowest_subsystem_ms_) {
      slowest_subsystem_ms_ = ms;
      slowest_subsystem_ = subsystem;
    }
    auto& timing = subsystem_timings_[subsystem];
    timing.total_ms += ms;
    if (ms > timing.max_ms) { timing.max_ms = ms; }
  }

  funkit::robot::SubsystemBase* slowest_subsystem_{nullptr};
  double slowest_subsystem_ms_{0.0};
  std::unordered_map<funkit::robot::SubsystemBase*, SubsystemTiming>
      subsystem_timings_{};
};

}  // namespace funkit::robot
