#pragma once

#include <variant>

#include "funkit/control/base/motor_specs.h"
#include "funkit/control/config/genome.h"
#include "funkit/control/hardware/IntermediateController.h"
#include "pdcsu_units.h"
#include "simulation/simbldc.h"
#include "util/sysdef.h"

namespace funkit::control::simulation {

class VirtualMonkey : public funkit::control::hardware::IntermediateController {
private:
  pdcsu::util::BasePlant ConstructPlant(funkit::control::base::MotorSpecs specs,
      pdcsu::units::ohm_t circuit_resistance,
      pdcsu::units::kgm2_t rotational_inertia, double friction);

public:
  VirtualMonkey(funkit::control::base::MotorSpecs specs,
      pdcsu::units::ohm_t circuit_resistance,
      pdcsu::units::kgm2_t rotational_inertia, double friction);

  void Tick() override;

  bool VerifyConnected() override { return true; }

  void ZeroEncoder(pdcsu::units::radian_t position) override;

  funkit::control::hardware::ControllerErrorCodes GetLastErrorCode() override {
    return funkit::control::hardware::ControllerErrorCodes::kAllOK;
  }

  void SetSoftLimits(pdcsu::units::radian_t forward_limit,
      pdcsu::units::radian_t reverse_limit) override {
    (void)forward_limit;
    (void)reverse_limit;
  }

  void SetGenome(config::MotorGenome genome) override;

  void EnableStatusFrames(config::StatusFrameSelections frames,
      pdcsu::units::ms_t faults_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t velocity_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t encoder_position_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t analog_position_ms = pdcsu::units::ms_t{20}) override;

  void OverrideStatusFramePeriod(funkit::control::config::StatusFrame frame,
      pdcsu::units::ms_t period) override {
    (void)frame;
    (void)period;
  }

  bool IsDuplicateControlMessage(base::ControlRequest cr) override;

  hardware::ReadResponse Read(hardware::ReadType type) override;

  void Write(base::ControlRequest cr) override;

  void SpecialConfigure(hardware::SpecialConfigureType type) override {
    (void)type;
  }

  void SetLoad(pdcsu::units::nm_t load);
  void SetBatteryVoltage(pdcsu::units::volt_t voltage);

private:
  config::Gains gains_;

  bool brake_mode_ = false;
  bool inverted = false;

  pdcsu::simulation::SimBLDC sim_bldc_;

  base::ControlRequest last_command_ = 0.0;

  std::chrono::steady_clock::time_point last_tick_;
  pdcsu::units::ms_t last_tick_time_{0};

  bool velocity_packet_enabled = true;
  bool position_packet_enabled = true;
};

}  // namespace funkit::control::simulation
