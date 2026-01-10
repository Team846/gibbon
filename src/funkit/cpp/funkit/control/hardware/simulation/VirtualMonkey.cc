#include "funkit/control/hardware/simulation/VirtualMonkey.h"

#include <algorithm>
#include <cmath>

#include "funkit/control/base/motor_specs.h"
#include "pdcsu_units.h"
#include "util/sysdef.h"

namespace funkit::control::simulation {

pdcsu::util::BasePlant VirtualMonkey::ConstructPlant(
    funkit::control::base::MotorSpecs specs,
    pdcsu::units::ohm_t circuit_resistance,
    pdcsu::units::kgm2_t rotational_inertia, double friction) {
  using namespace pdcsu::units;
  using namespace pdcsu::util;

  DefBLDC def_bldc{specs.stall_current, specs.free_current, specs.stall_torque,
      specs.free_speed, volt_t{12.0}};

  BasePlant plant{def_bldc, rotational_inertia,
      nm_t{friction * specs.stall_torque.value()},
      UnitDivision<nm_t, rpm_t>{0.0},
      [](radian_t, radps_t) -> nm_t { return nm_t{0.0}; }, ms_t{20},
      circuit_resistance};

  return plant;
}

VirtualMonkey::VirtualMonkey(funkit::control::base::MotorSpecs specs,
    pdcsu::units::ohm_t circuit_resistance,
    pdcsu::units::kgm2_t rotational_inertia, double friction)
    : sim_bldc_{ConstructPlant(
          specs, circuit_resistance, rotational_inertia, friction)},
      last_tick_(std::chrono::steady_clock::now()) {
  sim_bldc_.SetCurrentLimit(specs.stall_current);
}

void VirtualMonkey::Tick() {
  auto current_time = std::chrono::steady_clock::now();
  auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      current_time - last_tick_);
  last_tick_ = current_time;

  pdcsu::units::ms_t dt{pdcsu::units::ms_t{static_cast<double>(dt_ms.count())}};
  if (dt.value() > 30.0) { dt = pdcsu::units::ms_t{30.0}; }

  double duty_cycle = 0.0;
  if (double* dc = std::get_if<double>(&last_command_)) {
    duty_cycle = *dc;
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&last_command_)) {
    double error = vel->value() - sim_bldc_.getVelocity().value();
    duty_cycle = gains_.kP * error + gains_.kI * 0.0 + gains_.kD * 0.0 +
                 gains_.kF * vel->value();
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&last_command_)) {
    double error = pos->value() - sim_bldc_.getPosition().value();
    duty_cycle = gains_.kP * error + gains_.kI * 0.0 +
                 gains_.kD * sim_bldc_.getVelocity().value() +
                 gains_.kF * sim_bldc_.getCurrent().value();
  }
  duty_cycle = std::clamp(duty_cycle, -1.0, 1.0);

  sim_bldc_.setControlTarget(duty_cycle);
  sim_bldc_.Tick(dt);
}

void VirtualMonkey::SetGenome(config::MotorGenome genome) {
  inverted = false;
  brake_mode_ = genome.brake_mode;
  gains_.kP = genome.gains.kP;
  gains_.kI = genome.gains.kI;
  gains_.kD = genome.gains.kD;
  gains_.kF = genome.gains.kF;
  sim_bldc_.SetCurrentLimit(genome.motor_current_limit);
}

void VirtualMonkey::EnableStatusFrames(config::StatusFrameSelections frames,
    pdcsu::units::ms_t faults_ms, pdcsu::units::ms_t velocity_ms,
    pdcsu::units::ms_t encoder_position_ms,
    pdcsu::units::ms_t analog_position_ms) {
  bool disable_pos = true;
  bool disable_vel = true;
  for (auto frame : frames) {
    switch (frame) {
    case config::StatusFrame::kPositionFrame: disable_pos = false; break;
    case config::StatusFrame::kVelocityFrame: disable_vel = false; break;
    default: break;
    }
  }
  if (disable_pos) position_packet_enabled = false;
  if (disable_vel) velocity_packet_enabled = false;
}

bool VirtualMonkey::IsDuplicateControlMessage(base::ControlRequest cr) {
  if (double* dc = std::get_if<double>(&last_command_)) {
    if (double* cr_dc = std::get_if<double>(&cr)) { return *dc == *cr_dc; }
    return false;
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&last_command_)) {
    if (pdcsu::units::radps_t* cr_vel =
            std::get_if<pdcsu::units::radps_t>(&cr)) {
      return vel->value() == cr_vel->value();
    }
    return false;
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&last_command_)) {
    if (pdcsu::units::radian_t* cr_pos =
            std::get_if<pdcsu::units::radian_t>(&cr)) {
      return pos->value() == cr_pos->value();
    }
    return false;
  }
  return false;
}

hardware::ReadResponse VirtualMonkey::Read(hardware::ReadType type) {
  switch (type) {
  case hardware::ReadType::kReadPosition: {
    pdcsu::units::radian_t pos = sim_bldc_.getPosition();
    if (inverted) pos = pdcsu::units::radian_t{-pos.value()};
    if (!position_packet_enabled) { return 0.0; }
    return pos.value();
  }
  case hardware::ReadType::kReadVelocity: {
    pdcsu::units::radps_t vel = sim_bldc_.getVelocity();
    if (inverted) vel = pdcsu::units::radps_t{-vel.value()};
    if (!velocity_packet_enabled) { return 0.0; }
    return vel.value();
  }
  case hardware::ReadType::kReadCurrent: return sim_bldc_.getCurrent().value();
  case hardware::ReadType::kFwdSwitch: return -1.0;
  case hardware::ReadType::kRevSwitch: return -1.0;
  case hardware::ReadType::kAbsPosition: return 0.0;
  default: return 0.0;
  }
}

void VirtualMonkey::Write(base::ControlRequest cr) {
  if (double* dc = std::get_if<double>(&cr)) {
    double dc_val = *dc;
    if (inverted) dc_val = -dc_val;
    last_command_ = dc_val;
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&cr)) {
    pdcsu::units::radps_t vel_val = *vel;
    if (inverted) vel_val = pdcsu::units::radps_t{-vel_val.value()};
    last_command_ = vel_val;
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&cr)) {
    pdcsu::units::radian_t pos_val = *pos;
    if (inverted) pos_val = pdcsu::units::radian_t{-pos_val.value()};
    last_command_ = pos_val;
  }
}

void VirtualMonkey::SetLoad(pdcsu::units::nm_t load) {
  if (inverted) load = pdcsu::units::nm_t{-load.value()};
  sim_bldc_.SetLoad(load);
}

void VirtualMonkey::SetBatteryVoltage(pdcsu::units::volt_t voltage) {
  (void)voltage;
}

void VirtualMonkey::ZeroEncoder(pdcsu::units::radian_t position) {
  if (inverted) { position = pdcsu::units::radian_t{-position.value()}; }
  // sim_bldc_.SetPosition(position);
  (void)position;  // TODO: Implement this
}

}  // namespace funkit::control::simulation
