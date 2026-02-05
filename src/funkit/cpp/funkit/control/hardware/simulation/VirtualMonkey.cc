#include "funkit/control/hardware/simulation/VirtualMonkey.h"

#include <algorithm>
#include <cmath>

#include "funkit/control/base/motor_specs.h"
#include "pdcsu_units.h"
#include "util/sysdef.h"

namespace funkit::control::simulation {

namespace {

constexpr double kDtS = 0.01;
constexpr double kRpmToRadPerS = 2.0 * 3.14159265358979323846 / 60.0;
constexpr double kMinInertiaKgM2 = 1e-9;
constexpr double kMinVelocityTauS = 0.04;
constexpr double kMinFrictionCoeff = 0.02;

}  // namespace

VirtualMonkey::VirtualMonkey(funkit::control::base::MotorSpecs specs,
    pdcsu::units::ohm_t circuit_resistance,
    pdcsu::units::kgm2_t rotational_inertia, double friction)
    : position_rad_{0.0},
      velocity_rad_s_{0.0},
      free_speed_rad_s_{specs.free_speed.value() * kRpmToRadPerS},
      stall_current_A_{specs.stall_current.value()},
      stall_torque_Nm_{specs.stall_torque.value()},
      effective_inertia_kgm2_{
          std::max(rotational_inertia.value(), kMinInertiaKgM2)},
      velocity_tau_s_{},
      max_accel_rad_s2_{100.0} {
  double omega = free_speed_rad_s_;
  double J = effective_inertia_kgm2_;
  double T = stall_torque_Nm_;
  double friction_coeff = (friction > 0.0) ? friction : kMinFrictionCoeff;
  double tau_mech = J * omega / T;
  double tau_friction = J * omega / (friction_coeff * T);
  velocity_tau_s_ = std::max({tau_mech, tau_friction, kMinVelocityTauS});
  (void)circuit_resistance;
}

void VirtualMonkey::Tick() {
  double target_vel = 0.0;
  if (double* dc = std::get_if<double>(&last_command_)) {
    target_vel = (*dc) * free_speed_rad_s_;
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&last_command_)) {
    target_vel = vel->value();
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&last_command_)) {
    double error = pos->value() - position_rad_;
    target_vel = gains_.kP * error;
  }
  target_vel = std::clamp(target_vel, -free_speed_rad_s_, free_speed_rad_s_);

  double alpha = kDtS / (velocity_tau_s_ + kDtS);
  double desired_vel = velocity_rad_s_ + alpha * (target_vel - velocity_rad_s_);
  double max_delta_vel = max_accel_rad_s2_ * kDtS;
  double delta_vel = desired_vel - velocity_rad_s_;
  delta_vel = std::clamp(delta_vel, -max_delta_vel, max_delta_vel);
  velocity_rad_s_ += delta_vel;
  position_rad_ += velocity_rad_s_ * kDtS;
}

void VirtualMonkey::SetGenome(config::MotorGenome genome, bool force_set) {
  inverted = false;
  brake_mode_ = genome.brake_mode;
  gains_.kP = genome.gains.kP;
  gains_.kI = genome.gains.kI;
  gains_.kD = genome.gains.kD;
  gains_.kF = genome.gains.kF;
  double I_limit = genome.motor_current_limit.value();
  if (stall_current_A_ > 0.0 && effective_inertia_kgm2_ > 0.0) {
    double tau_max = (I_limit / stall_current_A_) * stall_torque_Nm_;
    max_accel_rad_s2_ = tau_max / effective_inertia_kgm2_;
  }
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
  (void)faults_ms;
  (void)velocity_ms;
  (void)encoder_position_ms;
  (void)analog_position_ms;
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
    double pos = position_rad_;
    if (inverted) pos = -pos;
    if (!position_packet_enabled) { return 0.0; }
    return pos;
  }
  case hardware::ReadType::kReadVelocity: {
    double vel = velocity_rad_s_;
    if (inverted) vel = -vel;
    if (!velocity_packet_enabled) { return 0.0; }
    return vel;
  }
  case hardware::ReadType::kReadCurrent: return std::abs(velocity_rad_s_) * 0.1;
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

void VirtualMonkey::SetLoad(pdcsu::units::nm_t load) { (void)load; }

void VirtualMonkey::SetBatteryVoltage(pdcsu::units::volt_t voltage) {
  (void)voltage;
}

void VirtualMonkey::ZeroEncoder(pdcsu::units::radian_t position) {
  double pos = position.value();
  if (inverted) pos = -pos;
  position_rad_ = pos;
}

}  // namespace funkit::control::simulation
