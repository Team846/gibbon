#include "subsystems/hardware/ictest.h"

#include <frc/Filesystem.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "funkit/control/base/motor_specs.h"
#include "funkit/control/config/genome.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;
using namespace pdcsu::control;
using namespace pdcsu::util;

ICTestSubsystem::ICTestSubsystem()
    : GenericSubsystem("ictest"),
      esc_1_{base::SPARK_MAX_NEO,
          MotorConstructionParameters{ports::ictest_::kMotor1_CANID, "", true}},
      esc_2_{base::SPARK_MAX_NEO,
          MotorConstructionParameters{ports::ictest_::kMotor2_CANID, "", true}},
      max_vel_mps_(0.0) {
  MotorGenome genome_backup{.motor_current_limit = 40_u_A,
      .smart_current_limit = 30_u_A,
      .voltage_compensation = 12_u_V,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};
  SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  RegisterPreference("end_mass_lbs", 7.0);
  RegisterPreference("circuit_resistance", 0.01_u_ohm);
  RegisterPreference("friction_newtons", 1.0_u_N);
  RegisterPreference("num_motors", 2);
  RegisterPreference("viscous_damping", 0.0);
  RegisterPreference("gear_ratio_in_per_rot", 2.2146);

  RegisterPreference("ipg", 0.2);
}

ICTestSubsystem::~ICTestSubsystem() = default;

void ICTestSubsystem::Setup() {
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");

  auto motor_specs = base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO);

  auto voltage_compensation = genome.voltage_compensation;
  DefBLDC def_bldc(amp_t(motor_specs.stall_current.value()),
      amp_t(motor_specs.free_current.value()),
      nm_t(motor_specs.stall_torque.value()),
      rpm_t(motor_specs.free_speed.value()),
      volt_t(voltage_compensation.value()));

  int num_motors = GetPreferenceValue_int("num_motors");
  double gear_ratio_in_per_rot =
      GetPreferenceValue_double("gear_ratio_in_per_rot");
  meter_t travel_per_rot = inch_t(gear_ratio_in_per_rot);
  UnitDivision<radian_t, meter_t> gear_ratio = 1_u_rot / travel_per_rot;

  kg_t mass = pound_t(GetPreferenceValue_double("end_mass_lbs"));
  mps2_t effective_gravity = 1.0_u_mps2;
  newton_t friction = newton_t(GetPreferenceValue_double("friction_newtons"));
  double viscous_damping_val = GetPreferenceValue_double("viscous_damping");
  UnitDivision<newton_t, rpm_t> viscous_damping =
      UnitDivision<newton_t, rpm_t>(viscous_damping_val);
  ms_t control_period = ms_t(20.0);
  ohm_t circuit_res = GetPreferenceValue_unit_type<ohm_t>("circuit_resistance");

  linear_sys_ = std::make_unique<DefLinearSys>(def_bldc, num_motors, gear_ratio,
      effective_gravity, mass, friction, viscous_damping, control_period,
      circuit_res);

  esc_1_.Setup(
      genome, std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys>{
                  *linear_sys_});
  esc_1_.EnableStatusFrames(
      {config::StatusFrame::kFaultFrame, config::StatusFrame::kCurrentFrame,
          config::StatusFrame::kPositionFrame,
          config::StatusFrame::kVelocityFrame},
      ms_t{20}, ms_t{5}, ms_t{5}, ms_t{20});
  esc_1_.SetPosition(meter_t{0});

  esc_2_.Setup(
      genome, std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys>{
                  *linear_sys_});
  esc_2_.EnableStatusFrames(
      {config::StatusFrame::kFaultFrame, config::StatusFrame::kCurrentFrame},
      ms_t{20}, ms_t{5}, ms_t{5}, ms_t{20});
  esc_2_.SetPosition(meter_t{0});

  icnor_controller_ = std::make_unique<ICNORPositionControl>(*linear_sys_);

  double free_speed_rpm = motor_specs.free_speed.value();
  double free_speed_radps = free_speed_rpm * (2.0 * M_PI / 60.0);
  mps_t max_vel_real = linear_sys_->toReal(radps_t(free_speed_radps * 0.95));

  icnor_controller_->setConstraints(radps_t(free_speed_radps * 0.95),
      amp_t(genome.smart_current_limit.value()));

  icnor_controller_->setTolerance(0.6_u_rad, 1.2_u_rad);

  icnor_controller_->setProjectionHorizon(1);

  icnor_controller_->setDesaturationThresh(9_u_rad);

  max_vel_mps_ = max_vel_real.value();

  std::string learner_path =
      frc::filesystem::GetDeployDirectory() + "/ictest.iclearn";
  icnor_controller_->attachLearner(learner_path);
}

bool ICTestSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc_1");
  FUNKIT_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify esc_2");
  return ok;
}

ICTestTarget ICTestSubsystem::ZeroTarget() const {
  ICTestTarget target;
  target.pos = 0_in;
  return target;
}

ICTestReadings ICTestSubsystem::ReadFromHardware() {
  if (!linear_sys_) { return ICTestReadings{0_in, inches_per_second_t{0}}; }

  meter_t pos_real = esc_1_.GetPosition<meter_t>();
  mps_t vel_real = esc_1_.GetVelocity<mps_t>();

  ICTestReadings readings{units::inch_t(pos_real.value() / 0.0254),
      inches_per_second_t(vel_real.value() / 0.0254)};

  Graph("position", inch_t{readings.pos.value()});
  Graph("velocity", readings.vel.value());

  return readings;
}

void ICTestSubsystem::WriteToHardware(ICTestTarget target) {
  if (!icnor_controller_ || !linear_sys_) { return; }

  meter_t current_pos_real = esc_1_.GetPosition<meter_t>();
  mps_t current_vel_real = esc_1_.GetVelocity<mps_t>();
  radian_t current_pos_native = linear_sys_->toNative(current_pos_real);
  radps_t current_vel_native = linear_sys_->toNative(current_vel_real);

  meter_t target_pos_real = inch_t(target.pos.value());
  radian_t target_pos_native = linear_sys_->toNative(target_pos_real);
  Graph("target_pos", inch_t{target.pos.value()});
  radps_t target_vel_native = 0_u_radps;

  double output = icnor_controller_->getOutput(target_pos_native,
      target_vel_native, current_pos_native, current_vel_native);

  output *= GetPreferenceValue_double("ipg");

  meter_t error_real = target_pos_real - current_pos_real;
  Graph("icerror", inch_t{error_real.value() / 0.0254});
  Graph("output", output);
  Graph("icnor/has_valid_solution",
      icnor_controller_->hasValidSolution() ? 1.0 : 0.0);
  Graph("icnor/tstar", icnor_controller_->getTstar());
  Graph("icnor/zeta", icnor_controller_->getZeta());
  Graph("icnor/alpha", icnor_controller_->getAlpha());
  Graph("icnor/beta", icnor_controller_->getBeta());
  Graph("icnor/gamma", icnor_controller_->getGamma());

  esc_1_.WriteDC(output);
  esc_2_.WriteDC(output);

  Graph("esc_pos", esc_1_.GetPosition<meter_t>().value());
}
