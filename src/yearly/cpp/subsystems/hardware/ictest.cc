#include "subsystems/hardware/ictest.h"

#include <frc/Filesystem.h>

#include <cmath>
#include <memory>
#include <string>
#include <variant>

#include "funkit/control/base/motor_specs.h"
#include "funkit/control/config/genome.h"
#include "funkit/robot/calculators/AprilTagCalculator.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;
using namespace pdcsu::control;
using namespace pdcsu::util;

namespace {

void SetupEsc(HigherMotorController& esc, const MotorGenome& genome,
    const std::variant<DefLinearSys, DefArmSys>& plant,
    bool need_pos_vel_frames) {
  esc.Setup(genome, plant);
  if (need_pos_vel_frames) {
    esc.EnableStatusFrames(
        {StatusFrame::kFaultFrame, StatusFrame::kCurrentFrame,
            StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame},
        ms_t{20}, ms_t{5}, ms_t{5}, ms_t{20});
  } else {
    esc.EnableStatusFrames(
        {StatusFrame::kFaultFrame, StatusFrame::kCurrentFrame}, ms_t{20},
        ms_t{5}, ms_t{5}, ms_t{20});
  }
  esc.SetPosition(0_deg_);
}

}  // namespace

ICTestSubsystem::ICTestSubsystem()
    : GenericSubsystem("ictest"),
      esc_1_{base::TALON_FX_KRAKENX60,
          MotorConstructionParameters{
              ports::ictest_::kMotor1_CANID, "", false}},
      cancoder_1_{ports::ictest_::kCANCoder1_CANID, ctre::phoenix6::CANBus{""}},
      cancoder_2_{
          ports::ictest_::kCANCoder2_CANID, ctre::phoenix6::CANBus{""}} {
  RegisterPreference("end_mass_lbs", 0.25);
  RegisterPreference("friction_nm", 0.5_Nm_);
  RegisterPreference("num_motors", 1);
  RegisterPreference("viscous_damping", 0.0);

  RegisterPreference("offset1", 0.0_rot_);
  RegisterPreference("offset2", 0.0_rot_);
  RegisterPreference("max_tolerance", 0.025_rot_);
  RegisterPreference("min_rots", -3.0_rot_);
  RegisterPreference("max_rots", 3.0_rot_);

  cancoder_1_.OptimizeBusUtilization();
  cancoder_2_.OptimizeBusUtilization();
  cancoder_1_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);
  cancoder_2_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);

  RegisterPreference("ipg", 0.1);
}

ICTestSubsystem::~ICTestSubsystem() = default;

void ICTestSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 60_A_,
      .smart_current_limit = 80_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};
  SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::SPARK_FLEX_VORTEX);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  int num_motors = GetPreferenceValue_int("num_motors");

  kg_t mass = pound_t(GetPreferenceValue_double("end_mass_lbs"));
  nm_t friction = nm_t(GetPreferenceValue_double("friction_nm"));
  double viscous_damping_val = GetPreferenceValue_double("viscous_damping");
  UnitDivision<nm_t, rpm_t> viscous_damping =
      UnitDivision<nm_t, rpm_t>(viscous_damping_val);
  ms_t control_period = ms_t(10.0);

  arm_sys_ = std::make_unique<DefArmSys>(
      def_bldc, num_motors, 58_rot_ / 16_rot_ * 90_rot_ / 10_rot_,
      [](radian_t, radps_t) -> nm_t { return nm_t{0.0}; },
      mass * 0.1_m_ * 0.1_m_, friction, viscous_damping, control_period);

  SetupEsc(esc_1_, genome_backup, *arm_sys_, true);

  icnor_controller_ = std::make_unique<ICNORPositionControl>(*arm_sys_);

  icnor_controller_->setConstraints(radps_t(motor_specs.free_speed * 0.95),
      genome_backup.smart_current_limit);

  icnor_controller_->setTolerance(10_deg_, 20_deg_);

  icnor_controller_->setProjectionHorizon(1);

  icnor_controller_->setDesaturationThresh(15_rad_);

  std::string learner_path =
      frc::filesystem::GetDeployDirectory() + "/ictest.iclearn";
  icnor_controller_->attachLearner(learner_path);

  ZeroWithCRT();
}

bool ICTestSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc_1");
  return ok;
}

ICTestTarget ICTestSubsystem::ZeroTarget() const {
  ICTestTarget target;
  target.pos = radian_t{0};
  return target;
}

void ICTestSubsystem::ZeroWithCRT() {
  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};

  abs1 = abs1 - GetPreferenceValue_unit_type<rotation_t>("offset1");
  abs2 = abs2 - GetPreferenceValue_unit_type<rotation_t>("offset2");

  TurretPositionCalculator::CrtInputs inputs{abs1, abs2, teethA, teethB,
      mainTeeth, GetPreferenceValue_unit_type<rotation_t>("min_rots"),
      GetPreferenceValue_unit_type<rotation_t>("max_rots"),
      GetPreferenceValue_unit_type<rotation_t>("max_tolerance")};

  auto sol = TurretPositionCalculator::GetPosition(inputs);
  esc_1_.SetPosition(-sol.turretRotations);
}

void ICTestSubsystem::ZeroEncoders() {
  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};
  SetPreferenceValue("offset1", abs1);
  SetPreferenceValue("offset2", abs2);
  Log("Zeroed turret encoders");
}

ICTestReadings ICTestSubsystem::ReadFromHardware() {
  if (!arm_sys_) { return ICTestReadings{radian_t{0}, radps_t{0}}; }

  radian_t pos_real = esc_1_.GetPosition<radian_t>();
  radps_t vel_real = esc_1_.GetVelocity<radps_t>();

  ICTestReadings readings{pos_real, vel_real};

  funkit::robot::calculators::AprilTagCalculator::turret_angle = -pos_real;
  funkit::robot::calculators::AprilTagCalculator::turret_vel = -vel_real;

  Graph("position", degree_t(readings.pos));
  Graph("velocity", degps_t(readings.vel));

  Graph("caf_x",
      funkit::robot::calculators::AprilTagCalculator::view_turret_off_x);
  Graph("caf_y",
      funkit::robot::calculators::AprilTagCalculator::view_turret_off_y);
  Graph("ca_bearing",
      funkit::robot::calculators::AprilTagCalculator::view_full_turret_angle);

  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};

  abs1 = abs1 - GetPreferenceValue_unit_type<rotation_t>("offset1");
  abs2 = abs2 - GetPreferenceValue_unit_type<rotation_t>("offset2");

  TurretPositionCalculator::CrtInputs inputs{abs1, abs2, teethA, teethB,
      mainTeeth, GetPreferenceValue_unit_type<rotation_t>("min_rots"),
      GetPreferenceValue_unit_type<rotation_t>("max_rots"),
      GetPreferenceValue_unit_type<rotation_t>("max_tolerance")};

  auto sol = TurretPositionCalculator::GetPosition(inputs);

  Graph("abs1", degree_t{abs1});
  Graph("abs2", degree_t{abs2});
  Graph("sol/pos", degree_t{sol.turretRotations});
  Graph("sol/error", degree_t{sol.error});
  Graph("sol/valid", sol.isValid);

  return readings;
}

void ICTestSubsystem::WriteToHardware(ICTestTarget target) {
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
  esc_1_.ModifyGenome(genome);

  if (!icnor_controller_ || !arm_sys_) { return; }

  radian_t current_pos_real = esc_1_.GetPosition<radian_t>();
  radps_t current_vel_real = esc_1_.GetVelocity<radps_t>();
  radian_t current_pos_native = arm_sys_->toNative(current_pos_real);
  radps_t current_vel_native = arm_sys_->toNative(current_vel_real);

  radian_t target_pos_native = arm_sys_->toNative(target.pos);
  Graph("target_pos", target.pos);
  Graph("target_pos_native", target_pos_native);
  radps_t target_vel_native = arm_sys_->toNative(target.vel);
  Graph("current_pos_native", degree_t(current_pos_native));
  Graph("current_vel_native", degps_t(current_vel_native));

  double output = icnor_controller_->getOutput(target_pos_native,
      target_vel_native, current_pos_native, current_vel_native);

  output *= GetPreferenceValue_double("ipg");

  radian_t error_real = target.pos - current_pos_real;
  inpos = u_abs(error_real) < 2_deg_;
  Graph("icerror", error_real);
  Graph("output", output);
  Graph("icnor/has_valid_solution",
      icnor_controller_->hasValidSolution() ? 1.0 : 0.0);

  esc_1_.WriteDC(output);

  Graph("esc_pos", degree_t(esc_1_.GetPosition<radian_t>()));
}
