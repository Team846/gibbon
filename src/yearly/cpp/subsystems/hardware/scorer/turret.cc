#include "subsystems/hardware/scorer/turret.h"

#include <cmath>
#include <frc/Filesystem.h>
#include <frc/RobotBase.h>

#include "funkit/control/config/genome.h"
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

TurretSubsystem::TurretSubsystem()
    : GenericSubsystem("turret"),
      esc_{base::TALON_FX_KRAKENX60, ports::turret_::kTurretParams},
      cancoder_1_{ports::turret_::kCANCoder1_CANID, ctre::phoenix6::CANBus{""}},
      cancoder_2_{
          ports::turret_::kCANCoder2_CANID, ctre::phoenix6::CANBus{""}} {
  cancoder_1_.OptimizeBusUtilization();
  cancoder_2_.OptimizeBusUtilization();
  cancoder_1_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);
  cancoder_2_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);

  RegisterPreference("icnor/IPG", 0.5);
  RegisterPreference("icnor/friction_nm", 5.0_Nm_);
  RegisterPreference("icnor/agvel_compensation", 2.5);
  RegisterPreference("encoder/offset1", 0.0_rot_);
  RegisterPreference("encoder/offset2", 0.0_rot_);
  RegisterPreference("encoder/min_rots", -3.0_rot_);
  RegisterPreference("encoder/max_rots", 3.0_rot_);
  RegisterPreference("encoder/max_tolerance", 0.025_rot_);
  RegisterPreference("tolerance", 2_deg_);

  RegisterPreference("wrap/positive", 260_deg_);
  RegisterPreference("wrap/negative", -260_deg_);
}

TurretSubsystem::~TurretSubsystem() = default;

void TurretSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 30_A_,
      .smart_current_limit = 20_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  arm_sys_ = std::make_unique<DefArmSys>(
      def_bldc, 1, 58_rot_ / 16_rot_ * 90_rot_ / 10_rot_,
      [](radian_t, radps_t) -> nm_t { return nm_t{0.0}; },
      14_lb_ * 0.13_m_ * 0.13_m_,
      GetPreferenceValue_unit_type<nm_t>("icnor/friction_nm"),
      1.0_Nm_ / 600_radps_, 10_ms_);

  SetupEsc(esc_, genome_backup, *arm_sys_, true);

  icnor_controller_ = std::make_unique<ICNORPositionControl>(*arm_sys_);

  icnor_controller_->setConstraints(radps_t(motor_specs.free_speed * 0.95),
      genome_backup.smart_current_limit);

  icnor_controller_->setTolerance(10_deg_, 20_deg_);

  icnor_controller_->setProjectionHorizon(1);

  icnor_controller_->setDesaturationThresh(25_rad_);

  std::string learner_path =
      frc::filesystem::GetDeployDirectory() + "/ictest.iclearn";
  icnor_controller_->attachLearner(learner_path);
  if (!frc::RobotBase::IsSimulation()) { ZeroWithCRT(); }
}

TurretTarget TurretSubsystem::ZeroTarget() const {
  return TurretTarget{0_deg_, 0_degps_};
}

void TurretSubsystem::ZeroWithCRT() {
  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};

  abs1 = abs1 - GetPreferenceValue_unit_type<rotation_t>("encoder/offset1");
  abs2 = abs2 - GetPreferenceValue_unit_type<rotation_t>("encoder/offset2");

  TurretPositionCalculator::CrtInputs inputs{abs1, abs2, teethA, teethB,
      mainTeeth, GetPreferenceValue_unit_type<rotation_t>("encoder/min_rots"),
      GetPreferenceValue_unit_type<rotation_t>("encoder/max_rots"),
      GetPreferenceValue_unit_type<rotation_t>("encoder/max_tolerance")};

  auto sol = TurretPositionCalculator::GetPosition(inputs);
  esc_.SetPosition(sol.turretRotations);
}

void TurretSubsystem::ZeroEncoders() {
  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};
  SetPreferenceValue("encoder/offset1", abs1);
  SetPreferenceValue("encoder/offset2", abs2);
  Log("Zeroed turret encoders");
}

bool TurretSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Turret esc");
  return ok;
}

TurretReadings TurretSubsystem::ReadFromHardware() {
  if (!arm_sys_) { return TurretReadings{radian_t{0}, radps_t{0}, false}; }

  radian_t pos_real = esc_.GetPosition<radian_t>();
  radps_t vel_real = esc_.GetVelocity<radps_t>();

  TurretReadings readings{pos_real, vel_real, false};

  degree_t error = GetTarget().pos_ - pos_real;
  Graph("error", error);

  readings.in_position_ =
      u_abs(error) < GetPreferenceValue_unit_type<degree_t>("tolerance");

  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};

  abs1 = abs1 - GetPreferenceValue_unit_type<rotation_t>("encoder/offset1");
  abs2 = abs2 - GetPreferenceValue_unit_type<rotation_t>("encoder/offset2");

  TurretPositionCalculator::CrtInputs inputs{abs1, abs2, teethA, teethB,
      mainTeeth, GetPreferenceValue_unit_type<rotation_t>("encoder/min_rots"),
      GetPreferenceValue_unit_type<rotation_t>("encoder/max_rots"),
      GetPreferenceValue_unit_type<rotation_t>("encoder/max_tolerance")};

  auto sol = TurretPositionCalculator::GetPosition(inputs);

  Graph("sol/pos", sol.turretRotations);
  Graph("sol/error", sol.error);
  Graph("sol/valid", sol.isValid);

  return readings;
}

void TurretSubsystem::WriteToHardware(TurretTarget target) {
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
  esc_.ModifyGenome(genome);

  if (!icnor_controller_ || !arm_sys_) { return; }

  degree_t wrap_positive =
      GetPreferenceValue_unit_type<degree_t>("wrap/positive");
  degree_t wrap_negative =
      GetPreferenceValue_unit_type<degree_t>("wrap/negative");
  if (target.pos_ > wrap_positive) {
    double n = std::floor((target.pos_ - wrap_positive).value() / 360.0);
    target.pos_ -= degree_t{360.0 * n};
  }
  if (target.pos_ < wrap_negative) {
    double n = std::floor((wrap_negative - target.pos_).value() / 360.0);
    target.pos_ += degree_t{360.0 * n};
  }

  while (target.pos_ > wrap_positive) {
    target.pos_ -= 360_deg_;
  }
  while (target.pos_ < wrap_negative) {
    target.pos_ += 360_deg_;
  }

  Graph("target/pos", target.pos_);
  Graph("target/vel", target.vel_);

  radian_t current_pos_real = esc_.GetPosition<radian_t>();
  radps_t current_vel_real = esc_.GetVelocity<radps_t>();
  radian_t current_pos_native = arm_sys_->toNative(current_pos_real);
  radps_t current_vel_native = arm_sys_->toNative(current_vel_real);

  radian_t target_pos_native = arm_sys_->toNative(target.pos_);
  radps_t target_vel_native = arm_sys_->toNative(target.vel_);

  target_vel_native =
      GetPreferenceValue_double("icnor/agvel_compensation") * target_vel_native;

  double output = icnor_controller_->getOutput(target_pos_native,
      target_vel_native, current_pos_native, current_vel_native);

  output *= GetPreferenceValue_double("icnor/IPG");

  esc_.WriteDC(output);
}
