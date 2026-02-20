#include "subsystems/hardware/scorer/hood.h"

#include <frc/Filesystem.h>

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
  esc.SetPosition(radian_t{0});
}

}  // namespace

HoodSubsystem::HoodSubsystem()
    : GenericSubsystem("hood"),
      esc_{base::SPARK_MAX_NEO550, ports::hood_::kHoodParams} {
  RegisterPreference("icnor/IPG", 0.5);
  RegisterPreference("icnor/friction_nm", 5.0_Nm_);
  RegisterPreference("encoder/offset", 0.0_rot_);
  RegisterPreference("tolerance", 1.5_deg_);
}

HoodSubsystem::~HoodSubsystem() = default;

void HoodSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 25_A_,
      .smart_current_limit = 30_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO550);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  arm_sys_ = std::make_unique<DefArmSys>(
      def_bldc, 1, 144_rot_ / 1_rot_,
      [](radian_t, radps_t) -> nm_t { return nm_t{0}; },
      3_lb_ * 0.2_m_ * 0.2_m_,
      GetPreferenceValue_unit_type<nm_t>("icnor/friction_nm"),
      1.0_Nm_ / 1200_radps_, 10_ms_);

  SetupEsc(esc_, genome_backup, *arm_sys_, true);

  icnor_controller_ = std::make_unique<ICNORPositionControl>(*arm_sys_);

  icnor_controller_->setConstraints(radps_t(motor_specs.free_speed * 0.95),
      genome_backup.smart_current_limit);

  icnor_controller_->setTolerance(10_deg_, 20_deg_);

  icnor_controller_->setProjectionHorizon(1);

  icnor_controller_->setDesaturationThresh(15_rad_);

  std::string learner_path =
      frc::filesystem::GetDeployDirectory() + "/ictest.iclearn";
  icnor_controller_->attachLearner(learner_path);
  if (!frc::RobotBase::IsSimulation()) { ZeroWithAbsoluteEncoder(); }
  // esc_.SetPosition(degree_t{81});
}

HoodTarget HoodSubsystem::ZeroTarget() const {
  return HoodTarget{60_deg_, 0_degps_};
}

void HoodSubsystem::ZeroWithAbsoluteEncoder() {
  degree_t abs = rotation_t(
      esc_.SpecialRead(funkit::control::hardware::ReadType::kAbsPosition));
  abs = abs - GetPreferenceValue_unit_type<rotation_t>("encoder/offset");
  degree_t norm = abs % 360_deg_;
  // if (norm > 85_deg_ || norm < 40_deg_) {
  //   throw std::runtime_error("Hood absolute encoder out of range");
  // }
  esc_.SetPosition(norm);
}

void HoodSubsystem::ZeroEncoders() {
  auto abs = rotation_t(
      esc_.SpecialRead(funkit::control::hardware::ReadType::kAbsPosition));
  abs = abs - 81_deg_;
  SetPreferenceValue("encoder/offset", abs);
  Log("Zeroed hood encoders");
}

bool HoodSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Hood esc");
  return ok;
}

HoodReadings HoodSubsystem::ReadFromHardware() {
  if (!arm_sys_) { return HoodReadings{radian_t{0}, radps_t{0}, false}; }

  radian_t pos_real = esc_.GetPosition<radian_t>();
  radps_t vel_real = esc_.GetVelocity<radps_t>();

  HoodReadings readings{pos_real, vel_real, false};

  degree_t error = u_clamp(GetTarget().pos_, 50_deg_, 75_deg_) - pos_real;
  Graph("error", error);

  readings.in_position_ =
      u_abs(error) < GetPreferenceValue_unit_type<degree_t>("tolerance");

  auto abs = rotation_t(
      esc_.SpecialRead(funkit::control::hardware::ReadType::kAbsPosition));
  abs = abs - GetPreferenceValue_unit_type<rotation_t>("encoder/offset");
  degree_t norm = degree_t(abs) % 360_deg_;
  Graph("absolute_encoder_pos", norm);

  return readings;
}

void HoodSubsystem::WriteToHardware(HoodTarget target) {
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
  esc_.ModifyGenome(genome);

  target.pos_ = u_clamp(target.pos_, 50_deg_, 75_deg_);

  if (!icnor_controller_ || !arm_sys_) { return; }

  radian_t current_pos_real = esc_.GetPosition<radian_t>();
  radps_t current_vel_real = esc_.GetVelocity<radps_t>();
  radian_t current_pos_native = arm_sys_->toNative(current_pos_real);
  radps_t current_vel_native = arm_sys_->toNative(current_vel_real);

  radian_t target_pos_native = arm_sys_->toNative(target.pos_);
  radps_t target_vel_native = arm_sys_->toNative(target.vel_);

  double output = icnor_controller_->getOutput(target_pos_native,
      target_vel_native, current_pos_native, current_vel_native);

  output = std::clamp(output, -GetPreferenceValue_double("icnor/IPG"),
      GetPreferenceValue_double("icnor/IPG"));

  Graph("output", output);
  Graph("position", degree_t(current_pos_real));
  Graph("target/position", degree_t(target.pos_));

  esc_.WriteDC(output);
}
