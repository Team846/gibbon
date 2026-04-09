#include "subsystems/hardware/scorer/turret.h"

#include <frc/Filesystem.h>
#include <frc/RobotBase.h>
#include <funkit/math/collection.h>

#include <cmath>

#include "funkit/control/config/genome.h"
#include "funkit/robot/calculators/AprilTagCalculator.h"
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
      cancoder_1_{
          ports::turret_::kCANCoder1_CANID, ctre::phoenix6::CANBus{"thalamus"}},
      cancoder_2_{ports::turret_::kCANCoder2_CANID,
          ctre::phoenix6::CANBus{"thalamus"}} {
  cancoder_1_.OptimizeBusUtilization();
  cancoder_2_.OptimizeBusUtilization();
  cancoder_1_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);
  cancoder_2_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);

  RegisterPreference("icnor/IPG", 1.0);
  RegisterPreference("icnor/friction_nm", 2.0_Nm_);
  RegisterPreference("icnor/load_nm", 2.5_Nm_);
  RegisterPreference("encoder/offset1", -0.0126953125_rot_);
  RegisterPreference("encoder/offset2", -0.029052734375_rot_);
  RegisterPreference("encoder/min_rots", -3.0_rot_);
  RegisterPreference("encoder/max_rots", 3.0_rot_);
  RegisterPreference("encoder/max_tolerance", 0.012_rot_);
  RegisterPreference("tolerance", 20_deg_);

  RegisterPreference("wrap/positive", 195_deg_);
  RegisterPreference("wrap/negative", -270_deg_);
  RegisterPreference("wrap/project_stop", 0.38_s_);

  RegisterPreference("accel_factor", 5.0);
  RegisterPreference("accel_alpha", 0.3);
}

TurretSubsystem::~TurretSubsystem() = default;

void TurretSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 50_A_,
      .smart_current_limit = 50_A_,
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
      [&](radian_t x, radps_t v) -> nm_t {
        return nm_t{std::tanh((x.value() + radian_t{20_deg_}.value()) * 1.2) *
                    GetPreferenceValue_unit_type<nm_t>("icnor/load_nm")};
      },
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

  esc_.SetControllerSoftLimits(
      arm_sys_->toNative(
          GetPreferenceValue_unit_type<degree_t>("wrap/positive")),
      arm_sys_->toNative(
          GetPreferenceValue_unit_type<degree_t>("wrap/negative")));
}

TurretTarget TurretSubsystem::ZeroTarget() const {
  return TurretTarget{0_deg_, 0_degps_};
}

void TurretSubsystem::ZeroWithCRT(bool retry) {
  if (!is_initialized()) return;

  auto abs1 = rotation_t{cancoder_1_.GetAbsolutePosition().GetValueAsDouble()};
  auto abs2 = rotation_t{cancoder_2_.GetAbsolutePosition().GetValueAsDouble()};

  abs1 = abs1 - GetPreferenceValue_unit_type<rotation_t>("encoder/offset1");
  abs2 = abs2 - GetPreferenceValue_unit_type<rotation_t>("encoder/offset2");

  TurretPositionCalculator::CrtInputs inputs{abs1, abs2, teethA, teethB,
      mainTeeth, GetPreferenceValue_unit_type<rotation_t>("encoder/min_rots"),
      GetPreferenceValue_unit_type<rotation_t>("encoder/max_rots"),
      GetPreferenceValue_unit_type<rotation_t>("encoder/max_tolerance")};

  for (int i = 0; i < (retry ? 5 : 1); i++) {
    auto sol = TurretPositionCalculator::GetPosition(inputs);
    if (sol.isValid) {
      esc_.SetPosition(sol.turretRotations);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  if (retry) { Error("Unable to zero turret with CRT after 5 attempts"); }
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
  if (!arm_sys_) {
    return TurretReadings{radian_t{0}, radps_t{0}, false, false, 0_deg_};
  }

  radian_t pos_real = esc_.GetPosition<radian_t>();
  radps_t vel_real = esc_.GetVelocity<radps_t>();

  TurretReadings readings{pos_real, vel_real, false, false, 0_deg_};

  degree_t error =
      funkit::math::CoterminalDifference(GetTarget().pos_, pos_real);
  Graph("readings/pos", degree_t(pos_real), true);
  Graph("debug/error", error);

  funkit::robot::calculators::AprilTagCalculator::turret_angle = pos_real;
  funkit::robot::calculators::AprilTagCalculator::turret_vel = vel_real;

  readings.error_ = u_abs(error);

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

  radian_t pos_project_wrap =
      pos_real +
      vel_real * GetPreferenceValue_unit_type<second_t>("wrap/project_stop");
  if (pos_project_wrap >=
          GetPreferenceValue_unit_type<degree_t>("wrap/positive") ||
      pos_project_wrap <=
          GetPreferenceValue_unit_type<degree_t>("wrap/negative")) {
    readings.about_to_wrap_ = true;
  } else {
    readings.about_to_wrap_ = false;
  }
  Graph("debug/about_to_wrap", readings.about_to_wrap_);

  return readings;
}

void TurretSubsystem::WriteToHardware(TurretTarget target) {
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
  esc_.ModifyGenome(genome);

  if (GetReadings().pos_ > 400_deg_ || GetReadings().pos_ < -400_deg_) {
    zero_walk_ctr_++;

    if (zero_walk_ctr_ < 30)
      esc_.WriteDC(0.04);
    else
      esc_.WriteDC(-0.04);

    if (zero_walk_ctr_ > 60) zero_walk_ctr_ = 0;

    ZeroWithCRT(false);
    return;
  }

  if (!std::isfinite(target.pos_.value())) {
    esc_.WriteDC(0.0);
    return;
  }

  if (!icnor_controller_ || !arm_sys_) { return; }

  degree_t wrap_positive =
      GetPreferenceValue_unit_type<degree_t>("wrap/positive");
  degree_t wrap_negative =
      GetPreferenceValue_unit_type<degree_t>("wrap/negative");

  degree_t ntarget = wrap_offset_ + target.pos_;

  while (ntarget > wrap_positive) {
    wrap_offset_ -= 360_deg_;
    ntarget = wrap_offset_ + target.pos_;
  }
  while (ntarget < wrap_negative) {
    wrap_offset_ += 360_deg_;
    ntarget = wrap_offset_ + target.pos_;
  }

  target.pos_ = ntarget;

  Graph("target/pos", ntarget);
  Graph("target/vel", target.vel_);

  radian_t target_pos_native = arm_sys_->toNative(target.pos_);
  radps_t target_vel_native = arm_sys_->toNative(target.vel_);
  radian_t current_pos_native = arm_sys_->toNative(GetReadings().pos_);
  radps_t current_vel_native = arm_sys_->toNative(GetReadings().vel_);

  radps2_t accel_inst = 0.0_radps2_;

  if (last_time_ > 0.0_ms_) {
    auto dt = u_max(9.0_ms_, (funkit::wpilib::CurrentFPGATime() - last_time_));
    accel_inst = (target_vel_native - last_vel_) / dt *
                 GetPreferenceValue_double("accel_factor");
  }
  last_vel_ = target_vel_native;
  last_time_ = funkit::wpilib::CurrentFPGATime();

  double accel_alpha = GetPreferenceValue_double("accel_alpha");
  if (accel_alpha < 0.0) { accel_alpha = 0.0; }
  if (accel_alpha > 1.0) { accel_alpha = 1.0; }

  accel_est_ = radps2_t{accel_alpha * accel_inst.to_base() +
                        (1.0 - accel_alpha) * accel_est_.to_base()};

  Graph("debug/accel_inst", accel_inst);
  Graph("debug/accel", accel_est_);

  radian_t native_error = target_pos_native - current_pos_native;
  radps_t avg_vel =
      (current_vel_native + target_vel_native + native_error / 0.04_s_) / 2.0;
  if (u_abs(avg_vel) > 2_radps_) {
    second_t comp_time = u_clamp(native_error / avg_vel, -0.05_s_, 0.05_s_);

    Graph("debug/accel_comp_time", comp_time);

    target_vel_native += comp_time * accel_est_;
    target_pos_native += 0.5 * comp_time * comp_time * accel_est_;
  }

  double output = icnor_controller_->getOutput(target_pos_native,
      target_vel_native, current_pos_native, current_vel_native);

  output *= GetPreferenceValue_double("icnor/IPG");

  esc_.WriteDC(output);
}
