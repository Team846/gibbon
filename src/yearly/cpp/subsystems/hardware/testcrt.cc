#include "subsystems/hardware/testcrt.h"

#include <cmath>

#include "funkit/control/base/motor_specs.h"
#include "funkit/control/config/genome.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_units.h"

using namespace funkit::control;
using namespace funkit::control::config;
using namespace pdcsu::control;
using namespace pdcsu::util;

TurretTestSubsystem::TurretTestSubsystem()
    : funkit::robot::GenericSubsystem<TurretTestReadings, TurretTestTarget>(
          "TurretTestSubsystem"),
      esc_1_{base::SPARK_MAX_NEO550, MotorConstructionParameters{16, "", true}},
      esc_2_{
          base::SPARK_MAX_NEO550, MotorConstructionParameters{21, "", true}} {
  RegisterPreference("offset1", 0.0_u_rot);
  RegisterPreference("offset2", 0.0_u_rot);
  RegisterPreference("min_rots", -0.5_u_rot);
  RegisterPreference("max_rots", 3.0_u_rot);
}

TurretTestSubsystem::~TurretTestSubsystem() = default;

bool TurretTestSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc_1");
  FUNKIT_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify esc_2");
  return ok;
}

void TurretTestSubsystem::Setup() {
  auto genome =
      SubsystemGenomeHelper::LoadGenomePreferences(*this, "TurretTestGenome");

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO550);

  auto voltage_compensation = genome.voltage_compensation;
  DefBLDC def_bldc(amp_t(motor_specs.stall_current.value()),
      amp_t(motor_specs.free_current.value()),
      nm_t(motor_specs.stall_torque.value()),
      rpm_t(motor_specs.free_speed.value()),
      volt_t(voltage_compensation.value()));

  arm_sys_ = std::make_unique<DefArmSys>(
      def_bldc, 1, scalar_t(1.0),
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_u_Nm; },
      0.5_u_kg * 0.0_u_m * 0.0_u_m, 0.0_u_Nm, 1.0_u_Nm / 1_u_radps, 20_u_ms);

  esc_1_.Setup(
      genome, std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys>{
                  *arm_sys_});
  esc_2_.Setup(
      genome, std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys>{
                  *arm_sys_});

  esc_1_.EnableStatusFrames(
      {config::StatusFrame::kFaultFrame, config::StatusFrame::kCurrentFrame,
          config::StatusFrame::kVelocityFrame,
          config::StatusFrame::kAbsoluteFrame},
      ms_t{20}, ms_t{20}, ms_t{20}, ms_t{20});

  esc_2_.EnableStatusFrames(
      {config::StatusFrame::kFaultFrame, config::StatusFrame::kCurrentFrame,
          config::StatusFrame::kVelocityFrame,
          config::StatusFrame::kAbsoluteFrame},
      ms_t{20}, ms_t{20}, ms_t{20}, ms_t{20});
}

void TurretTestSubsystem::ZeroEncoders() {
  auto abs1 = rotation_t{esc_1_.SpecialRead(hardware::ReadType::kAbsPosition)};
  auto abs2 = rotation_t{esc_2_.SpecialRead(hardware::ReadType::kAbsPosition)};
  SetPreferenceValue("offset1", abs1);
  SetPreferenceValue("offset2", abs2);
  Log("Zeroed turret encoders");
}

TurretTestReadings TurretTestSubsystem::ReadFromHardware() {
  auto abs1 = rotation_t{esc_1_.SpecialRead(hardware::ReadType::kAbsPosition)};
  auto abs2 = rotation_t{esc_2_.SpecialRead(hardware::ReadType::kAbsPosition)};

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

  return TurretTestReadings{};
}

void TurretTestSubsystem::WriteToHardware(TurretTestTarget target) {
  esc_1_.WriteDC(0.0);
  esc_2_.WriteDC(0.0);
}

TurretTestTarget TurretTestSubsystem::ZeroTarget() const {
  TurretTestTarget target;
  target.pos = rotation_t{0.0};
  return target;
}