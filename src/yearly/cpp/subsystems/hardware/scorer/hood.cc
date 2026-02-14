#include "subsystems/hardware/scorer/hood.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

// USE ICNOR

HoodSubsystem::HoodSubsystem()
    : GenericSubsystem("hood"),
      esc_{base::SPARK_MAX_NEO550, ports::hood_::kHoodParams} {}

HoodSubsystem::~HoodSubsystem() = default;

void HoodSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 30_A_,
      .smart_current_limit = 20_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO550);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  // TODO: Fix
  DefArmSys hood_plant(
      def_bldc, 1, 1_rot_ / 141_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; }, 0.001044_kgm2_,
      0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_.Setup(genome_backup, hood_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_.SetPosition(radian_t{0});
}

HoodTarget HoodSubsystem::ZeroTarget() const { return HoodTarget{0_deg_}; }

bool HoodSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Hood esc");
  return ok;
}

HoodReadings HoodSubsystem::ReadFromHardware() {
  // esc_.GetPosition()

  return HoodReadings{};
}

void HoodSubsystem::WriteToHardware(HoodTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_.ModifyGenome(genome);
}