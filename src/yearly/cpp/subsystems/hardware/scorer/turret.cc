#include "subsystems/hardware/scorer/turret.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

// TODO: Replace with ICNOR

TurretSubsystem::TurretSubsystem()
    : GenericSubsystem("turret"),
      esc_{base::TALON_FX_KRAKENX60, ports::turret_::kTurretParams} {}

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

  // TODO: Fix
  DefArmSys turret_plant(
      def_bldc, 2, 2_rot_ / 1_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; }, 0.001044_kgm2_,
      0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_.Setup(genome_backup, turret_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_.SetPosition(radian_t{0});
}

TurretTarget TurretSubsystem::ZeroTarget() const {
  return TurretTarget{0_deg_};
}

bool TurretSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Turret esc");
  return ok;
}

TurretReadings TurretSubsystem::ReadFromHardware() {
  // esc_.GetPosition()

  return TurretReadings{};
}

void TurretSubsystem::WriteToHardware(TurretTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_.ModifyGenome(genome);
}