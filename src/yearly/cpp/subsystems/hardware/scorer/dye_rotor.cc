#include "subsystems/hardware/scorer/dye_rotor.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

DyeRotorSubsystem::DyeRotorSubsystem()
    : GenericSubsystem("DyeRotor"),
      esc_{base::TALON_FX_KRAKENX60, ports::dye_rotor_::kDyeRotorParams} {}

DyeRotorSubsystem::~DyeRotorSubsystem() = default;

void DyeRotorSubsystem::Setup() {
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
  DefArmSys DyeRotor_plant(
      def_bldc, 2, 2_rot_ / 1_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; }, 0.001044_kgm2_,
      0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_.Setup(genome_backup, DyeRotor_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_.SetPosition(radian_t{0});
}

DyeRotorTarget DyeRotorSubsystem::ZeroTarget() const {
  return DyeRotorTarget{0_deg_};
}

bool DyeRotorSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Dye Rotor esc");
  return ok;
}

DyeRotorReadings DyeRotorSubsystem::ReadFromHardware() {
  // esc_.GetPosition()

  return DyeRotorReadings{};
}

void DyeRotorSubsystem::WriteToHardware(DyeRotorTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_.ModifyGenome(genome);
}