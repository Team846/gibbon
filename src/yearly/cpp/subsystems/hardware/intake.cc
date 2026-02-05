#include "subsystems/hardware/intake.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

IntakeSubsystem::IntakeSubsystem()
    : GenericSubsystem("intake"),
      esc_{base::SPARK_MAX_NEO, ports::intake_::kMotorParams},
      intesc_{base::SPARK_MAX_NEO, ports::intake_::kMotorParams2} {
  RegisterPreference("idle_dc", 0.0);
  RegisterPreference("intake_dc", 0.8);
}

IntakeSubsystem::~IntakeSubsystem() = default;

void IntakeSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 20_A_,
      .smart_current_limit = 30_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};
  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs = base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefArmSys intake_plant(
      def_bldc, 1, 1_u_, [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; },
      0.004_kgm2_, 0.1_Nm_, 0.1_Nm_ / 600_radps_, 20_ms_);

  esc_.Setup(genome_backup, intake_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_.SetPosition(radian_t{0});

  intesc_.Setup(genome_backup, intake_plant);

  intesc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  intesc_.SetPosition(radian_t{0});
}

IntakeTarget IntakeSubsystem::ZeroTarget() const {
  return IntakeTarget{IntakeState::kIdle};
}

bool IntakeSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify esc 1");
  return ok;
}

IntakeReadings IntakeSubsystem::ReadFromHardware() { return IntakeReadings{}; }

void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  genome.voltage_compensation = 12_V_;
  genome.smart_current_limit = 50_A_;
  genome.motor_current_limit = 40_A_;
  esc_.ModifyGenome(genome);

  if (target.state == IntakeState::kIdle) {
    esc_.WriteDC(GetPreferenceValue_double("idle_dc"));
  } else if (target.state == IntakeState::kIntake) {
    esc_.WriteDC(GetPreferenceValue_double("intake_dc"));
  }

  intesc_.WriteDC(target.realintake);
}