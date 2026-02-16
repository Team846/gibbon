#include "subsystems/hardware/hoptake/hoptake_ss.h"

#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

HoptakeSuperstructure::HoptakeSuperstructure()
    : GenericSubsystem("Hoptake_ss"), intake(), pivot() {
  RegisterPreference("init_intake", true);
  RegisterPreference("init_pivot", true);
}

HoptakeSuperstructure::~HoptakeSuperstructure() = default;

void HoptakeSuperstructure::Setup() {
  if (GetPreferenceValue_bool("init_intake")) {
    intake.InitByParent();
    intake.Setup();
  }
  if (GetPreferenceValue_bool("init_pivot")) {
    pivot.InitByParent();
    pivot.Setup();
  }
}

HoptakeSSTarget HoptakeSuperstructure::ZeroTarget() const {
  return HoptakeSSTarget{HoptakeState::kIdle, 0.0_fps_};
}

bool HoptakeSuperstructure::VerifyHardware() {
  return intake.VerifyHardware() && pivot.VerifyHardware();
}

HoptakeSSReadings HoptakeSuperstructure::ReadFromHardware() {
  intake.UpdateReadings();
  pivot.UpdateReadings();
  return HoptakeSSReadings{};
}

void HoptakeSuperstructure::WriteToHardware(HoptakeSSTarget target) {
  IntakeTarget intake_trgt{IntakeState::kIdle, 0.0_fps_};
  PivotTarget pivot_trgt{PivotState::kStow};

  if (target.target_state == HoptakeState::kAgitate) {
    pivot_trgt.target_state = PivotState::kAgitate;
    intake_trgt.target_state = IntakeState::kIntake;
  } else if (target.target_state == HoptakeState::kEvac) {
    pivot_trgt.target_state = PivotState::kIntake;
    intake_trgt.target_state = IntakeState::kEvac;
  } else if (target.target_state == HoptakeState::kIntake) {
    pivot_trgt.target_state = PivotState::kIntake;
    intake_trgt.target_state = IntakeState::kIntake;
  } else {
    pivot_trgt.target_state = PivotState::kStow;
    intake_trgt.target_state = IntakeState::kIdle;
  }

  intake_trgt.dt_vel_ = target.drivetrain_vel;

  intake.SetTarget(intake_trgt);
  pivot.SetTarget(pivot_trgt);

  intake.UpdateHardware();
  pivot.UpdateHardware();
}