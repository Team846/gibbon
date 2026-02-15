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
    intake.Init();
    intake.Setup();
  }
  if (GetPreferenceValue_bool("init_pivot")) {
    pivot.Init();
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
  return HoptakeSSReadings{};
}

void HoptakeSuperstructure::WriteToHardware(HoptakeSSTarget target) {
  IntakeTarget intake_trgt{IntakeState::kIdle, 0.0_fps_};
  PivotTarget pivot_trgt{PivotState::kStow};

  if (target.target_state == HoptakeState::kAgitate) {
    if (!agitate_reached_) {
      pivot_trgt.target_state = PivotState::kAgitate;
      if (pivot.GetReadings().in_position_) { agitate_reached_ = true; }
    } else {
      pivot_trgt.target_state = PivotState::kIntake;
      if (pivot.GetReadings().in_position_) { agitate_reached_ = false; }
    }
  } else if (target.target_state == HoptakeState::kEvac) {
    pivot_trgt.target_state = PivotState::kIntake;
  } else if (target.target_state == HoptakeState::kIntake) {
    pivot_trgt.target_state = PivotState::kIntake;
  } else {
    pivot_trgt.target_state = PivotState::kStow;
  }

  intake_trgt.dt_vel_ = target.drivetrain_vel;

  intake.SetTarget(intake_trgt);
  pivot.SetTarget(pivot_trgt);
}