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
  return HoptakeSSTarget{};
}

bool HoptakeSuperstructure::VerifyHardware() {
  return intake.VerifyHardware() && pivot.VerifyHardware();
}

HoptakeSSReadings HoptakeSuperstructure::ReadFromHardware() {
  return HoptakeSSReadings{};
}

void HoptakeSuperstructure::WriteToHardware(HoptakeSSTarget target) {
  // if (target.override_state == HoptakeOverrides::kAgitate) {
  // } else if (target.override_state == HoptakeOverrides::kEvac) {
  // } else {
  // }
}