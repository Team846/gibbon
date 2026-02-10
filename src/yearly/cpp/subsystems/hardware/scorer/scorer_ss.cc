// #include "subsystems/hardware/scorer/scorer_ss.h"

// #include "ports.h"

// using namespace funkit::control;
// using namespace funkit::control::config;

// ScorerSuperstructure::ScorerSuperstructure()
//     : GenericSubsystem("scorer_ss"), turret(), hood(), shooter() {
//   RegisterPreference("init_turret", true);
//   RegisterPreference("init_hood", true);
//   RegisterPreference("init_shooter", true);
// }

// ScorerSuperstructure::~ScorerSuperstructure() = default;

// void ScorerSuperstructure::Setup() {
//   if (GetPreferenceValue_bool("init_turret")) {
//     turret.Init();
//     turret.Setup();
//   }
//   if (GetPreferenceValue_bool("init_hood")) {
//     hood.Init();
//     hood.Setup();
//   }
//   if (GetPreferenceValue_bool("init_shooter")) {
//     shooter.Init();
//     shooter.Setup();
//   }
// }

// ScorerSSTarget ScorerSuperstructure::ZeroTarget() const {
//   return ScorerSSTarget{};
// }

// bool ScorerSuperstructure::VerifyHardware() {
//   return turret.VerifyHardware() && hood.VerifyHardware() &&
//          shooter.VerifyHardware();
// }

// ScorerSSReadings ScorerSuperstructure::ReadFromHardware() {
//   return ScorerSSReadings{};
// }

// void ScorerSuperstructure::WriteToHardware(ScorerSSTarget target) {}