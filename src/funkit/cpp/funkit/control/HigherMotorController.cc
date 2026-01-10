#include "funkit/control/HigherMotorController.h"

#include "funkit/control/MonkeyMaster.h"
#include "funkit/control/calculators/CurrentTorqueCalculator.h"
#include "pdcsu_units.h"

using namespace pdcsu::units;

// TODO: Check SetLoad() case

namespace funkit::control {

HigherMotorController::HigherMotorController(
    base::MotorMonkeyType mmtype, config::MotorConstructionParameters params)
    : mmtype_(mmtype),
      constr_params_(params),
      intermediate_controller_(nullptr),
      slot_id_(0) {}

void HigherMotorController::Setup(config::MotorGenome genome,
    std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys> plant) {
  genome_ = genome;
  plant_ = plant;

  slot_id_ =
      MonkeyMaster::ConstructController(mmtype_, constr_params_, genome_);

  MonkeyMaster::SetLoad(slot_id_, nm_t{0.0});

  intermediate_controller_ = nullptr;
}

void HigherMotorController::ModifyGenome(config::MotorGenome genome) {
  genome_ = genome;
  MonkeyMaster::SetGenome(slot_id_, genome_);
}

void HigherMotorController::WriteDC(double duty_cycle) {
  MonkeyMaster::WriteDC(slot_id_, duty_cycle);
}

amp_t HigherMotorController::GetCurrent() {
  auto response =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadCurrent);
  return amp_t{response};
}

void HigherMotorController::SetLoad(nm_t load) {
  MonkeyMaster::SetLoad(slot_id_, load);
}

bool HigherMotorController::VerifyConnected() {
  return MonkeyMaster::VerifyConnected(slot_id_);
}

void HigherMotorController::SetControllerSoftLimits(
    radian_t forward_limit, radian_t reverse_limit) {
  MonkeyMaster::SetSoftLimits(slot_id_, forward_limit, reverse_limit);
}

void HigherMotorController::SetSoftLimits(config::SoftLimits soft_limits) {
  soft_limits_ = soft_limits;
}

void HigherMotorController::EnableStatusFrames(
    std::vector<config::StatusFrame> frames, ms_t faults_ms, ms_t velocity_ms,
    ms_t encoder_position_ms, ms_t analog_position_ms) {
  config::StatusFrameSelections selections(frames.begin(), frames.end());
  MonkeyMaster::EnableStatusFrames(slot_id_, selections, faults_ms, velocity_ms,
      encoder_position_ms, analog_position_ms);
}

void HigherMotorController::OverrideStatusFramePeriod(
    config::StatusFrame frame, ms_t period) {
  MonkeyMaster::OverrideStatusFramePeriod(slot_id_, frame, period);
}

void HigherMotorController::SpecialConfiguration(
    control::hardware::SpecialConfigureType type) {
  MonkeyMaster::SpecialConfigure(slot_id_, type);
}

hardware::ReadResponse HigherMotorController::SpecialRead(
    hardware::ReadType type) {
  return MonkeyMaster::Read(slot_id_, type);
}

}  // namespace funkit::control
