#pragma once

#include <vector>

#include "funkit/control/base/motor_control_base.h"
#include "funkit/control/config/genome.h"
#include "pdcsu_units.h"

namespace funkit::control::hardware {

enum ControllerErrorCodes {
  kAllOK,
  kInvalidCANID,
  kVersionMismatch,
  kHALError,
  kFollowingError,
  kCANDisconnected,
  kCANMessageStale,
  kDeviceDisconnected,
  kConfigFailed,
  kTimeout,
  kWarning,
  kError,
};

enum ReadType {
  kReadPosition,
  kReadVelocity,
  kReadCurrent,
  kFwdSwitch,
  kRevSwitch,
  kAbsPosition,
  kTemperature,
  kRestFault,
};
using ReadResponse = double;

using SpecialConfigureType = base::LimitSwitchDefaultState;

class IntermediateController {
public:
  IntermediateController() = default;
  ~IntermediateController() = default;

  virtual void Tick() = 0;

  virtual void SetSoftLimits(pdcsu::units::radian_t forward_limit,
      pdcsu::units::radian_t reverse_limit) = 0;

  virtual void SetGenome(config::MotorGenome gains, bool force_set = false) = 0;

  virtual void EnableStatusFrames(config::StatusFrameSelections frames,
      pdcsu::units::ms_t faults_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t velocity_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t encoder_position_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t analog_position_ms = pdcsu::units::ms_t{20}) = 0;
  virtual void OverrideStatusFramePeriod(
      config::StatusFrame frame, pdcsu::units::ms_t period) {
    throw std::runtime_error("OverrideStatusFramePeriod not implemented for "
                             "this IntermediateController type.");
  };

  virtual bool IsDuplicateControlMessage(base::ControlRequest cr) = 0;

  // Boolean values are represented as -1.0 or 1.0.
  virtual ReadResponse Read(ReadType type) = 0;

  virtual void Write(base::ControlRequest cr) = 0;

  virtual void SpecialConfigure(SpecialConfigureType type) {
    throw std::runtime_error("Special Configure not implemented for this "
                             "IntermediateController type.");
  };

  virtual ControllerErrorCodes GetLastErrorCode() = 0;

  virtual bool VerifyConnected() = 0;

  virtual void ZeroEncoder(pdcsu::units::radian_t position) = 0;
};

}  // namespace funkit::control::hardware