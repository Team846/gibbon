#pragma once

#include "funkit/base/Loggable.h"
#include "funkit/math/fieldpoints.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "pdcsu_units.h"
#include "ports.h"

struct GPDTarget {};

struct GPDReadings {
  degree_t optimal_pos;
  bool has_target;
  bool locked_target;
};

class GPDSubsystem
    : public funkit::robot::GenericSubsystem<GPDReadings, GPDTarget> {
public:
  GPDSubsystem(funkit::robot::swerve::DrivetrainSubsystem* drivetrain);

  std::pair<funkit::math::Vector2D, bool> getBestGP(
      const std::vector<funkit::math::Vector2D> algae);

  void Setup() override;

  GPDTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("GPDCam1");

  GPDReadings ReadFromHardware() override;

  void WriteToHardware(GPDTarget target) override;

  funkit::robot::swerve::DrivetrainSubsystem* drivetrain_;

  degree_t gp_spin_;

  funkit::math::Vector2D filtered_pos_{};
  bool has_prev_pos_ = false;

  bool locked_target_ = false;
};