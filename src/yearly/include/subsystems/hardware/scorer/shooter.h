#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

#include "funkit/math/RampRateLimiter.h"

struct ShooterReadings {
  fps_t vel;
  bool is_spun_up;
};

struct ShooterTarget {
  fps_t target_vel;
};

class ShooterSubsystem
    : public funkit::robot::GenericSubsystem<ShooterReadings, ShooterTarget> {
public:
  ShooterSubsystem();
  ~ShooterSubsystem();

  void Setup() override;

  ShooterTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

  void StartGraph();

private:
  funkit::control::HigherMotorController esc_1_;
  funkit::control::HigherMotorController esc_2_;

  static constexpr inch_t kWheelRadius{2.0};

  ShooterReadings ReadFromHardware() override;

  void WriteToHardware(ShooterTarget target) override;

  bool filling_graph = false;

  std::array<double, 300> errors_graph{};
  size_t errors_graph_pos = 0U;

  fps_t last_vel_ = 0.0_fps_;
  ms_t last_time_ = -1.0_ms_;

  fps2_t accel_est_ = 0.0_mps2_;

  funkit::math::RampRateLimiter ramp_rate{};
};
