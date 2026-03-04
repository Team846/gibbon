#include <gtest/gtest.h>

#include "calculators/ShootingCalculator.h"

using namespace pdcsu::units;

TEST(ShootingCalculatorTest, RedBlueMirrorKeepsValidityAndSpeedClose) {
  ShootingCalculator::Setup();

  ShootingCalculatorInput red{
      .position = {100_in_, 200_in_},  // just put random numbers here for now
      .velocity = {2.0_fps_, 1.0_fps_},
      .bearing = 20_deg_,
      .yaw_rate = 0_degps_,
      .is_blue_alliance = false};

  ShootingCalculatorInput blue = red;
  blue.position = {funkit::math::FieldPoint::field_size_x - red.position[0],
      funkit::math::FieldPoint::field_size_y - red.position[1]};
  blue.bearing = red.bearing + 180_deg_;
  blue.is_blue_alliance = true;

  ShootingCalculatorOutputs out_red =
      ShootingCalculator::CalculateFromInput(red);
  ShootingCalculatorOutputs out_blue =
      ShootingCalculator::CalculateFromInput(blue);

  EXPECT_EQ(out_red.is_valid, out_blue.is_valid);
  EXPECT_NEAR(out_red.shooter_vel.value(), out_blue.shooter_vel.value(), 0.5);
}