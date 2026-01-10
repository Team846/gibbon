// #pragma once

// #include <time.h>
// #include <units/acceleration.h>
// #include <units/angular_velocity.h>
// #include <units/length.h>
// #include <units/mass.h>
// #include <units/velocity.h>

// #include "funkit/math/Differentiator.h"
// #include "funkit/math/Smoother.h"
// #include "funkit/math/calculator.h"
// #include "funkit/math/constants.h"
// #include "funkit/math/vectors.h"

// /*
// AntiTippingCalculator

// Class that provides static methods to aid in preventing the robot from
// tipping.
// */
// class AntiTippingCalculator {
// public:
//   /*
//   SetElevatorHeight()

//   Sets the position of the end effector from the ground. Location of the CG
//   of the elevator is heuristically determined.
//   */
//   static void SetElevatorHeight(units::inch_t height);

//   /*
//   SetElevatorHeight()

//   Sets the position of the end effector from the ground. Location of the CG
//   of the elevator is heuristically determined.
//   */
//   static void SetTelescopeHeight(units::inch_t height);

//   /*
//   CalculateRobotCG()

//   Finds the location of the center of gravity of the robot.
//   */
//   static funkit::math::Vector3D CalculateRobotCG();

//   /*
//   LimitAcceleration()

//   Limits the acceleration of the robot to prevent tipping.
//   */
//   static funkit::math::uVec<units::feet_per_second_squared, 2>
//   LimitAcceleration(funkit::math::uVec<units::feet_per_second, 2>
//   vel_vec,
//       units::degree_t bearing);

// private:
//   static funkit::math::Vector3D elev_cg_position_;
//   static funkit::math::Vector3D tele_cg_position_;

//   static funkit::math::Differentiator elev_df_vel;
//   static funkit::math::Differentiator elev_df_acc;
//   static funkit::math::Smoother elev_acc_smoother;

//   static funkit::math::Differentiator tele_df_vel;
//   static funkit::math::Differentiator tele_df_acc;
//   static funkit::math::Smoother tele_acc_smoother;

//   static units::newton_t tele_force;
//   static units::newton_t elev_force;
// };