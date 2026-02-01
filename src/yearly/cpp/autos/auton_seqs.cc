#include "autos/auton_seqs.h"

#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "funkit/robot/swerve/drive_to_point_command.h"
#include "funkit/robot/swerve/wait_until_close.h"
#include "pdcsu_units.h"

using namespace pdcsu::units;

/************************
AUTONOMOUS HELPER MACROS

START DEFINE MACROS
*************************/

using INSTANT = frc2::InstantCommand;
using SEQUENCE = frc2::SequentialCommandGroup;
using WAIT = frc2::WaitCommand;

using FPT = funkit::math::FieldPoint;

#define MAX_ACCEL_CS2_NORM 25_fps2_
#define MAX_DECEL_CS2_NORM 20_fps2_
#define MAX_VEL_CS2_NORM 15_fps_

#define MAX_ACCEL_CS2_CURVE 25_fps2_
#define MAX_DECEL_CS2_CURVE 20_fps2_
#define MAX_VEL_CS2_CURVE 15_fps_

#define MAX_ACCEL_CS2_BUMP 35_fps2_
#define MAX_DECEL_CS2_BUMP 35_fps2_
#define MAX_VEL_CS2_BUMP 15_fps_

#define MAX_ACCEL_LEAVE 10_fps2_
#define MAX_DECEL_LEAVE 10_fps2_
#define MAX_VEL_LEAVE 5_fps_

#define MAX_ACCEL_SIMTEST 30_fps2_
#define MAX_DECEL_SIMTEST 30_fps2_
#define MAX_VEL_SIMTEST 15_fps_

#define START_Y (298.5_in_ - 16.5_in_)

#define PARALLEL_DEADLINE(deadline, parallel) \
  frc2::ParallelDeadlineGroup { deadline, parallel }

#define PARALLEL_RACE(action1, action2) \
  frc2::ParallelRaceGroup { action1, action2 }

#define SEQUENCE(action1, action2) \
  frc2::SequentialCommandGroup { action1, action2 }

#define AUTO_NAME(default_name)                                \
  std::string(default_name) + (is_blue_side ? "_B_" : "_R_") + \
      (is_left_side ? "L" : "R")

#define MKPT(x, y, bearing, velocity) \
  FPT{{x, y}, bearing, velocity}.mirror(is_blue_side).mirrorOnlyX(!is_left_side)

#define START2(x, y, start_bearing)                              \
  INSTANT {                                                      \
    [&, blue = is_blue_side, left = is_left_side]() {            \
      FPT start_point{{x, y}, start_bearing, 0_fps_};            \
      start_point = start_point.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.UpdateReadings();                    \
      container.drivetrain_.SetPosition(                         \
          {start_point.point[0], start_point.point[1]});         \
      Log("Auto Start");                                         \
    }                                                            \
  }

#define DRIVE(auto_name, x, y, bearing, final_velocity)                   \
  funkit::robot::swerve::DriveToPointCommand {                            \
    &(container.drivetrain_), MKPT(x, y, bearing, final_velocity),        \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name \
  }

#define DRIVE(auto_name, x, y, bearing, final_velocity)                   \
  funkit::robot::swerve::DriveToPointCommand {                            \
    &(container.drivetrain_), MKPT(x, y, bearing, final_velocity),        \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name \
  }

#define DRIVE_PT(auto_name, pt, pt_name)                                     \
  funkit::robot::swerve::DriveToPointCommand {                               \
    &(container.drivetrain_), pt, MAX_VEL_##auto_name##_##pt_name,           \
        MAX_ACCEL_##auto_name##_##pt_name, MAX_DECEL_##auto_name##_##pt_name \
  }

#define DRIVE_PT_BEARING(auto_name, pt, pt_name)                              \
  funkit::robot::swerve::DriveToPointCommand {                                \
    &(container.drivetrain_), pt, MAX_VEL_##auto_name##_##pt_name,            \
        MAX_ACCEL_##auto_name##_##pt_name, MAX_DECEL_##auto_name##_##pt_name, \
        funkit::robot::swerve::DriveToPointFlags::kRequireBearing             \
  }

#define END_BUMPC1_PT MKPT(90_in_, 223.61_in_, 35_deg_, 0_fps_)
#define START_BUMPC1_PT MKPT(90_in_, 136.61_in_, 35_deg_, 0_fps_)
// 8_fps
#define END_BUMPC23_PT MKPT(102_in_, 223.61_in_, 35_deg_, 0_fps_)
#define START_BUMPC23_PT MKPT(102_in_, 136.61_in_, 35_deg_, 0_fps_)

#define P1C1_INTAKE_PT MKPT(76.5_in_, 287.35_in_, 0_deg_, 11_fps_)
#define P2C1_INTAKE_PT MKPT(87.25_in_, 312.6_in_, 7_deg_, 7_fps_)
#define P3C1_INTAKE_PT MKPT(93.75_in_, 320.2_in_, 15_deg_, 0_fps_)

#define P1C2_INTAKE_PT MKPT(101.1_in_, 254.5_in_, 0_deg_, 11_fps_)
#define P2C2_INTAKE_PT MKPT(108.35_in_, 289.42_in_, 7_deg_, 7_fps_)
#define P3C2_INTAKE_PT MKPT(114.85_in_, 316.35_in_, 15_deg_, 0_fps_)

#define P1C3_INTAKE_PT MKPT(111.1_in_, 254.5_in_, 20_deg_, 11_fps_)
#define P2C3_INTAKE_PT MKPT(118.35_in_, 283.42_in_, 40_deg_, 11_fps_)
#define P3C3_INTAKE_PT MKPT(124.85_in_, 300.65_in_, 60_deg_, 7_fps_)
#define P4C3_INTAKE_PT MKPT(140.85_in_, 310.35_in_, 90_deg_, 0_fps_)

#define FPC_EXPECTED_START_UF \
  FPT { {92.5_in_, 144.54_in_}, 0_deg_, 0_fps_ }
#define SIM_EXP_START_UF \
  FPT { {20_in_, 20_in_}, 0_deg_, 0_fps_ }

#define FPC_SIM_START()                                                      \
  INSTANT {                                                                  \
    [&, blue = is_blue_side, left = is_left_side] {                          \
      if (!frc::RobotBase::IsSimulation()) return;                           \
      FPT exp_start = FPC_EXPECTED_START_UF.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.SetPosition(exp_start.point);                    \
      container.drivetrain_.SetOdomBearing(exp_start.bearing);               \
      Log("FPC Simulated Auto Start");                                       \
    }                                                                        \
  }

#define SIM_TEST_START()                                                \
  INSTANT {                                                             \
    [&, blue = is_blue_side, left = is_left_side] {                     \
      if (!frc::RobotBase::IsSimulation()) return;                      \
      FPT exp_start = SIM_EXP_START_UF.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.SetPosition(exp_start.point);               \
      container.drivetrain_.SetOdomBearing(exp_start.bearing);          \
      Log("Simulated Auto Start");                                      \
    }                                                                   \
  }

#define __AUTO__(codeName, stringName)                                 \
  codeName::codeName(                                                  \
      RobotContainer& container, bool is_blue_side, bool is_left_side) \
      : funkit::robot::GenericCommandGroup<RobotContainer, codeName,   \
            SEQUENCE> {                                                \
    container, AUTO_NAME(stringName),

/***********************
AUTONOMOUS HELPER MACROS

END DEFINE MACROS
************************/

/************************
| ---------------------- |
|  AUTONOMOUS SEQUENCES  |
|                        |
| START DEFINE SEQUENCES |
| ---------------------- |
*************************/

__AUTO__(LeaveAuto, "LEAVE")
SEQUENCE {
  START2(158.5_in_, START_Y, 180_deg_), WAIT{0.25_s},
      DRIVE(LEAVE, 158.5_in_, START_Y - 3_ft_, 180_deg_, 0_fps_),
}
}
{}

__AUTO__(CS2Auto, "CS2")
SEQUENCE {
  START2(92.5_in_, 144.54_in_, 0_deg_),
      DRIVE_PT_BEARING(CS2, END_BUMPC1_PT, BUMP),
      DRIVE_PT(CS2, P1C1_INTAKE_PT, NORM), DRIVE_PT(CS2, P2C1_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P3C1_INTAKE_PT, NORM), DRIVE_PT(CS2, P2C1_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P1C1_INTAKE_PT, NORM),
      // driveuntilfull
      DRIVE_PT_BEARING(CS2, END_BUMPC1_PT, NORM),
      DRIVE_PT_BEARING(CS2, START_BUMPC1_PT, BUMP),
      DRIVE_PT(CS2, END_BUMPC1_PT, NORM), DRIVE_PT(CS2, P1C2_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P2C2_INTAKE_PT, NORM), DRIVE_PT(CS2, P3C2_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P2C2_INTAKE_PT, NORM), DRIVE_PT(CS2, P1C2_INTAKE_PT, NORM),
      // driveuntilfull
      DRIVE_PT_BEARING(CS2, END_BUMPC23_PT, NORM),
      DRIVE_PT_BEARING(CS2, START_BUMPC23_PT, BUMP),
      DRIVE_PT(CS2, END_BUMPC23_PT, BUMP), DRIVE_PT(CS2, P1C3_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P2C3_INTAKE_PT, NORM), DRIVE_PT(CS2, P3C3_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P4C3_INTAKE_PT, NORM), DRIVE_PT(CS2, P3C3_INTAKE_PT, NORM),
      DRIVE_PT(CS2, P2C3_INTAKE_PT, NORM), DRIVE_PT(CS2, P1C3_INTAKE_PT, NORM),
      // driveuntilfull
      DRIVE_PT_BEARING(CS2, END_BUMPC23_PT, NORM),
      DRIVE_PT_BEARING(CS2, START_BUMPC23_PT, BUMP),
      DRIVE_PT_BEARING(CS2, END_BUMPC23_PT, BUMP),
}
}
{}

__AUTO__(SimTestAuto, "SIMTEST")
SEQUENCE {
  SIM_TEST_START(), DRIVE(SIMTEST, 42_in_, 51.66_in_, 0_deg_, 4_fps_),
      DRIVE(SIMTEST, 49_in_, 63.43_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 57_in_, 70.01_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 65_in_, 73.54_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 75_in_, 75_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 85_in_, 76.45_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 94_in_, 80.60_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 101_in_, 86.56_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 107_in_, 95.82_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 110_in_, 110_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 108_in_, 121.66_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 101_in_, 133.43_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 89_in_, 142.07_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 75_in_, 145_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 61_in_, 142.07_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 49_in_, 133.43_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 42_in_, 121.66_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 40_in_, 110_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 43_in_, 95.82_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 49_in_, 86.56_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 56_in_, 80.60_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 65_in_, 76.45_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 75_in_, 75_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 85_in_, 73.54_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 93_in_, 70.01_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 101_in_, 63.43_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 108_in_, 51.66_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 110_in_, 40_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 108_in_, 28.34_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 101_in_, 16.57_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 93_in_, 9.99_in_, 0_deg_, 6_fps_),
      DRIVE(SIMTEST, 85_in_, 6.46_in_, 0_deg_, 4_fps_),
      DRIVE(SIMTEST, 75_in_, 5_in_, 0_deg_, 0_fps_),
}
}
{}

/***********************
| --------------------- |
| AUTONOMOUS SEQUENCES  |
|                       |
| END DEFINE SEQUENCES  |
| --------------------- |
************************/