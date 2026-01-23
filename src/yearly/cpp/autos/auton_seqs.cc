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

#define MAX_ACCEL_3PC 25_u_fps2
#define MAX_DECEL_3PC 20_u_fps2
#define MAX_VEL_3PC 15_u_fps

#define MAX_ACCEL_1PC 24_u_fps2
#define MAX_DECEL_1PC 24_u_fps2
#define MAX_VEL_1PC 13_u_fps
#define MAX_ACCEL_1PCS 11_u_fps2
#define MAX_DECEL_1PCS 11_u_fps2
#define MAX_VEL_1PCS 9_u_fps

#define MAX_ACCEL_LEAVE 10_u_fps2
#define MAX_DECEL_LEAVE 10_u_fps2
#define MAX_VEL_LEAVE 5_u_fps

#define MAX_ACCEL_SIMTEST 30_u_fps2
#define MAX_DECEL_SIMTEST 30_u_fps2
#define MAX_VEL_SIMTEST 15_u_fps

#define START_Y (298.5_u_in - 16.5_u_in)

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
      FPT start_point{{x, y}, start_bearing, 0_u_fps};           \
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

#define SOURCELOC_PRE MKPT(34.203_u_in, 69.432_u_in, 53.5_u_deg, 0_u_fps)
#define SOURCELOC MKPT(18_u_in, 49.5_u_in, 53.5_u_deg, 0_u_fps)
#define SOURCELOC_u_inWARDS MKPT(-7.08_u_in, 32.69_u_in, 53.5_u_deg, 0_u_fps)

#define FPC_EXPECTED_START_UF \
  FPT { {50.5_u_in, 271.3_u_in}, 140_u_deg, 0_u_fps }
#define SIM_EXP_START_UF \
  FPT { {20_u_in, 20_u_in}, 0_u_deg, 0_u_fps }

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
  START2(158.5_u_in, START_Y, 180_u_deg), WAIT{0.25_s},
      DRIVE(LEAVE, 158.5_u_in, START_Y - 3_u_ft, 180_u_deg, 0_u_fps),
}
}
{}

__AUTO__(SimTestAuto, "SIMTEST")
SEQUENCE {
  SIM_TEST_START(), DRIVE(SIMTEST, 42_u_in, 51.66_u_in, 0_u_deg, 4_u_fps),
      DRIVE(SIMTEST, 49_u_in, 63.43_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 57_u_in, 70.01_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 65_u_in, 73.54_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 75_u_in, 75_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 85_u_in, 76.45_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 94_u_in, 80.60_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 101_u_in, 86.56_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 107_u_in, 95.82_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 110_u_in, 110_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 108_u_in, 121.66_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 101_u_in, 133.43_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 89_u_in, 142.07_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 75_u_in, 145_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 61_u_in, 142.07_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 49_u_in, 133.43_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 42_u_in, 121.66_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 40_u_in, 110_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 43_u_in, 95.82_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 49_u_in, 86.56_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 56_u_in, 80.60_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 65_u_in, 76.45_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 75_u_in, 75_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 85_u_in, 73.54_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 93_u_in, 70.01_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 101_u_in, 63.43_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 108_u_in, 51.66_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 110_u_in, 40_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 108_u_in, 28.34_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 101_u_in, 16.57_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 93_u_in, 9.99_u_in, 0_u_deg, 6_u_fps),
      DRIVE(SIMTEST, 85_u_in, 6.46_u_in, 0_u_deg, 4_u_fps),
      DRIVE(SIMTEST, 75_u_in, 5_u_in, 0_u_deg, 0_u_fps),
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