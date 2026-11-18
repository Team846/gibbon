# AGENTS.md - Repository Architecture Guide

This document provides a comprehensive overview of the FRC Team 846 codebase architecture, patterns, and structure to help AI agents and developers understand and work with this repository effectively. If graphify-out/ exists, use it.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Architecture Patterns](#architecture-patterns)
3. [Directory Structure](#directory-structure)
4. [Core Components](#core-components)
5. [Subsystem Architecture](#subsystem-architecture)
6. [Command Architecture](#command-architecture)
7. [Motor Control System](#motor-control-system)
8. [Configuration & Preferences](#configuration--preferences)
9. [Logging System](#logging-system)
10. [Build System](#build-system)
11. [Development Workflow](#development-workflow)
12. [Key Concepts](#key-concepts)
13. [Important Notes for AI Agents](#important-notes-for-ai-agents)

---

## Project Overview

This is the **2026 FRC codebase for Team 846 (Gibbon)**, written in **C++** using **WPILib 2026.2.1**. The codebase implements a custom robot architecture built on top of WPILib's command-based framework.

### Key Technologies
- **Language**: C++17/20
- **Framework**: WPILib 2026.2.1
- **Build System**: Gradle (WPILib GradleRIO plugin 2026.2.1)
- **Vendor Libraries**: 
  - Phoenix 6 v26.3.0 (CTRE motor controllers)
  - REVLib 2026.0.4 (REV motor controllers)
  - Studica 2026.0.0
- **Code Quality**: Spotless (clang-format 18.1.8), CppCheck 2.16.0
- **PDCSU**: tr21

### Project Philosophy
- **Subsystem-based architecture**: Each hardware component is a subsystem
- **Command-based control**: Commands set targets for subsystems
- **Preference-driven configuration**: Tuning parameters via NetworkTables/Preferences
- **Dual-loop update system**: Subsystems update in alternating groups for performance
- **Comprehensive logging**: Built-in logging system with hierarchical naming

---

## Architecture Patterns

### 1. Generic Architecture Pattern

The codebase uses a **generic architecture pattern** with base classes that provide common functionality:

- **`GenericRobot`**: Base robot class handling mode transitions, scheduling, and lifecycle
- **`GenericRobotContainer`**: Container managing subsystems and update groups
- **`GenericSubsystem<Readings, Target>`**: Template-based subsystem with typed readings and targets
- **`GenericCommand<RobotContainer, Subclass>`**: Base command class with lifecycle hooks

### 2. Readings/Target Pattern

Every subsystem follows a **readings/target pattern**:
- **Readings**: Immutable state structure read from hardware (`ReadFromHardware()`)
- **Target**: Immutable state structure written to hardware (`WriteToHardware()`)
- Clear separation between sensor input (readings) and actuator output (targets)

### 3. Dual Update Loop System

Subsystems are organized into **Group A** and **Group B** for performance:
- Updates alternate between groups each loop iteration
- Allows processing of twice as many subsystems within the same 10ms period
- Register with `RegisterSubsystemGroupA()`, `RegisterSubsystemGroupB()`, or `RegisterSubsystemGroupAB()`

### 4. Loggable Base Class

Most classes inherit from **`funkit::base::Loggable`** which provides:
- Hierarchical logging with parent/child naming (`parent/child`)
- Preference registration and retrieval
- SmartDashboard graphing capabilities
- Warning/error tracking

---

## Directory Structure

```
Gibbon/
├── src/
│   ├── yearly/                    # Year-specific robot code (2026 season)
│   │   ├── cpp/                  # Implementation files (.cc)
│   │   │   ├── subsystems/
│   │   │   │   ├── abstract/     # Abstract/logic subsystems (control_input, gpd)
│   │   │   │   └── hardware/     # Hardware subsystem implementations
│   │   │   ├── commands/
│   │   │   │   └── teleop/       # Teleoperated commands (drive, intake, shooter)
│   │   │   ├── calculators/      # Utility calculators (ShootingCalculator, TurretPositionCalculator)
│   │   │   ├── control_triggers.cc
│   │   │   └── FunkyRobot.cc     # Main robot class implementation
│   │   ├── include/              # Header files (.h)
│   │   │   ├── subsystems/       # Subsystem headers (robot_container, robot_constants, SubsystemHelper)
│   │   │   ├── commands/         # Command headers
│   │   │   ├── autos/            # Autonomous routine definitions
│   │   │   ├── calculators/      # Calculator headers
│   │   │   ├── control_triggers.h # Teleop trigger binding (ControlTriggerInitializer)
│   │   │   ├── rsighandler.h     # Signal handlers for simulation (configureSignalHandlers)
│   │   │   ├── ports.h           # CAN IDs, PWM ports, etc.
│   │   │   └── FunkyRobot.h      # Main robot class header
│   │   └── resources/            # Non-code resources
│   │       ├── logsclient/       # Log client scripts
│   │       ├── ntbackup/         # NetworkTables backup tools
│   │       └── swervevis/        # Swerve visualization tools
│   ├── funkit/                   # Custom architecture library
│   │   ├── cpp/                  # Implementation files
│   │   └── include/
│   │       └── funkit/
│   │           ├── base/         # Base classes (Loggable, FunkyLogSystem, compression, fserver)
│   │           ├── control/      # Motor control system
│   │           │   ├── base/     # Motor control base (MotorMonkeyType, motor_specs)
│   │           │   ├── calculators/  # Motor calculations (CurrentTorque, CircuitResistance)
│   │           │   ├── config/       # Configuration (genome, soft_limits)
│   │           │   └── hardware/     # Hardware abstractions (Cooked, TalonFX_interm, SparkMXFX_interm, simulation)
│   │           ├── math/         # Math utilities
│   │           ├── robot/        # Robot framework
│   │           │   ├── calculators/  # Robot calculators (e.g., AprilTag)
│   │           │   └── swerve/       # Swerve drive implementation
│   │           └── wpilib/       # WPILib utilities (NTAction, time, win_debug_stubs)
│   ├── pdcsu_tr21/               # PDCSU library (external, auto-downloaded)
│   └── deploy/                   # Autonomous sequence files (e.g. autos/points.lst)
├── build/                        # Build outputs (generated)
├── gradle/                       # Gradle wrapper files
├── vendordeps/                   # Vendor dependency JSON files
├── build.gradle                  # Main build configuration (static deploy from src/yearly/deploy)
└── README.md                     # Setup and usage documentation
```

---

## Core Components

### FunkyRobot (Main Robot Class)

**Location**: `src/yearly/include/FunkyRobot.h`, `src/yearly/cpp/FunkyRobot.cc`

The main robot class inherits from `funkit::robot::GenericRobot` and implements:

```cpp
class FunkyRobot : public funkit::robot::GenericRobot {
  void OnInitialize() override;    // Called once at startup
  void OnEnable() override;        // Called when robot enables
  void OnDisable() override;       // Called when robot disables
  void OnPeriodic() override;      // Called every 10ms (100Hz)
  void InitTeleop() override;      // Called when entering teleop mode
  void InitTest() override;        // Called when entering test mode
};
```

**Key responsibilities**:
- Dashboard button initialization (CANCoder offsets, zero bearing/odometry, zero turret encoders, path recording)
- Path recording control (start/stop, auto-start on enable)
- LED state management via `LEDsLogic` (homing, coasting, normal update)
- Homing/coast/gyro switch handling via `DigitalInput`s
- Teleop default commands: `DriveCommand`, `ShooterCommand`, `IntakeCommand`; trigger binding via `ControlTriggerInitializer::InitTeleopTriggers` (from `control_triggers.h`). Bindings are re-added every time teleop or test starts, so `GenericRobot` clears `CommandScheduler::GetDefaultButtonLoop()` on disable. Without that, they pile up because robot code is not restarted between matches. Clearing a fresh local `frc::EventLoop` does nothing.
- Simulation: `configureSignalHandlers()` from `rsighandler.h` in `main()`; FunkyFMS control mode for DS state
- `ShootingCalculator::Setup` in `OnInitialize`; `ShootingCalculator::Calculate` in `OnReadingsUpdated`, a `GenericRobot` hook that runs after `UpdateReadings` and before the scheduler, so commands use this loop's solution rather than last loop's (its time counts in the `reads` profile phase)
- `ZeroBearing` and `ZeroWithCRT` only retry with sleeps while disabled. While enabled they make one attempt, because a sleep on the main loop leaves the motors on their last command.
- Auto start pose (opt-in per auto): while disabled, when the selected auto changes, `OnPeriodic` sets drivetrain bearing and position to that auto's start point. Only autos that begin with `START2_SET_BEARING` (instead of `START2`) in `auton_seqs.cc` record a start point at construction; `ADD_AUTO_VARIANT` stores it in `auto_start_points_` keyed by chooser name. Autos using plain `START2` get no reset.

### RobotContainer

**Location**: `src/yearly/include/subsystems/robot_container.h`

Container class that holds all subsystems and registers them with update groups:

```cpp
class RobotContainer : public funkit::robot::GenericRobotContainer {
  LEDsSubsystem leds_{};
  DrivetrainConstructor drivetrain_constructor_{};
  funkit::robot::swerve::DrivetrainSubsystem drivetrain_{...};
  GPDSubsystem GPD_{&drivetrain_};
  ControlInputSubsystem control_input_{&drivetrain_};
  TurretTestSubsystem turr_test{};
  ShooterSubsystem shooter_{};
  IntakeSubsystem intake_{};
};
```

**Registration pattern**:
- Use preferences (`init_drivetrain`, `init_leds`, `init_gpd`, `init_shooter`, `init_intake`) to conditionally initialize subsystems
- Register with appropriate group (A, B, or AB): control_input and drivetrain/GPD/shooter/intake use Group AB; leds uses Group A

---

## Subsystem Architecture

### GenericSubsystem Template

**Location**: `src/funkit/include/funkit/robot/GenericSubsystem.h`

All subsystems inherit from `GenericSubsystem<Readings, Target>`:

```cpp
template <class Readings, class Target>
class GenericSubsystem : public frc2::SubsystemBase, public SubsystemBase {
  // Must implement:
  virtual Readings ReadFromHardware() = 0;      // Read sensors
  virtual void WriteToHardware(Target target) = 0;  // Write to actuators
  virtual Target ZeroTarget() const = 0;        // Zero/safe state
  virtual bool VerifyHardware() = 0;            // Hardware verification
  virtual void Setup() = 0;                     // Initial setup
  
  // Provided functionality:
  Readings GetReadings() const;                 // Get latest readings
  void SetTarget(Target target);                // Set target state
  void UpdateReadings();                        // Called by container
  void UpdateHardware();                        // Called by container
};
```

### Subsystem Lifecycle

1. **Construction**: Subsystem object created
2. **Init()**: Called by container registration (registers with WPILib)
3. **Setup()**: One-time hardware configuration
4. **UpdateReadings()**: Read sensors (alternating groups)
5. **UpdateHardware()**: Write actuators (alternating groups)
6. **VerifyHardware()**: Check hardware connections

### Current Subsystems

#### Hardware Subsystems
- **DrivetrainSubsystem**: Swerve drive with 4 modules (built by `DrivetrainConstructor`)
- **LEDsSubsystem**: LED strip control (logic in `LEDsLogic`, `leds_logic.h/.cc`)
- **ShooterSubsystem**: Shooter hardware (`shooter.h/.cc`)
- **IntakeSubsystem**: Intake hardware (`intake.h/.cc`)
- **TurretTestSubsystem**: Turret/motor test subsystem (`testcrt.h/.cc`), optionally registered

#### Abstract Subsystems
- **ControlInputSubsystem**: Processes joystick/gamepad input
- **GPDSubsystem**: Gamepad/Driver input processing

---

## Command Architecture

### GenericCommand Base Class

**Location**: `src/funkit/include/funkit/robot/GenericCommand.h`

Commands inherit from `GenericCommand<RobotContainer, Subclass>`:

```cpp
template <typename RobotContainer, typename Subclass>
class GenericCommand : public frc2::CommandHelper<frc2::Command, Subclass>,
                       public funkit::base::Loggable {
  // Must implement:
  virtual void OnInit() = 0;                    // Called on Initialize()
  virtual void Periodic() = 0;                  // Called every Execute()
  virtual void OnEnd(bool interrupted) = 0;     // Called on End()
  
  // Optional:
  virtual bool IsFinished() override;           // Default: false
};
```

### Command Pattern

1. **Construction**: Command created with reference to `RobotContainer`
2. **Initialize()**: Calls `OnInit()`, records start time
3. **Execute()**: Calls `Periodic()`, tracks timing
4. **End()**: Calls `OnEnd(bool interrupted)`, logs timing statistics
5. **IsFinished()**: Returns true to end command

### Example Command

```cpp
class DriveCommand : public funkit::robot::GenericCommand<RobotContainer, DriveCommand> {
  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;
};
```

**Key commands** (under `commands/teleop/`):
- **DriveCommand**: Teleoperated swerve drive control
- **ShooterCommand**: Shooter control (default command for `ShooterSubsystem`)
- **IntakeCommand**: Intake control (default command for `IntakeSubsystem`)

---

## Motor Control System

### HigherMotorController

**Location**: `src/funkit/include/funkit/control/HigherMotorController.h`

High-level motor controller abstraction that interfaces with `MonkeyMaster`:

```cpp
class HigherMotorController {
  HigherMotorController(base::MotorMonkeyType mmtype, config::MotorConstructionParameters params);
  
  void Setup(config::MotorGenome genome, std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys> plant);
  void ModifyGenome(config::MotorGenome genome);
  
  // Control modes:
  void WriteDC(double duty_cycle);
  void WriteVelocity(mps_t velocity);  void WriteVelocity(radps_t velocity);
  void WritePosition(meter_t position);  void WritePosition(radian_t position);
  void WriteVelocityOnController(mps_t velocity);  void WriteVelocityOnController(radps_t velocity);
  void WritePositionOnController(meter_t position);  void WritePositionOnController(radian_t position);
  void SetPosition(meter_t position);  void SetPosition(radian_t position);  // Zeroes encoder
  
  // Sensor readings (template with specializations):
  template <typename VelUnit> VelUnit GetVelocity();
  template <typename PosUnit> PosUnit GetPosition();
  amp_t GetCurrent();
  
  // Utility:
  void SetLoad(nm_t load);
  bool VerifyConnected();
  void SetControllerSoftLimits(radian_t forward_limit, radian_t reverse_limit);
  void SetSoftLimits(config::SoftLimits soft_limits);
  void EnableStatusFrames(std::vector<config::StatusFrame> frames, ms_t faults_ms, ...);
  void OverrideStatusFramePeriod(config::StatusFrame frame, ms_t period);
  void SpecialConfiguration(hardware::SpecialConfigureType type);
  hardware::ReadResponse SpecialRead(hardware::ReadType type);
};
```

**Key features**:
- **Constructor**: Takes `funkit::control::base::MotorMonkeyType` and `config::MotorConstructionParameters`
- **Plant**: `std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys>`; unit types (e.g. `mps_t`, `radps_t`, `meter_t`, `radian_t`) are PDCSU types
- **Template GetVelocity/GetPosition**: Specializations for `mps_t`/`radps_t` and `meter_t`/`radian_t` dispatch on plant type
- **Soft limits**: `SetControllerSoftLimits` (on-controller) and `SetSoftLimits` (SupremeLimiter/custom)
- **MotorGenome configuration**: All motor parameters (current limits, voltage compensation, PIDF gains, brake mode) in `config::MotorGenome`

### MonkeyMaster

**Location**: `src/funkit/include/funkit/control/MonkeyMaster.h`

Low-level motor controller factory and registry (formerly `MotorMonkey`):
- Manages motor controller instances
- Provides slot IDs for controllers
- Abstracts vendor differences (CTRE Phoenix 6, REV Spark MAX)
- Supports simulation via `VirtualMonkey` (formerly `MCSimulator`)
- Handles CAN bus utilization and power management
- `CheckForResets` runs every 300 loops (outside mode changes) and calls `SetGenome` again for any controller that reports a reset. Slots are numbered in `Setup` order. Slot 2 is the hood Spark MAX, because `ScorerSuperstructure::Setup` runs turret, then hood, shooter and dye rotor. Until 2026-11-17, REV's sticky `hasReset` warning was still set from power-on, so the first check after every boot (12–16 s in) logged "Reset detected on Motor Controller Slot ID 2" and stalled one loop 30–40 ms while disabled. The `SparkMXFX_interm` constructor now calls `ClearFaults()` after its first `Configure`, so only a reboot after construction (for example a brownout) sets the flag and triggers a reconfigure.

### Supported Motor Controllers

- **CTRE TalonFX** (Phoenix 6)
- **CTRE TalonSRX** (Phoenix 6)
- **REV Spark MAX**
- **REV Spark Flex**

### CAN Status Rates

`EnableStatusFrames(frames, faults_ms, velocity_ms, position_ms, analog_ms)` first calls `OptimizeBusUtilization` (TalonFX) and then turns on only the listed signals. Devices on the roboRIO bus (shooter, intake, pivot, dye rotor, hood Spark) send position and velocity every 10 ms, which matches the 100 Hz loop. Don't go back to 5 ms: in Nov 2026, 5 ms plus 400 Hz leader signals put the roboRIO bus at about 55% load with peaks at 100%. `StatusFrame::kLeader` (MotorVoltage and DutyCycle at 100 Hz) is only for a motor that has a follower; the shooter has one, the pivot does not. CANivore devices (swerve, turret) still use 5 ms.

---

## Configuration & Preferences

### Preference System

Preferences are stored in NetworkTables and persist across reboots. All `Loggable` classes can register preferences:

```cpp
// Register preferences (in constructor or Setup())
RegisterPreference("key_name", default_value);        // double, bool, int, string
RegisterPreference("key_name", default_unit_value);   // units::unit_t

// Retrieve preferences (anytime)
GetPreferenceValue_double("key_name");
GetPreferenceValue_bool("key_name");
GetPreferenceValue_int("key_name");
GetPreferenceValue_string("key_name");
GetPreferenceValue_unit_type<units::feet>("key_name");
```

### Preference Naming Convention

Preferences use hierarchical naming based on the `Loggable` name:
- Subsystem `"drivetrain"` with preference `"max_speed"` → `"drivetrain/max_speed"`
- Units are appended automatically: `"max_speed (ft/s)"` for unit types

### MotorGenome and SubsystemGenomeHelper

**Location**: `src/funkit/include/funkit/control/config/genome.h`

Motor configuration is managed through `MotorGenome` and `SubsystemGenomeHelper` (both in `funkit::control::config`):

```cpp
// MotorGenome contains all motor configuration:
struct MotorGenome {
  pdcsu::units::amp_t motor_current_limit;
  pdcsu::units::amp_t smart_current_limit;
  pdcsu::units::volt_t voltage_compensation;
  bool brake_mode;
  Gains gains;  // kP, kI, kD, kF
};

// Create genome preferences in constructor:
MotorGenome genome_backup{...};
SubsystemGenomeHelper::CreateGenomePreferences(*this, "genome", genome_backup);

// Load genome in Setup():
auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
motor_.Setup(genome, plant_variant);
```

**MotorConstructionParameters** contains:
- `can_id`: CAN address
- `bus`: CAN bus name (`std::string_view`, default `""`)
- `inverted`: Motor inversion (default `false`)
- `max_wait_time`: Control message timeout (`pdcsu::units::ms_t`, default 20 ms)

**StatusFrame** enum: `kPositionFrame`, `kVelocityFrame`, `kCurrentFrame`, `kFaultFrame`, `kSensorFrame`, `kAbsoluteFrame`, `kLeader`.

### Ports Configuration

**Location**: `src/yearly/include/ports.h`

All hardware ports defined in a single location (`ports.h` uses `funkit::control::config::MotorConstructionParameters` where applicable):
```cpp
struct ports {
  struct driver_ { static constexpr int kXbox_DSPort = 0; };
  struct operator_ { static constexpr int kXbox_DSPort = 1; };
  struct drivetrain_ {
    static constexpr int kFRDrive_CANID = 2;  // ... steer, CANCoder, PIGEON
  };
  struct leds_ { static constexpr int kLEDStrip1 = 6; };
  struct shooter_ {
    static constexpr MotorConstructionParameters kMotor1Params = {24, "", false};
    static constexpr MotorConstructionParameters kMotor2Params = {25, "", true};
  };
  struct intake_ {
    static constexpr MotorConstructionParameters kMotorParams = {21, "", true};
  };
};
```

**Robot constants** (`src/yearly/include/subsystems/robot_constants.h`): `robot_constants::total_weight`, `robot_constants::base::wheelbase_x`, `robot_constants::base::wheelbase_y`, `robot_constants::base::weight`, `robot_constants::base::height`. Used by `DrivetrainConstructor` for drivetrain geometry and weight.

---

## Logging System

### FunkyLogSystem

**Location**: `src/funkit/include/funkit/base/FunkyLogSystem.h`

Custom logging system with:
- Hierarchical logger names (`parent/child`)
- Multiple log levels: `Log()`, `Warn()`, `Error()`
- Format string support (fmt library)
- Warning/error counters

### Usage

All classes inheriting from `Loggable` can log:

```cpp
Log("Message with value: {}", value);
Warn("Warning message: {}", warning_value);
Error("Error message: {}", error_value);
Graph("key", value);  // Publish to SmartDashboard
```

### On-Robot Data Logs

- `GenericRobot` starts `frc::DataLogManager`, which records every NetworkTables value to a `.wpilog` on the roboRIO (`/u/logs` if a USB drive is present, else `/home/lvuser/logs`).
- When the FMS is connected, non-persist `Graph()` calls skip NetworkTables (field bandwidth) and append straight to the log under the same `NT:/SmartDashboard/...` names, so match and practice logs share keys.
- `src/yearly/resources/pull_logs-mac.sh` (Windows: `pull_logs.bat`) copies robot logs, event subfolders included, into `robot_logs/` (git-ignored). `logs/` holds simulation logs. The mac script runs from any directory and only copies files that are new or whose size changed, so a log that was still being written gets re-copied next time. It also pulls the FunkyLogSystem message logs into `robot_logs/foresting/`, saves the robot prefs to `nt_backups/networktables_MM_DD_HH_MM.json`, and saves the ICNOR `*.iclearn` files to `robot_logs/iclearn/<stamp>/`. Set `ROBOT=lvuser@172.22.11.2` to pull over USB. Copying large logs loads the roboRIO, so run it while disabled.
- `Log`/`Warn`/`Error` messages are queued and written by a background thread to `log_<time>.log846` on the roboRIO. The folder is `/u/foresting` when a writable USB drive is mounted at `/u` (the same rule `DataLogManager` uses for `/u/logs`), otherwise `/home/lvuser/foresting`. If the USB folder can't be created, it prints a warning and falls back instead of throwing. The low-space cleanup checks the free space of the folder actually in use. `pull_logs-mac.sh` pulls both locations into `robot_logs/foresting/`. Each line is `type;sender;content;timestamp;period;period_timestamp`. Any loop over 25 ms logs `Loop overrun: ... | mode reads scheduler periodic writes motors reset_check rest ... | slowest subsystem <name> <ms>`, which says where the time went. The slowest subsystem is timed with `steady_clock` in `GenericRobotContainer` (`SlowestSubsystem()`).
- `src/yearly/resources/mathtests/run_mathcheck-mac.sh` compiles `mathcheck.cc` with the real swerve, odometry, EKF, CRT and current-calculator sources using clang++ (not gradle) and checks them numerically. Run it after touching that math. It prints one known scale quirk rather than failing on it: the open-loop calculator turns exactly 2.000x the commanded rate. `PDCSU=~/Downloads/robotics/PDCSU run_mathcheck-mac.sh` builds against a PDCSU working copy and also checks its `u_sqrt` and `uVec` fixes.
- `src/yearly/resources/wpilog.py LOG` lists entries; `wpilog.py LOG key...` prints `time_s,name,value` CSV. No WPILib install needed. Logs from the USB drive can contain small corrupt chunks: the 2026-11-17 log had 29, many exactly 512 bytes. The old reader stopped at the first one, so it showed only 27 s of a 303 s log. It now skips forward to the next run of 40 valid records and prints how many bytes it skipped to stderr. On clean logs its output is identical to before. Negative timestamps, for values published before the log started, now come out as negative seconds.
- `src/yearly/resources/latency_check.py LOG` fits each camera's position error against pure wheel odometry (`camera{N}/pos` minus `readings/position`) to robot velocity while driving (above 2 ft/s, below 30°/s). It gives every 3 s window its own offset to absorb odometry drift. The slope is the camera's leftover timing error, and the script says whether to change `fudge_latency{N}`, or `synced_capture_delay` when `use_synced_capture` is on. Don't use the EKF as the reference: it is pulled toward the same camera data, mostly cam 4, so it hides errors. Samples are paired by robot loop (read at the next `Robot/loop_time`) because, within one loop, `readings/velocity` is logged before the camera values and `ekf/position` after them. If every camera shows the same error despite different settings, the script warns that the cause is odometry scale. On the 2026-11-13 01:49 log (normal driving, turret not zeroed) the errors were -21/-64/-65 ms with fudges 40/40/0; they shift 5-10 ms with the window length. For a clean measurement, check odometry with a tape measure, zero the turret, and drive back and forth past tags. It reads `use_synced_capture` and the fudges once for the whole log, so if a pref changed mid-log its numbers mix both settings (on 2026-11-17 the pref was switched on at 206 s of a 303 s log).
- Shooting while moving (2026-11-17 log): stationary shots matched the formula, the shooter tracked its target within 1 ft/s, the hood within 1°, and odometry matched camera 4 within 0.4% between stops. Driving straight at the hub at 7 ft/s from 105–150 in, `ShootingCalculator` assumed about 1.0 s of flight, subtracted 80–90 in of lead, and commanded 25–27 ft/s instead of the 32–35 ft/s a stationary shot at that range gets. The lead is drag-free: it assumes the ball keeps all of the robot's speed. `swim/tofGainPhys` scales only that lead (turret aim uses `swim/twistGain`), so changing it does not affect stationary shots. Five later sessions held 1,218 balls; each ball was found by the dip in `shooter/readings/velocity` while `MainField/shoot_point` was open. In every motion group (229 stationary, 700 sideways, 127 toward the hub, 61 away) the shooter was at or above its target at release, and the hood was within 1° of its target. Velocity matches the change in odometry position on both axes, so the lead points the right way, though velocity reads about 4% high. Changing `tofGainPhys` to 0.5 and 2 did not fix the short shots, but those tests ran while the USB drive was out and were not logged, so they could not be checked. The cause of the short moving shots is still open.
- Battery: in 4 of 5 sessions on 2026-11-17, `Robot/ds_battery_voltage` dropped to 6.4–7.7 V while enabled, below the roboRIO 2's default 6.75 V brownout. Shooter speed at release was unaffected even below 10 V.
- `turret/sol/valid` flickers false while the turret moves. That is harmless: the CRT solution is only used to zero the turret (`turret.cc`, `if (sol.isValid)`).

### Logging Hierarchy

Loggers form a tree structure:
- `Robot` (root)
  - `robot_container`
    - `drivetrain`
      - `module_0`
        - `drive`
        - `steer`

---

## Build System

### Gradle Configuration

**Main file**: `build.gradle`

Key features:
- **WPILib GradleRIO plugin**: 2026.2.1
- **Spotless**: Code formatting with clang-format 18.1.8
- **CppCheck**: Static analysis (2.16.0)
- **PDCSU auto-download**: Downloads PDCSU tr21 from the `github.com/prak132/PDCSU` releases (config: `pdcsuGroup = "tr"`, `pdcsuReleaseNumber = "21"`)
- **Multi-platform**: Supports RoboRIO and desktop (simulation)
- **Static file deploy**: `fileTree('src/yearly/deploy')` deployed to `/home/lvuser/deploy`

### Build Targets

- **Build**: `./gradlew build`
- **Deploy**: `./gradlew deploy` (requires robot connection)
- **Simulation**: `./gradlew simulateJava` or `./gradlew simulateNative`
- **Format**: `./gradlew spotlessApply`

### Source Organization

- **C++ sources**: `src/funkit/cpp/**/*.cc`, `src/yearly/cpp/**/*.cc`
- **Headers**: `src/funkit/include`, `src/yearly/include`, `src/pdcsu_tr21`

### Dependencies

Managed via `vendordeps/` JSON files:
- `Phoenix6-26.3.0.json`: CTRE Phoenix 6 v26.3.0 (frcYear: 2026)
- `REVLib.json`: REV Robotics 2026.0.4 (frcYear: 2026)
- `Studica.json`: Studica 2026.0.0
- `WPILibNewCommands.json`: WPILib command framework

---

## Development Workflow

### Creating a New Subsystem

1. **Create header** (`src/yearly/include/subsystems/hardware/my_subsystem.h`):
```cpp
#pragma once

#include "funkit/robot/GenericSubsystem.h"

struct MySubsystemReadings {
  // Sensor readings
};

struct MySubsystemTarget {
  // Actuator targets
};

class MySubsystem : public funkit::robot::GenericSubsystem<
    MySubsystemReadings, MySubsystemTarget> {
 public:
  MySubsystem();
  
  MySubsystemReadings ReadFromHardware() override;
  void WriteToHardware(MySubsystemTarget target) override;
  MySubsystemTarget ZeroTarget() const override;
  bool VerifyHardware() override;
  void Setup() override;
  
 private:
  // Hardware objects
};
```

2. **Create implementation** (`src/yearly/cpp/subsystems/hardware/my_subsystem.cc`):
```cpp
#include "subsystems/hardware/my_subsystem.h"

MySubsystem::MySubsystem() 
    : GenericSubsystem{"my_subsystem"} {
  RegisterPreference("some_param", 1.0);
}

MySubsystemReadings MySubsystem::ReadFromHardware() {
  MySubsystemReadings readings;
  // Read sensors
  return readings;
}

void MySubsystem::WriteToHardware(MySubsystemTarget target) {
  // Write to actuators
}

MySubsystemTarget MySubsystem::ZeroTarget() const {
  return MySubsystemTarget{/* safe state */};
}

bool MySubsystem::VerifyHardware() {
  // Check hardware connections
  return true;
}

void MySubsystem::Setup() {
  // Configure hardware
  // Example with motor controller:
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "motor_genome");
  motor_.Setup(genome, plant_variant);
}
```

3. **Register in RobotContainer**:
```cpp
// In robot_container.h
MySubsystem my_subsystem_{};

// In RobotContainer constructor
RegisterPreference("init_my_subsystem", true);
bool my_subsystem_init = GetPreferenceValue_bool("init_my_subsystem");
RegisterSubsystemGroupA({{&my_subsystem_, my_subsystem_init}});
```

### Creating a New Command

Teleop commands live under `commands/teleop/` (e.g. `drive_command.h`, `intake_command.h`, `shooter_command.h`).

1. **Create header** (`src/yearly/include/commands/my_command.h` or `commands/teleop/my_command.h`):
```cpp
#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class MyCommand : public funkit::robot::GenericCommand<
    RobotContainer, MyCommand> {
 public:
  MyCommand(RobotContainer& container);
  
  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;
};
```

2. **Create implementation** (`src/yearly/cpp/commands/my_command.cc`):
```cpp
#include "commands/my_command.h"

MyCommand::MyCommand(RobotContainer& container)
    : GenericCommand{container, "my_command"} {
  AddRequirements({&container_.my_subsystem_});
}

void MyCommand::OnInit() {
  // Initialize command
}

void MyCommand::Periodic() {
  // Update subsystem target
  MySubsystemTarget target{/* ... */};
  container_.my_subsystem_.SetTarget(target);
}

void MyCommand::OnEnd(bool interrupted) {
  // Cleanup
}

bool MyCommand::IsFinished() {
  return false;  // or condition to finish
}
```

### Adding Preferences

Preferences should be registered in the constructor or `Setup()` method:

```cpp
// In constructor or Setup()
RegisterPreference("param_name", default_value);
RegisterPreference("param_name", default_unit_value);  // for units

// Retrieve anywhere
auto value = GetPreferenceValue_double("param_name");
auto unit_value = GetPreferenceValue_unit_type<units::feet>("param_name");
```

---

## Key Concepts

### Units Library

The codebase uses **PDCSU units** (`pdcsu::units`, from `pdcsu_tr21/util/units.h`) as the primary unit system for type-safe physical quantities. PDCSU units provide compile-time type checking and automatic conversions between compatible unit types. **Keep quantities in unit form as much as possible**; only call `.value()` when necessary (e.g. at API boundaries, logging, or when a raw double is required). Units with the same dimensions (L, M, T, I, R) **auto-convert** via constructors and operators.

#### Why Use PDCSU Units

- **Type safety**: Prevents mixing incompatible units (e.g., can't accidentally add meters to seconds)
- **Automatic conversions**: Units with the same dimensions (L, M, T, I, R) are automatically converted when combined or assigned
- **Self-documenting code**: Makes physical quantities clear in the code
- **Compile-time checking**: Catches unit errors at compile time, not runtime
- **Consistent internal representation**: All internal code uses PDCSU units for consistency

#### Basic Usage

```cpp
pdcsu::units::fps_t speed = pdcsu::units::fps_t{10};
pdcsu::units::degree_t angle = pdcsu::units::degree_t{45};
pdcsu::units::amp_t current = pdcsu::units::amp_t{20};
pdcsu::units::radian_t position = pdcsu::units::radian_t{1.5};
pdcsu::units::inch_t distance = pdcsu::units::inch_t{24};
pdcsu::units::meter_t length = pdcsu::units::meter_t{1.5};
```

**CRITICAL**: Always use **PDCSU unit literals** (e.g. `45_deg_`, `10_fps_`, `1_rad_`, `12_V_`, `20_A_`, `5_in_`, `1.5_m_`) or PDCSU explicit constructors (e.g. `pdcsu::units::degree_t{45}`). **NEVER use WPILib unit literals** (e.g. `45_deg`, `10_fps`, `1_rad`, `12_V`, `20_A`, `5_in`, `1.5_m` without trailing underscore). PDCSU literals end with a trailing underscore (`_`), WPILib literals do not. This is a common mistake—always double-check that your literals have the trailing underscore.

#### Common PDCSU Unit Types

```cpp
// Length
pdcsu::units::inch_t, pdcsu::units::foot_t, pdcsu::units::meter_t

// Angle
pdcsu::units::degree_t, pdcsu::units::radian_t, pdcsu::units::rotation_t

// Velocity
pdcsu::units::fps_t, pdcsu::units::mps_t, pdcsu::units::radps_t, pdcsu::units::degps_t

// Acceleration
pdcsu::units::fps2_t

// Time
pdcsu::units::second_t, pdcsu::units::ms_t, pdcsu::units::minute_t

// Current, Torque, Mass, etc.
pdcsu::units::amp_t, pdcsu::units::nm_t, pdcsu::units::kg_t, pdcsu::units::pound_t
```

#### Unit class: members and free functions

**Members** (`pdcsu::units::Unit`, `pdcsu_tr21/util/units.h`):

| Member | Description |
|--------|-------------|
| `value()` | Returns the quantity in display units (e.g. `foot_t{3}.value()` → 3.0). Use only at API/logging boundaries. |
| `to_base()` | Internal representation in base units; used for same-dimension math. Prefer typed operations over manual base use. |
| `from_base(double)` | Static: build a unit from a base-value. Same type as the unit you call it on (e.g. `foot_t::from_base(x)`). |
| `dims()` | Returns a string of dimension exponents (e.g. `"m"`, `"ft/s"`) for debugging. |

**Arithmetic**: Same-dimension units support `+`, `-`, `+=`, `-=` (types can differ in factor/tag, e.g. `inch_t` and `foot_t`). `*` and `/` with another unit yield a new compound/division type (e.g. `foot_t * second_t` → length×time). Multiplication/division by a plain `double` scale the value and preserve the unit type. `operator%(double)` and `operator%(int)` use `std::fmod` on the numeric value in display units. `operator%(unit)` in tr20 also used each side's display value, so `370_deg_ % 3.14159_rad_` came out wrong; tr21 uses base values. Degrees are defined as exactly 0.01745329 rad (relative error 1.4e-8), so mixing degrees and radians is off by about 5e-5° per turn.

**Comparisons**: `==`, `!=`, `<`, `<=`, `>`, `>=` are defined for same-dimension units.

**Free functions** (in `pdcsu::units`):

| Function | Description |
|----------|-------------|
| `u_abs(u)` | Returns the unit with non-negative numeric value; same type as `u`. |
| `u_sqrt(u)` | Returns the square root as a unit with halved dimension exponents (e.g. `mps2_t` → `mps_t`). **Wrong in tr20 for non-SI inputs** (ft²/s² comes back 3.281x too small); fixed in tr21. Gibbon does not call it: use `std::sqrt` on `.value()` or `.to_base()`. |
| `u_sin(u)`, `u_cos(u)`, `u_tan(u)` | Trig on `radian_t` or `degree_t`; return `double`. |
| `u_asin(x)`, `u_acos(x)`, `u_atan(x)` | Inverse trig on raw `double`; return `radian_t`. |
| `u_atan2(y, x)` | Two-argument atan; `y` and `x` must be same-dimension units; returns `radian_t`. |
| `u_min(a, b)`, `u_max(a, b)` | Same-dimension units; return type of first argument. |
| `u_clamp(u, min, max)` | Clamp `u` to `[min, max]`; all same dimensions. |
| `u_floor(u)`, `u_ceil(u)`, `u_round(u)` | Round to integer value in display units; return same unit type. tr20 rounded the SI base value instead (`u_round(44.6_deg_)` gave 0°); fixed in tr21. |
| `u_pow(u, exp)` | Power with a runtime `double` exponent. tr20 kept the input's unit type, which is dimensionally wrong; tr21 only allows it on dimensionless units (compile error otherwise). |
| `u_pow<Num, Den>(u)` | tr21 and later: power with a compile-time exponent (`u_pow<2>(x)`, `u_pow<1, 3>(x)`); dimensions are multiplied by `Num/Den`. |
| `u_copysign(u, sign)` | Copy sign from `sign` (unit or `double`) onto `u`; return same type as `u`. |

#### When to use `.value()`

Use `.value()` **only when necessary**: e.g. passing to APIs that take `double`, logging, or serialization. **Prefer keeping values in unit form** for type safety and so the type system can catch dimension errors. Let units auto-convert by combining and assigning typed values rather than extracting scalars early.

```cpp
// Prefer: keep in units, let types convert
pdcsu::units::fps_t v = 10_fps_;
pdcsu::units::inch_t x = v * 1_s_;  // same dimension as length: fine

// Only at boundaries: extract when the API requires double
double raw = v.value();
units::feet_per_second_t wpi_v{raw};
```

#### Units in Preferences

Preferences support both WPILib and PDCSU unit types. The unit abbreviation is automatically appended to the key name:

```cpp
// Register with PDCSU unit type
RegisterPreference("max_speed", pdcsu::units::fps_t{15});
// Creates preference key: "drivetrain/max_speed (ft/s)"

// Retrieve with PDCSU unit type
auto max_speed = GetPreferenceValue_unit_type<pdcsu::units::fps_t>("max_speed");
```

#### Radians and arc-length (r·θ = s)

Arc-length is **s = r·θ**. In PDCSU, angle is a distinct dimension (R). So:

- **r·θ → length**: If `r` is `foot_t` and `θ` is `radian_t`, then `r * θ` has type (length × angle), **not** pure length. To obtain a length from `r * θ`, **divide by 1_rad_** (or by a `radian_t`): e.g. `(r * theta) / 1_rad_` gives a length.
- **θ = s/r**: The quotient `s / r` is dimensionless. To get an **angle** (e.g. `radian_t`), **multiply by 1_rad_** (or otherwise express the result as `radian_t`): e.g. `(s / r) * 1_rad_` or `radian_t{(s / r).value()}` if you must break out.

This matches how PDCSU's `DefLinearSys` and related code use `1_rad_` when combining length and angle (see `pdcsu_tr21/util/sysdef.h`).

#### Vector Types and uVec class

The codebase uses **PDCSU vectors** (`pdcsu::util::math::uVec<UT, N>` from `pdcsu_tr21/util/math/uvec.h`) instead of WPILib vectors. Vectors hold unit-typed elements and keep type safety. Common aliases: `Vector1D`, `Vector2D`, `Vector3D` are `uVec<pdcsu::units::inch_t, 1>`, `uVec<pdcsu::units::inch_t, 2>`, `uVec<pdcsu::units::inch_t, 3>`.

**uVec constructors**:

| Constructor | Description |
|-------------|-------------|
| `uVec()` | Default: N elements of zero (same unit type UT). |
| `uVec({a, b, ...})` | From initializer list of N values of type UT. |
| `uVec(magnitude, theta, angleIsBearing)` | **2D only.** Polar: `magnitude` (UT), `theta` (`degree_t`). If `angleIsBearing == true`, 0° is +y and angle is clockwise; else 0° is +x and counter-clockwise. |
| `uVec(std::pair<T,T>)` | **2D only.** From a pair of UT values. |
| `uVec(other)` | Copy constructor. |

**uVec arithmetic and access**:

| Operation | Description |
|-----------|-------------|
| `a + b`, `a - b` | Element-wise; same `UT` and `N`. |
| `v * scalar`, `v / scalar` | Scale each element; scalar is `double`. |
| `+=`, `-=`, `*=`, `/=` | In-place; right-hand side as above. |
| `v[i]` | Element access; `i < N`; returns `UT&` or `const UT&`. |
| `v == w` | Equality with tolerance 1e-9 on base values. |

**uVec methods** (return types and constraints):

| Method | Returns | Notes |
|--------|---------|--------|
| `magnitude()` | `UT` | Euclidean norm; 2D or 3D. |
| `unit()` | `uVec<UT,N>` | Normalized direction. tr20 divided by `magnitude().to_base()`, so the length was wrong for non-SI units; fixed in tr21. |
| `rotate(angle, clockwise=true)` | `uVec<UT,N>` | **2D only.** Returns new vector; default is clockwise. Does not modify `*this`. |
| `dot(other)` | compound type | Dot product; `other` can be `uVec<UT2,N>`; result has units UT×UT2. |
| `cross(other)` | `uVec<ResultType,3>` | **3D only.** Cross product; result type from `data[i]*other[j]`. |
| `angle(angleIsBearing=false)` | `degree_t` | **2D only.** Angle of vector. If `angleIsBearing`, 0° is +y, clockwise. |
| `angleBetween(other, angleIsBearing=false)` | `degree_t` | **2D.** Angle from this to `other` (signed). |
| `angleAimTowards(other, angleIsBearing=false)` | `degree_t` | **2D.** Angle of `(other - *this)`; direction to aim from this toward other. |
| `projectOntoAnother(other)` | `uVec<UT,N>` | **2D.** Project this onto `other`; returns component of this along other. |
| `projectOntoThis(other)` | `uVec<UT2,N>` | **2D.** Project `other` onto this; returns component of other along this. |
| `AddToMagnitude(delta)` | `uVec<UT,N>` | **2D.** New vector with same direction, magnitude = current magnitude + `delta` (UT). |
| `resize(magnitude)` | `uVec<UT,N>` | **2D.** New vector with same direction, given magnitude (UT). |
| `toPair()` | `std::pair<T,T>` | **2D only.** |
| `toString()` | `std::string` | Format `"<v0, v1, ...>"` using `.value()` on each element. |

**Example usage**:

```cpp
using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;
Vector2D position{10_in_, 20_in_};
Vector2D from_polar(mag_inch, 45_deg_, false);  // magnitude (inch_t) + angle
auto mag = position.magnitude();                // inch_t
auto ang = position.angle(true);                // degree_t, bearing from +y
auto turned = position.rotate(90_deg_);         // new vector, clockwise
auto d = position.dot(other);                   // unit type from UT*UT
auto aim = pos.angleAimTowards(target, true);   // angle to turn toward target
Vector2D along = v.projectOntoAnother(direction);
Vector2D scaled = v.resize(12_in_);             // same direction, length 12 in
```

#### Conversion at Boundaries

**Important**: Convert to WPILib units only at boundaries where interaction with WPILib APIs or hardware is necessary:

- **MonkeyMaster**: Motor controller interface (converts internally in `HigherMotorController` using plants)
- **Field2d**: WPILib field visualization (convert for `SetRobotPose`, etc.)

```cpp
// Internal code uses PDCSU units
pdcsu::units::fps_t velocity = pdcsu::units::fps_t{10};

// Convert only at boundary
units::feet_per_second_t wpi_velocity{velocity.value()};

// Convert back from WPILib
pdcsu::units::fps_t pdcsu_velocity{wpi_velocity.to<double>()};
```

#### Best Practices

1. **Always prefer PDCSU units**: Use PDCSU unit types internally throughout the codebase; keep expressions in unit form.
2. **Convert at boundaries**: Only convert to WPILib units (or call `.value()`) when interfacing with WPILib or other APIs that need raw numbers.
3. **Use literals or explicit constructors**: e.g. `90_deg_` or `pdcsu::units::degree_t{90}`; stay in units rather than extracting early.
4. **Extract only when necessary**: Use `.value()` only when an API, logger, or serializer requires a `double`.
5. **Radians vs length**: For arc-length `s = r·θ`, get length by `(r * theta) / 1_rad_`; for angle from `s/r`, get `radian_t` by multiplying by `1_rad_` or constructing `radian_t` from the dimensionless quotient when appropriate.
6. **Type safety is your friend**: If code doesn't compile due to unit/dimension mismatch, it's preventing a bug.

### Swerve Drive

The swerve drive implementation (`funkit::robot::swerve`) includes:
- **4 swerve modules**: Each with drive and steer motors
- **Odometry**: Position tracking via module encoders and gyro. Each wheel's travel is applied along the steer angle halfway between the previous and current reading, and the result is rotated by the bearing halfway through the loop (`bearing - dtheta / 2`). Using the end-of-loop steer angle put a sideways error of about `sin(yaw_rate * 5 ms)` of the distance into field-relative driving while spinning (1.4% at 180°/s in the sim; up to 2.5 in per 2 s on the 2026-11-13 log).
- **Pose estimation**: Kalman filter fusion with AprilTag vision
- **PoseEKF** (`odometry/pose_ekf.h`): EKF over `[x, y, heading offset]` that runs every loop alongside the linear `PoseEstimator`. Odometry is the predict step; each tag's raw `(distance, tx)` is a separate correction, with a chi² gate that releases after `ekf/gate_release_count` rejections in a row. The rejection counter goes up by one per gated-out tag and down by one per accepted tag, so the gate also releases when most tags are rejected but a few still pass. Resetting it on every accepted tag left the filter stuck for 20 s in a sim that started 14 in and 3° off. `pose_estimator/use_ekf` (default false) makes it drive `estimated_pose.position`, `variance`, and `bearing_correction_at_` (the tag heading correction). Velocity always comes from `PoseEstimator`. `ekf/*` graphs are published in both modes. `AprilTagCalculator` history stores gyro-only bearing (`pose.bearing - heading_offset`), and its `observations` output carries the per-tag data the EKF uses. `ZeroBearing` sets the EKF heading offset to 0, so pressing zero bearing discards the tag heading. On 2026-11-17 it reset correct offsets of 77° and 92° (the robot sat disabled at that angle) to 0, and in another session the heading was up to 24° off while enabled until tags pulled it back. With `use_ekf` on, only zero bearing when the robot is square to the field.
- **Camera time sync (UDP 5809)**: NT4 measures the client clock offset only once per connection, so MonkeySee's `result[0]` (capture time in robot time) drifts. `funkit::wpilib::TimeSyncServer` (started in `GenericRobot`) answers pings with the FPGA clock. A ping is big-endian `[u16 0x8461][u64 echo]`; the reply is `[u16 0x8461][u64 fpga_us][u64 echo]`. This is the same format as the reverted April `freciver` (`~/Downloads/robotics/UDPstuff`). MonkeySee's `pipeline/timesync.py` pings at 10 Hz and keeps the offset from the lowest-RTT sample in a 5 s window. It resets that window when the offset jumps more than 100 ms, which happens after a robot code restart. MonkeySee then fills `result[0]` with the synced robot time and publishes `AprilTagsCam{N}/sync_rtt_ms` (-1 when not synced). `AprilTagCalculator` uses `result[0] - april_tags/synced_capture_delay` as the capture time only when all of these hold: `april_tags/use_synced_capture` is true (default false), that camera's `sync_rtt_ms` is between 0 and 50, and the frame is 0–250 ms old on arrival. Otherwise it uses `arrival - result[1] - fudge_latency{N}`. Per camera it graphs `april_tags/camera{N}/latency_ms` (the latency it used), `synced_latency_ms` (measured), `sync_rtt_ms`, and `used_synced`. Rollout: deploy both sides with the pref off, compare `latency_ms` and `synced_latency_ms`, then turn the pref on. `synced_capture_delay` covers only the camera's exposure before the driver timestamp. On 2026-11-17 (pref on in all five sessions), the sync round trip was 0.6–0.7 ms and measured latency a steady 27–29 ms. `latency_check.py` put every camera 35–60 ms later than that, so about 45 ms is the suggested `synced_capture_delay`. MonkeySee logged 1–3 "Camera disconnect" errors per session.
- **Open-loop control**: Velocity-based control for teleop. Modules drive by duty cycle (`target / max_speed`, times the cosine of steer error). `SwerveDrivetrain/drive_vel_kP` (default 0, off) adds per-module wheel-speed feedback, `kP * (target - measured)` in duty cycle per ft/s, clamped to ±0.15 and applied only when the target is above 0.5 ft/s. It is graphed as `<module>/target/drive_vel_fb_dc`. It exists because modules given the same duty cycle ran 3–5% apart (right side slower, Nov 2026), which made the robot drift while spinning in place.
- **Shooting gains**: `ShootingCalculator::GetBaseVelocity` now returns the real drag-free speed in ft/s. Until 2026-11-16 it went through `u_sqrt` and returned the m/s number. The two prefs that had absorbed that factor were renamed so a stale stored value cannot be used with the new code: `2ptvel/kAdditive` 4.0 → `2ptvel/kAdditivePhys` 1.2192 (×0.3048) and `swim/tofGain` 0.35 → `swim/tofGainPhys` 1.1483 (÷0.3048). Shooter targets and time of flight are unchanged.
- **Velocity**: use `pose.velocity` (wheel average), not `estimated_pose.velocity`. On the 2026-11-13 log the filter velocity lagged the wheels by about 80 ms, was off by 1.4 ft/s (median) while accelerating, and spiked while parked.
- **Path following**: Commands for driving to points and locking. The final velocity in `MKPT(...)` is ignored (`Initialize` zeros it unless `kLockToPoint`), and with `drive_to_point/kE` = 7 the slow-down only starts 1–3 in from the point, so pass-through speed is set by each segment's `MAX_VEL_*`. `DriveToPointCommand::Initialize` runs `Execute()` once. Otherwise the previous point's `End` (which zeros the target) would leave the drivetrain at 0 ft/s for one loop at every pass-through point. "Overshot" means the robot has moved past the target along the start→target line, so sideways drift doesn't end the command early. Teleop translation applies the deadband and exponent to the stick's magnitude and keeps its direction.

### Update Groups

Subsystems are split into two update groups for performance:
- **Group A**: Updates on even loop iterations
- **Group B**: Updates on odd loop iterations
- **Group AB**: Updates every iteration (use sparingly)

This allows 200Hz effective update rate for subsystems while maintaining 100Hz robot loop.

### Hardware Verification

Subsystems implement `VerifyHardware()` to check connections:
- Motor controller communication
- Sensor presence
- CAN bus connectivity
- Can be triggered via dashboard button

### Path Recording

The drivetrain includes path recording functionality:
- Records odometry data during autonomous
- Saves to CSV files in `/home/lvuser/deploy`
- Can be used for replay or analysis
- Controlled via dashboard buttons

---

## Important Notes for AI Agents

### Code Modification Rules

1. **No comments unless requested**: Under no conditions should you add comments to code files unless specifically prompted to do so by the user. However, do not remove existing comments that remain relevant.

2. **Documentation file policy**: Do not create or update any `.md` files unless explicitly prompted by the user, except for `AGENTS.md` which should be maintained and updated as needed.

3. **Keep AGENTS.md updated**: Whenever the repository structure changes significantly (new subsystems, major architecture changes, new patterns, etc.), you must update `AGENTS.md` to reflect these changes. **Additionally, if you repeatedly struggle with the same issue or mistake, document it in AGENTS.md to help future sessions avoid the same problem.**

4. **Thorough research required**: When working on code, search through documentation and examine architecture folders (`src/funkit/include/funkit/`, `src/funkit/cpp/`, etc.) thoroughly to understand existing patterns and implementations before making changes.

5. **Performance critical**: Code must run very quickly and in real time. The robot loop runs at 100Hz (every 10ms), and subsystems update at 200Hz effective rate. Avoid expensive operations, unnecessary allocations, or blocking calls in hot paths.

6. **Incremental changes**: Always make small, careful, systematic, and targeted changes. Avoid large refactors unless explicitly requested. Test and verify each change before moving to the next.

7. **Systematic process**: Whenever a user provides you with a screenshot, a graph, portions of terminal, or a pasted entry, you must first analyze it carefully. Explain to the user what issues you have found and how you plan to resolve it.

8. **Never auto-run builds**: You must NEVER automatically run `./gradlew build`, `./gradlew deploy`, or any other build/compilation commands. Always suggest that the user runs these commands, but never execute them yourself. The user needs to control when builds happen.

### Loggable NT Caching (Performance)

`Loggable::Graph(key, double)` and `Loggable::GetPreferenceValue_double(key)` cache an `nt::DoublePublisher` / `nt::DoubleEntry` per short key on first call (lazy init) and reuse the handle on subsequent calls. Hot path is `unordered_map::find(string_view) + pub.Set(value) / entry.Get()`. Pre-format dynamic keys once instead of building them inside hot loops, otherwise the per-call `fmt::format` defeats the cache. `GetPreferenceValue_bool` / `_int` are cached the same way (`nt::BooleanEntry` / `nt::IntegerEntry`), because every loop reloads 8 motor genomes and each reload reads `brake_mode`. Int `Graph` goes through the cached double overload (same NT type as the old `PutNumber`), and bool `Graph` caches an `nt::BooleanPublisher`. `GetPreferenceValue_string`, string `Graph`, and `SetPreferenceValue` are still uncached. Unit-typed `Graph`/pref calls get their unit string from `Loggable::UnitDims<U>()`, a function-local static built once per type. Before that, `dims()` built an `std::ostringstream` on every call (about 100 unit graphs and 70 unit pref reads per loop). `SubsystemGenomeHelper::LoadGenomePreferences` caches each (subsystem, genome name) and re-reads its prefs at most once per second, so genome pref edits take effect within 1 s. New cache entries are back-dated by 125 ms × their index, so the reloads land in different loops instead of all at once. It runs in 8 `WriteToHardware` calls every loop. Unit-typed prefs and graphs benefit indirectly because they recurse into the cached double overload.

CAN signal reads: `TalonFX_interm::Read` refreshes position, velocity and acceleration together with one `BaseStatusSignal::RefreshAll` the first time it is called in a loop, and later reads in that loop reuse them (`Tick()` clears the flag). Before this, every position or velocity read refreshed two signals on its own, about 100 separate refreshes per loop, and `MonkeyMaster::WriteMessages` repeated the reads the subsystems had already done. Latency compensation still extrapolates to the time of the call. The swerve CANcoder graph and `Robot/can_bus_usage` are sampled every 10th loop. `SupremeLimiter::Limit` takes its inputs by const reference; each `BasePlant` copy also copies a `std::function`, which heap-allocates.

Loop profiling: `GenericRobot` publishes `Robot/profile/<phase>_avg_ms` / `_max_ms` once a second for the phases `mode`, `reads`, `scheduler`, `periodic`, `writes`, `motors`, `reset_check` and `rest`. It also publishes `Robot/profile/subsystem/<name>_avg_ms` / `_max_ms` (reads and writes combined, averaged per loop) and `Robot/profile/loop_max_ms`. Per-subsystem timing lives in `GenericRobotContainer::Timed`. In Nov 2026 the loop's own work took 8.5–9.4 ms of its 10 ms period, before this profiling existed. On 2026-11-17, after the CAN read caching, enabled loops took a median of 6 ms (95th percentile 9 ms) and went over 10 ms 2–3% of the time. While enabled, the only loops over 20 ms were the first loop after each enable (31–38 ms, the mode transition). Every 25–45 ms `Loop overrun` message came in the first 35 s after boot, while disabled. That includes the hood reset below, about 20 ms `Hoptake_ss` writes, 25–30 ms `SwerveDrivetrain` reads, and 17 ms `control_input` writes, so none of them affect driving.

### Critical Bug Fixes

9. **HAL timing units**: `HAL_GetFPGATime()` returns **microseconds**, NOT milliseconds. The implementation in `src/funkit/cpp/funkit/wpilib/time.cc` had a bug where it wrapped the microsecond value in `pdcsu::units::ms_t`, causing a 1000x timing error. The correct implementation converts microseconds to seconds: `pdcsu::units::second_t(time_us / 1000000.0)`. Similarly, `HAL_UpdateNotifierAlarm()` expects microseconds. Always verify HAL function units with the WPILib documentation.

10. **Static thread_local caching bug in SwerveModuleSubsystem**: The `SwerveModuleSubsystem` constructor had `static thread_local` cached parameters in lambdas that caused all four swerve modules to share the same motor parameters (including CAN IDs) from the first constructed module. The bug was in initializer list lambdas that cached `getMotorParams()` results. This caused all steer motors to read the same angle. **Never use `static thread_local` in constructor initializer lists when each instance needs unique values**. The fix was to directly call `getMotorParams()` without caching: `drive_params_{getMotorParams(unique_config, common_config).first}` instead of using a lambda with `static thread_local auto cached_params`.

11. **Inverted steer gear ratio**: The steer gear ratio in `DrivetrainConstructor.cc` was inverted, causing steer position readings to be 21.4x too large. PDCSU's `DefArmSys` expects `gear_ratio` to represent "motor rotations per output rotation" because it uses `toReal(motor_pos) = motor_pos / gear_ratio`. For a 150:7 gearbox (150 motor rotations → 7 output rotations), the gear ratio should be `150_tr / 7_tr = 21.43`, NOT `7_tr / 150_tr = 0.0467`. **Always verify gear ratio direction**: if the gearbox is N:M (N motor rotations → M output rotations), use `N/M` for PDCSU plants.

12. **Inverted drive gear ratio**: The drive gear ratio formula in `DrivetrainConstructor.cc` was inverted, causing drive position readings to be 68x too small. PDCSU's `DefLinearSys` expects `gear_ratio` (type `UnitDivision<radian_t, meter_t>`) to represent "motor radians per meter" because it uses `toReal(motor_radians) = motor_radians / gear_ratio` to get meters. The formula was `(drive_reduction_value * 12.0) / 3.14159265358979323846` which gave ~1.94 rad/m, but should be `(2.0 * 3.14159265358979323846 * 3.28084) / drive_reduction_value` which gives ~132.9 rad/m. **For DefLinearSys**: gear_ratio = (2π × conversion_to_feet) / (feet_per_motor_rotation).

13. **Loggable non-copyable NT handle members**: `Loggable` caches `nt::DoublePublisher` / `nt::DoubleEntry` handles in `std::unordered_map`s for the `Graph(double)` / `GetPreferenceValue_double` hot paths. Both NT types inherit from `nt::Publisher` / `nt::Subscriber`, which explicitly `= delete` their copy constructors. This makes the unordered_map's copy constructor **declared but ill-formed when instantiated** (libc++ fails inside `insert(__u.begin(), __u.end())`). Without explicit special members, the compiler may instantiate `Loggable`'s implicit copy constructor during overload resolution (e.g. for `frc2::CommandHelper::ToPtr()`'s `std::make_unique<CRTP>(std::move(*this))`), producing cryptic `__construct_at` errors in `<__memory/allocator_traits.h>`. Fix in `Loggable.h`: explicitly `Loggable(const Loggable&) = delete;`, `Loggable& operator=(const Loggable&) = delete;`, and `Loggable(Loggable&&) = default;`. Because `GenericCommand` has a user-declared virtual destructor (which suppresses the implicit move constructor), it must also declare `GenericCommand(GenericCommand&&) = default;` so that `make_unique<CRTP>(std::move(*this))` resolves to a real move and doesn't fall back to copy. Also avoid taking `Loggable` by value: `SubsystemBase` previously took `Loggable parent` by value, forcing a copy at the call site (e.g. `SwerveModuleSubsystem`); use `const Loggable& parent` instead. **Rule**: any `Loggable`-derived class with a user-declared destructor needs `T(T&&) = default;` for `CommandHelper::ToPtr` (and other move-into-unique_ptr patterns) to compile, and never pass `Loggable` by value.

14. **PDCSU is now hosted at `github.com/prak132/PDCSU`**: `build.gradle` downloads the release asset `pdcsu_tr<N>.zip` from tag `tr_<N>` (currently 21). The zip needs a top-level `pdcsu_tr<N>/` folder (`git archive --prefix=pdcsu_tr20/ -o pdcsu_tr20.zip HEAD`). The working repo is `~/Downloads/robotics/PDCSU`; its files are CRLF in the working tree and LF in git. `src/pdcsu_tr<N>/` is git-ignored in gibbon, and gradle only downloads it when the folder is missing. To ship a PDCSU change, publish a new release number and bump `pdcsuReleaseNumber`; gradle then deletes the old `src/pdcsu_*` folder and downloads the new one on every machine. Re-uploading under the same tag does not reach machines that already have the folder. The new release has no GCC PCHs (`*.h.gch`); old releases shipped PCHs built from unpatched headers that GCC could use instead of edited headers. Patches on top of the original tr19: ICNOR plan coefficients (`icnor.h` `make_solve_context` rhs terms), learner confidence (`conf = (r2 - 0.5) * 2`), FFModel friction sign (`util.h`), and LS3x3 B/C Cramer terms (`solvers.h`), all from 2026-11-09. Released in tr21 (2026-11-17): `u_sqrt` returns the SI-based result (tr20 kept the input's factor and came out wrong for non-SI inputs), `u_floor` / `u_ceil` / `u_round` round in display units, `u_pow(u, double)` requires a dimensionless unit and `u_pow<Num, Den>(u)` was added, unit-by-unit `%` uses base values, `uVec::unit()` returns a vector of length 1 in its own unit, and `projectOntoThis` / `projectOntoAnother` divide by the squared length. On 2026-11-17 every one of these was marked `[[deprecated]]` in a scratch copy and all 61 Gibbon `.cc` files plus PDCSU's own headers compiled with no use of any of them (the two old `u_sqrt` calls in `ShootingCalculator` were already replaced), so releasing them changes no robot behaviour. `2024SystemCoreTesting` (pinned to tr19) calls `u_sqrt` on inch² in its odometry residual; bumping it would make those residuals 39x larger and change its Huber weights. Also `uVec` stores its components in a `std::array` instead of a `std::vector`, so vectors no longer heap-allocate (2–5x faster swerve, odometry and shooting math on a desktop, identical results; it changes struct layouts, so it needs a clean build). On 2026-11-13 `ICNORLearner::saveAsync` was changed to skip a save while the previous file write is still pending, instead of blocking the robot loop in `waitForIO()`.

### Windows Terminal Environment

13. **PowerShell syntax**: This repository uses Windows PowerShell. The `&&` operator does NOT work in PowerShell. Use semicolons (`;`) to chain commands, or use separate commands. For example:
   - ❌ Wrong: `cd src && move funkit funkit`
   - ✅ Correct: `cd src; Move-Item -Path funkit -Destination funkit` or use separate commands

### Development Guidelines

14. **Always inherit from base classes**: Use `GenericSubsystem`, `GenericCommand`, `Loggable`

15. **Register preferences**: All configurable values should be preferences

16. **Use PDCSU unit types and literals**: Prefer PDCSU `units::unit_t` types over raw doubles (see [Units Library](#units-library) section for details). **ALWAYS use PDCSU literals with trailing underscore** (e.g. `45_deg_`, `10_fps_`, `1_rad_`), **NEVER use WPILib literals without trailing underscore** (e.g. `45_deg`, `10_fps`, `1_rad`). This is a critical distinction—WPILib and PDCSU literals look similar but are incompatible.

17. **Follow naming conventions**: See [Naming Conventions](#naming-conventions) section below

18. **Log appropriately**: Use `Log()`, `Warn()`, `Error()` for debugging

19. **Update groups matter**: Register subsystems with appropriate groups

20. **Readings are immutable**: Don't modify readings after reading from hardware

21. **Targets set state**: Commands should set subsystem targets, not directly control hardware

22. **Preference keys are hierarchical**: Include subsystem name in key if needed

23. **Verify hardware**: Always implement `VerifyHardware()` for troubleshooting

### Naming Conventions

The codebase follows consistent naming conventions for maintainability:

#### Classes and Types
- **Subsystems**: `SubsystemNameSubsystem` (e.g., `LEDsSubsystem`, `DrivetrainSubsystem`)
- **Commands**: `CommandNameCommand` (e.g., `DriveCommand`, `AimCommand`)
- **Structs for data**: `SubsystemNameReadings`, `SubsystemNameTarget` (e.g., `DrivetrainReadings`, `DrivetrainTarget`)
- **Constructors/Helpers**: `SubsystemNameConstructor` (e.g., `DrivetrainConstructor`)
- **Abstract/Logic subsystems**: Descriptive names without "Subsystem" suffix when it's clear from context (e.g., `ControlInputSubsystem`, `GPDSubsystem`)

#### Files
- **Files match class names**: Header and implementation files use the same name as the primary class
  - `MySubsystem.h` / `MySubsystem.cc` for `MySubsystem` class
  - Use lowercase with underscores for file names matching C++ conventions
- **Location follows structure**:
  - Hardware subsystems: `src/yearly/include/subsystems/hardware/`, `src/yearly/cpp/subsystems/hardware/`
  - Abstract subsystems: `src/yearly/include/subsystems/abstract/`, `src/yearly/cpp/subsystems/abstract/`
  - Commands: `src/yearly/include/commands/`, `src/yearly/cpp/commands/`

#### Variables and Members
- **Member variables**: Use trailing underscore (e.g., `container_`, `leds_`, `drivetrain_`)
- **Local variables**: No trailing underscore, camelCase or descriptive names
- **Constants**: `kConstantName` (e.g., `kFRDrive_CANID`, `kLEDStrip1`)
- **Preference keys**: Use snake_case (e.g., `max_speed`, `translation_deadband`)

#### Namespaces and Organization
- **Custom library**: Everything in `src/funkit/` uses `funkit::` namespace
- **Year-specific code**: Everything in `src/yearly/` typically in global namespace (or year-specific namespace if needed)
- **Nested namespaces**: Use nested namespaces for logical grouping (e.g., `funkit::robot::swerve::`)

#### Preferences
- **Hierarchical naming**: Preferences are automatically namespaced by the `Loggable` name
  - Subsystem `"drivetrain"` with key `"max_speed"` → stored as `"drivetrain/max_speed"`
- **Unit types**: Unit preferences automatically append unit abbreviation (e.g., `"max_speed (ft/s)"`)
- **Grouping**: Use forward slashes for logical grouping within a subsystem (e.g., `"gains/_kP"`, `"limits/upper_limit"`)

## Additional Resources

- **README.md**: Setup instructions and basic usage
- **WPILib Documentation**: https://docs.wpilib.org/
- **Phoenix 6 Documentation**: https://pro.docs.ctr-electronics.com/
- **REV Documentation**: https://docs.revrobotics.com/

---

*Last Updated: Nov 2026 - WPILib 2026.2.1, Phoenix 6 26.3.0, PDCSU tr21. (The rest of this line is from Jan 2026.) RobotContainer: ShooterSubsystem, IntakeSubsystem, TurretTestSubsystem (testcrt); no ICTestSubsystem. Commands: DriveCommand, ShooterCommand, IntakeCommand (commands/teleop/). FunkyRobot: control_triggers (ControlTriggerInitializer), rsighandler (configureSignalHandlers in sim), ShootingCalculator, LEDsLogic. yearly include: control_triggers.h, rsighandler.h, robot_constants.h, SubsystemHelper.h. HigherMotorController: base::MotorMonkeyType, config::MotorGenome/MotorConstructionParameters, pdcsu::util::DefLinearSys/DefArmSys, SetControllerSoftLimits/SetSoftLimits, SpecialConfiguration/SpecialRead, config::StatusFrame. ports: driver_, operator_, shooter_, intake_ (MotorConstructionParameters).*

