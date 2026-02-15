# AGENTS.md - Repository Architecture Guide

This document provides a comprehensive overview of the FRC Team 846 codebase architecture, patterns, and structure to help AI agents and developers understand and work with this repository effectively.

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
  - Phoenix 6 v26.1.0 (CTRE motor controllers)
  - REVLib 2026.0.0 (REV motor controllers)
- **Code Quality**: Spotless (clang-format 18.1.8), CppCheck 2.16.0
- **PDCSU**: tr15

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
│   ├── pdcsu_tr15/               # PDCSU library (external, auto-downloaded)
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
- Teleop default commands: `DriveCommand`, `ShooterCommand`, `IntakeCommand`; trigger binding via `ControlTriggerInitializer::InitTeleopTriggers` (from `control_triggers.h`)
- Simulation: `configureSignalHandlers()` from `rsighandler.h` in `main()`; FunkyFMS control mode for DS state
- `ShootingCalculator::Setup` in `OnInitialize`; `ShootingCalculator::Calculate` in `OnPeriodic`

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

### Supported Motor Controllers

- **CTRE TalonFX** (Phoenix 6)
- **CTRE TalonSRX** (Phoenix 6)
- **REV Spark MAX**
- **REV Spark Flex**

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
- **PDCSU auto-download**: Downloads PDCSU tr15 library from GitHub releases (config: `pdcsuGroup = "tr"`, `pdcsuReleaseNumber = "15"`)
- **Multi-platform**: Supports RoboRIO and desktop (simulation)
- **Static file deploy**: `fileTree('src/yearly/deploy')` deployed to `/home/lvuser/deploy`

### Build Targets

- **Build**: `./gradlew build`
- **Deploy**: `./gradlew deploy` (requires robot connection)
- **Simulation**: `./gradlew simulateJava` or `./gradlew simulateNative`
- **Format**: `./gradlew spotlessApply`

### Source Organization

- **C++ sources**: `src/funkit/cpp/**/*.cc`, `src/yearly/cpp/**/*.cc`
- **Headers**: `src/funkit/include`, `src/yearly/include`, `src/pdcsu_tr15`

### Dependencies

Managed via `vendordeps/` JSON files:
- `Phoenix6-26.1.0.json`: CTRE Phoenix 6 v26.1.0 (frcYear: 2026)
- `REVLib.json`: REV Robotics 2026.0.0 (frcYear: 2026)
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

The codebase uses **PDCSU units** (`pdcsu::units`, from `pdcsu_tr15/util/units.h`) as the primary unit system for type-safe physical quantities. PDCSU units provide compile-time type checking and automatic conversions between compatible unit types. **Keep quantities in unit form as much as possible**; only call `.value()` when necessary (e.g. at API boundaries, logging, or when a raw double is required). Units with the same dimensions (L, M, T, I, R) **auto-convert** via constructors and operators.

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

**Members** (`pdcsu::units::Unit`, `pdcsu_tr15/util/units.h`):

| Member | Description |
|--------|-------------|
| `value()` | Returns the quantity in display units (e.g. `foot_t{3}.value()` → 3.0). Use only at API/logging boundaries. |
| `to_base()` | Internal representation in base units; used for same-dimension math. Prefer typed operations over manual base use. |
| `from_base(double)` | Static: build a unit from a base-value. Same type as the unit you call it on (e.g. `foot_t::from_base(x)`). |
| `dims()` | Returns a string of dimension exponents (e.g. `"m"`, `"ft/s"`) for debugging. |

**Arithmetic**: Same-dimension units support `+`, `-`, `+=`, `-=` (types can differ in factor/tag, e.g. `inch_t` and `foot_t`). `*` and `/` with another unit yield a new compound/division type (e.g. `foot_t * second_t` → length×time). Multiplication/division by a plain `double` scale the value and preserve the unit type. `operator%(unit)` and `operator%(double)` use `std::fmod` on the numeric value and return the same unit type.

**Comparisons**: `==`, `!=`, `<`, `<=`, `>`, `>=` are defined for same-dimension units.

**Free functions** (in `pdcsu::units`):

| Function | Description |
|----------|-------------|
| `u_abs(u)` | Returns the unit with non-negative numeric value; same type as `u`. |
| `u_sqrt(u)` | Returns the square root as a unit with halved dimension exponents (e.g. `mps2_t` → `mps_t`). |
| `u_sin(u)`, `u_cos(u)`, `u_tan(u)` | Trig on `radian_t` or `degree_t`; return `double`. |
| `u_asin(x)`, `u_acos(x)`, `u_atan(x)` | Inverse trig on raw `double`; return `radian_t`. |
| `u_atan2(y, x)` | Two-argument atan; `y` and `x` must be same-dimension units; returns `radian_t`. |
| `u_min(a, b)`, `u_max(a, b)` | Same-dimension units; return type of first argument. |
| `u_clamp(u, min, max)` | Clamp `u` to `[min, max]`; all same dimensions. |
| `u_floor(u)`, `u_ceil(u)`, `u_round(u)` | Round to integer value in display units; return same unit type. |
| `u_pow(u, exp)` | Power; `exp` is `double`; dimensions scale by `exp`. |
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

This matches how PDCSU's `DefLinearSys` and related code use `1_rad_` when combining length and angle (see `pdcsu_tr15/util/sysdef.h`).

#### Vector Types and uVec class

The codebase uses **PDCSU vectors** (`pdcsu::util::math::uVec<UT, N>` from `pdcsu_tr15/util/math/uvec.h`) instead of WPILib vectors. Vectors hold unit-typed elements and keep type safety. Common aliases: `Vector1D`, `Vector2D`, `Vector3D` are `uVec<pdcsu::units::inch_t, 1>`, `uVec<pdcsu::units::inch_t, 2>`, `uVec<pdcsu::units::inch_t, 3>`.

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
| `unit()` | `uVec<UT,N>` | Normalized direction; uses `magnitude().to_base()` for division. |
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
- **Odometry**: Position tracking via module encoders and gyro
- **Pose estimation**: Kalman filter fusion with AprilTag vision
- **Open-loop control**: Velocity-based control for teleop
- **Path following**: Commands for driving to points and locking

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

### Critical Bug Fixes

9. **HAL timing units**: `HAL_GetFPGATime()` returns **microseconds**, NOT milliseconds. The implementation in `src/funkit/cpp/funkit/wpilib/time.cc` had a bug where it wrapped the microsecond value in `pdcsu::units::ms_t`, causing a 1000x timing error. The correct implementation converts microseconds to seconds: `pdcsu::units::second_t(time_us / 1000000.0)`. Similarly, `HAL_UpdateNotifierAlarm()` expects microseconds. Always verify HAL function units with the WPILib documentation.

10. **Static thread_local caching bug in SwerveModuleSubsystem**: The `SwerveModuleSubsystem` constructor had `static thread_local` cached parameters in lambdas that caused all four swerve modules to share the same motor parameters (including CAN IDs) from the first constructed module. The bug was in initializer list lambdas that cached `getMotorParams()` results. This caused all steer motors to read the same angle. **Never use `static thread_local` in constructor initializer lists when each instance needs unique values**. The fix was to directly call `getMotorParams()` without caching: `drive_params_{getMotorParams(unique_config, common_config).first}` instead of using a lambda with `static thread_local auto cached_params`.

11. **Inverted steer gear ratio**: The steer gear ratio in `DrivetrainConstructor.cc` was inverted, causing steer position readings to be 21.4x too large. PDCSU's `DefArmSys` expects `gear_ratio` to represent "motor rotations per output rotation" because it uses `toReal(motor_pos) = motor_pos / gear_ratio`. For a 150:7 gearbox (150 motor rotations → 7 output rotations), the gear ratio should be `150_tr / 7_tr = 21.43`, NOT `7_tr / 150_tr = 0.0467`. **Always verify gear ratio direction**: if the gearbox is N:M (N motor rotations → M output rotations), use `N/M` for PDCSU plants.

12. **Inverted drive gear ratio**: The drive gear ratio formula in `DrivetrainConstructor.cc` was inverted, causing drive position readings to be 68x too small. PDCSU's `DefLinearSys` expects `gear_ratio` (type `UnitDivision<radian_t, meter_t>`) to represent "motor radians per meter" because it uses `toReal(motor_radians) = motor_radians / gear_ratio` to get meters. The formula was `(drive_reduction_value * 12.0) / 3.14159265358979323846` which gave ~1.94 rad/m, but should be `(2.0 * 3.14159265358979323846 * 3.28084) / drive_reduction_value` which gives ~132.9 rad/m. **For DefLinearSys**: gear_ratio = (2π × conversion_to_feet) / (feet_per_motor_rotation).

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

*Last Updated: Jan 2026 - WPILib 2026.2.1, PDCSU tr15. RobotContainer: ShooterSubsystem, IntakeSubsystem, TurretTestSubsystem (testcrt); no ICTestSubsystem. Commands: DriveCommand, ShooterCommand, IntakeCommand (commands/teleop/). FunkyRobot: control_triggers (ControlTriggerInitializer), rsighandler (configureSignalHandlers in sim), ShootingCalculator, LEDsLogic. yearly include: control_triggers.h, rsighandler.h, robot_constants.h, SubsystemHelper.h. HigherMotorController: base::MotorMonkeyType, config::MotorGenome/MotorConstructionParameters, pdcsu::util::DefLinearSys/DefArmSys, SetControllerSoftLimits/SetSoftLimits, SpecialConfiguration/SpecialRead, config::StatusFrame. ports: driver_, operator_, shooter_, intake_ (MotorConstructionParameters).*

