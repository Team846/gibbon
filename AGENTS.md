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

This is the **2026 FRC codebase for Team 846 (Gibbon)**, written in **C++** using **WPILib 2026.1.1**. The codebase implements a custom robot architecture built on top of WPILib's command-based framework.

### Key Technologies
- **Language**: C++17/20
- **Framework**: WPILib 2026.1.1
- **Build System**: Gradle (WPILib GradleRIO plugin 2026.1.1)
- **Vendor Libraries**: 
  - Phoenix 6 v26.1.0 (CTRE motor controllers)
  - REVLib 2026.0.0 (REV motor controllers)
- **Code Quality**: Spotless (clang-format 18.1.8), CppCheck 2.16.0
- **PDCSU**: tr12

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
│   │   │   │   ├── abstract/     # Abstract/logic subsystems (not hardware)
│   │   │   │   └── hardware/     # Hardware subsystem implementations
│   │   │   ├── commands/
│   │   │   │   ├── general/      # General commands
│   │   │   │   └── teleop/       # Teleoperated commands
│   │   │   ├── calculators/      # Utility calculators
│   │   │   └── FunkyRobot.cc     # Main robot class implementation
│   │   ├── include/              # Header files (.h)
│   │   │   ├── subsystems/       # Subsystem headers
│   │   │   ├── commands/         # Command headers
│   │   │   ├── autos/            # Autonomous routine definitions
│   │   │   ├── ports.h           # CAN IDs, PWM ports, etc.
│   │   │   ├── robot_constants.h # Robot-specific constants
│   │   │   └── FunkyRobot.h      # Main robot class header
│   │   └── resources/            # Non-code resources
│   │       ├── deploy/           # Files deployed to RoboRIO
│   │       │   └── autos/        # Scriptable autonomous routines
│   │       ├── logsclient/       # Log client scripts
│   │       ├── ntbackup/         # NetworkTables backup tools
│   │       └── swervevis/        # Swerve visualization tools
│   ├── funkit/                   # Custom architecture library
│   │   ├── cpp/                  # Implementation files
│   │   └── include/
│   │       └── funkit/
│   │           ├── base/         # Base classes (Loggable, logging)
│   │           ├── control/      # Motor control system
│   │           │   ├── base/     # Motor control base classes
│   │           │   ├── calculators/  # Motor calculations
│   │           │   ├── config/       # Configuration structures
│   │           │   └── hardware/     # Hardware abstractions
│   │           ├── math/         # Math utilities
│   │           ├── robot/        # Robot framework
│   │           │   ├── calculators/  # Robot calculators (e.g., AprilTag)
│   │           │   └── swerve/       # Swerve drive implementation
│   │           └── wpilib/       # WPILib utilities
│   ├── pdcsu_tr12/               # PDCSU library (external, auto-downloaded)
│   └── deploy/                   # Files to deploy to RoboRIO
│       └── autos/                # Autonomous sequence files
├── build/                        # Build outputs (generated)
├── gradle/                       # Gradle wrapper files
├── vendordeps/                   # Vendor dependency JSON files
├── build.gradle                  # Main build configuration
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
- Dashboard button initialization
- Path recording control
- LED state management
- Homing switch handling
- Coast mode control

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
  ICTestSubsystem ictest_{};
};
```

**Registration pattern**:
- Use preferences to conditionally initialize subsystems
- Register with appropriate group (A, B, or AB) for update scheduling

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
- **DrivetrainSubsystem**: Swerve drive with 4 modules
- **LEDsSubsystem**: LED strip control
- **ICTestSubsystem**: Test subsystem for motor controllers

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

**Key commands**:
- **DriveCommand**: Teleoperated swerve drive control

---

## Motor Control System

### HigherMotorController

**Location**: `src/funkit/include/funkit/control/HigherMotorController.h`

High-level motor controller abstraction that interfaces with `MonkeyMaster`:

```cpp
class HigherMotorController {
  HigherMotorController(MotorMonkeyType mmtype, MotorConstructionParameters params);
  
  void Setup(MotorGenome genome, std::variant<DefLinearSys, DefArmSys> plant);
  void ModifyGenome(MotorGenome genome);
  
  // Control modes:
  void WriteDC(double duty_cycle);
  template <typename VelUnit> void WriteVelocity(VelUnit velocity);
  template <typename PosUnit> void WritePosition(PosUnit position);
  template <typename VelUnit> void WriteVelocityOnController(VelUnit velocity);
  template <typename PosUnit> void WritePositionOnController(PosUnit position);
  template <typename PosUnit> void SetPosition(PosUnit position);  // Zeroes encoder
  
  // Sensor readings (template functions with unit type safety):
  template <typename VelUnit> VelUnit GetVelocity();
  template <typename PosUnit> PosUnit GetPosition();
  pdcsu::units::amp_t GetCurrent();
  
  // Utility:
  void SetLoad(pdcsu::units::nm_t load);
  bool VerifyConnected();
  void EnableStatusFrames(std::vector<StatusFrame> frames, ...);
};
```

**Key features**:
- **Template functions**: `GetPosition<meter_t>()` for linear systems, `GetPosition<radian_t>()` for arm systems
- **Plant-based unit conversion**: Uses `DefLinearSys` or `DefArmSys` to convert between real-world units and native motor controller units
- **Type safety**: `static_assert` ensures correct unit types match the plant type
- **MotorGenome configuration**: All motor parameters (current limits, voltage compensation, PIDF gains, brake mode) stored in `MotorGenome`

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

Motor configuration is managed through `MotorGenome` and `SubsystemGenomeHelper`:

```cpp
// MotorGenome contains all motor configuration:
struct MotorGenome {
  pdcsu::units::amp_t motor_current_limit;
  pdcsu::units::amp_t smart_current_limit;
  pdcsu::units::volt_t voltage_compensation;
  bool brake_mode;
  config::Gains gains;  // PIDF gains (kP, kI, kD, kF)
};

// Create genome preferences in constructor:
MotorGenome genome_backup{
    .motor_current_limit = 40_u_A,
    .smart_current_limit = 30_u_A,
    .voltage_compensation = 12_u_V,
    .brake_mode = true};
SubsystemGenomeHelper::CreateGenomePreferences(*this, "genome", genome_backup);

// Load genome in Setup():
auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
motor_.Setup(genome, plant_variant);
```

**MotorConstructionParameters** is now simplified to only contain:
- `can_id`: CAN address
- `bus`: CAN bus name (empty string for default)
- `inverted`: Motor inversion
- `max_wait_time`: Control message timeout

### Ports Configuration

**Location**: `src/yearly/include/ports.h`

All hardware ports defined in a single location:
```cpp
struct ports {
  struct drivetrain_ {
    static constexpr int kFRDrive_CANID = 2;
    static constexpr int kFLDrive_CANID = 5;
    // ...
  };
  struct leds_ {
    static constexpr int kLEDStrip1 = 6;
  };
  // ...
};
```

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
- **WPILib GradleRIO plugin**: 2026.1.1
- **Spotless**: Code formatting with clang-format 18.1.8
- **CppCheck**: Static analysis (2.16.0)
- **PDCSU auto-download**: Downloads PDCSU tr12 library from GitHub releases
- **Multi-platform**: Supports RoboRIO and desktop (simulation)

### Build Targets

- **Build**: `./gradlew build`
- **Deploy**: `./gradlew deploy` (requires robot connection)
- **Simulation**: `./gradlew simulateJava` or `./gradlew simulateNative`
- **Format**: `./gradlew spotlessApply`

### Source Organization

- **C++ sources**: `src/funkit/cpp/**/*.cc`, `src/yearly/cpp/**/*.cc`
- **Headers**: `src/funkit/include`, `src/yearly/include`, `src/pdcsu_tr12`

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

1. **Create header** (`src/yearly/include/commands/my_command.h`):
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

The codebase uses **PDCSU units** (`pdcsu::units`) as the primary unit system for type-safe physical quantities. PDCSU units provide compile-time type checking and automatic conversions between compatible unit types.

#### Why Use PDCSU Units

- **Type safety**: Prevents mixing incompatible units (e.g., can't accidentally add meters to seconds)
- **Automatic conversions**: Units with the same dimensions are automatically converted
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

**Note**: PDCSU units do not support user-defined literals like WPILib. Always use explicit constructors: `pdcsu::units::degree_t{90}` instead of `90_u_deg`.

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

#### Extracting Numeric Values

When you need to extract the raw numeric value (for calculations, logging, or APIs that don't support units), use `.value()`:

```cpp
pdcsu::units::fps_t speed = pdcsu::units::fps_t{10};
double speed_value = speed.value();  // Extracts 10.0

pdcsu::units::degree_t angle = pdcsu::units::degree_t{45};
double angle_radians = pdcsu::units::radian_t{angle}.value();  // Convert and extract
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

#### Vector Types

The codebase uses **PDCSU vectors** (`pdcsu::util::math::uVec`) instead of WPILib vectors:

```cpp
using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;
Vector2D position{pdcsu::units::inch_t{10}, pdcsu::units::inch_t{20}};
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

1. **Always prefer PDCSU units**: Use PDCSU unit types internally throughout the codebase
2. **Convert at boundaries**: Only convert to WPILib units when interfacing with WPILib APIs
3. **Use explicit constructors**: Always use `pdcsu::units::degree_t{90}` instead of literals
4. **Extract only when necessary**: Use `.value()` only when interfacing with APIs that require raw values
5. **Type safety is your friend**: If code doesn't compile due to unit mismatch, it's preventing a bug!

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

12. **Inverted drive gear ratio**: The drive gear ratio formula in `DrivetrainConstructor.cc` was inverted, causing drive position readings to be 68x too small. PDCSU's `DefLinearSys` expects `gear_ratio` (type `UnitDivision<radian_t, meter_t>`) to represent "motor radians per meter" because it uses `toReal(motor_radians) = motor_radians / gear_ratio` to get meters. The formula was `(drive_reduction_value * 12.0) / M_PI` which gave ~1.94 rad/m, but should be `(2.0 * M_PI * 3.28084) / drive_reduction_value` which gives ~132.9 rad/m. **For DefLinearSys**: gear_ratio = (2π × conversion_to_feet) / (feet_per_motor_rotation).

### Windows Terminal Environment

13. **PowerShell syntax**: This repository uses Windows PowerShell. The `&&` operator does NOT work in PowerShell. Use semicolons (`;`) to chain commands, or use separate commands. For example:
   - ❌ Wrong: `cd src && move funkit funkit`
   - ✅ Correct: `cd src; Move-Item -Path funkit -Destination funkit` or use separate commands

### Development Guidelines

14. **Always inherit from base classes**: Use `GenericSubsystem`, `GenericCommand`, `Loggable`

15. **Register preferences**: All configurable values should be preferences

16. **Use unit types**: Prefer `units::unit_t` types over raw doubles (see [Units Library](#units-library) section for details)

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

*Last Updated: Jan 2026 - Updated for 2026 season with WPILib 2026.1.1, Phoenix 6 v26.1.0, REVLib 2026.0.0, and PDCSU tr12. Previous updates include MotorMonkey→MonkeyMaster rename, HigherMotorController API changes (MotorGenome, plant-based unit conversion), and removal of HMCHelper*

