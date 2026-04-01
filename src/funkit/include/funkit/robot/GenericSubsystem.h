#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

#include "funkit/base/Loggable.h"

namespace funkit::robot {

/**
 * FUNKIT_VERIFY
 * 
 * A macro used as a means to verify something, if failed it will log a verification error. 
 * Expr must evaluate to true for the check to pass. If not, the bool ok (A flag on if the whole verification routine passes) is set to false and an error is logged.
 * @param expr - Boolean on whether or not something is properly verified
 * @param ok - An Lvalue bool that is set to false on failure
 * @param fail_msg - The message attached to the Error log
 */
#define FUNKIT_VERIFY(expr, ok, fail_msg)       \
  if (!(expr)) {                                \
    ok = false;                                 \
    Error("Verification failed: {}", fail_msg); \
  }

/**
 * SubsystemBase
 * 
 * A non-templated subsystem base class that inherits from Loggable for logging utilities. 
 */
class SubsystemBase : public funkit::base::Loggable {
public:
  SubsystemBase(std::string name) : Loggable{name} {}
  SubsystemBase(Loggable parent, std::string name) : Loggable{parent, name} {}

  virtual ~SubsystemBase() = default;

  /**
   * Init()
   * 
   * Initialization of subsystem base by registering it in the container. Does not actually configure any related hardware.
   */
  virtual void Init() = 0;

   /**
   * Setup()
   * 
   * Sets up the subsystem base by configuring hardware, and is called when GenericRobot::StartCompetition() is called
   */
  virtual void Setup() = 0;

  // Updates readings of subsystem
  virtual void UpdateReadings() = 0;

  // Updates hardware of subsystem
  virtual void UpdateHardware() = 0;

  // Verifies hardware of subsystem
  virtual bool VerifyHardware() = 0;

  // Sets subsystem target to zero 
  virtual void SetTargetZero() = 0;
};

/**
 * GenericSubsystem
 * 
 * A templated base class for robot subsystems. 
 * Inherits from frc2::SubsystemBase (WPILib command subsystem) and SubsystemBase for logging and lifecycle utilities.
 * 
 * @tparam Readings - A struct holding information returned by ReadFromHardware(). 
 * @tparam Target - A struct holding commanded outputs that get passed to WriteToHardware(). 
 */
template <class Readings, class Target>
class GenericSubsystem : public frc2::SubsystemBase, public SubsystemBase {
public:
  // Construct a new subsystem.
  explicit GenericSubsystem(std::string name)
      : funkit::robot::SubsystemBase{name} {}

  // Construct a subsystem as a child of another subsystem.
  explicit GenericSubsystem(const Loggable& parent, std::string name)
      : funkit::robot::SubsystemBase{parent, name} {}

  bool is_initialized() { return init_; }

  /**
   * Disables the copy constructor for GenericSubsystem
   * Subsystems are unique objects, having functionality to copy can create issues. 
   */
  GenericSubsystem(const GenericSubsystem&) = delete;

  /**
   * Disables the operator= for GenericSubsystem
   * Subsystems are unique objects, having functionality for copy assignment can create issues. 
   */
  GenericSubsystem& operator=(const GenericSubsystem&) = delete;

  // Destructs a subsystem
  virtual ~GenericSubsystem() { Warn("Destroying subsystem"); };

  // Initializer function for RobotContainer use only.
  void Init() override final {
    SetName(name());
    Log("Initializing subsystem");
    init_ = true;
  }

  /**
   * InitByParent()
   * Alternative initializer function to be called by a parent subsystem only. Will not register with WPILib.
   * Used for child subsystems, specifically superstructures and drivetrain swerve modules.
   */
  void InitByParent() {
    SetName(name());
    Log("Initializing subsystem (by parent)");
    init_ = true;
  }

private:
  bool init_;

public:
  // Get the zero state target.
  virtual Target ZeroTarget() const = 0;

  // Fetches new readings and update subsystem readings state.
  void UpdateReadings() override final {
    if (is_initialized()) {
      readings_ = ReadFromHardware();
    } else {
      readings_ = Readings{};
    }
  }

  // Writes to subsystem hardware with the latest target output.
  void UpdateHardware() override final {
    if (is_initialized()) WriteToHardware(target_);
  }

  virtual bool VerifyHardware() override = 0;

  // Get the latest readings.
  const Readings& GetReadings() const { return readings_; };

  // Set the subystem target state.
  void SetTarget(Target target) { target_ = target; }

  // Set the subsystem to its zero state.
  void SetTargetZero() override { target_ = ZeroTarget(); }

  // Returns the latest target state
  auto GetTarget() const { return target_; }

private:
  Readings readings_;
  Target target_;

protected:
  // Fetches and return new readings.
  virtual Readings ReadFromHardware() = 0;

  // Writes output to hardware.
  virtual void WriteToHardware(Target target) = 0;
};

}  // namespace funkit::robot
