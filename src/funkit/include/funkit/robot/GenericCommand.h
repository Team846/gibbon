#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "funkit/base/Loggable.h"
#include "funkit/wpilib/time.h"

namespace funkit::robot {

/**
 * GenericCommand
 *
 * A class that creates a generalized command. It inherits from both
 * CommandHelper and Loggable, helping provide logging utilities beyond WPILibs
 * Commands.
 *
 * A command represents a schedulable robot behavior. It gets
 * called by the CommandScheduler periodically while active, and ended when
 * finished or interrupted
 */
template <typename RobotContainer, typename Subclass>
class GenericCommand : public frc2::CommandHelper<frc2::Command, Subclass>,
                       public funkit::base::Loggable {
public:
  /**
   * Constructor for GenericCommand
   *
   * @param container: The robot container
   * @param name: The name of the command
   */
  GenericCommand(RobotContainer& container, std::string name)
      : Loggable{name}, container_{container} {
    frc2::Command::SetName(name);
    Log("Constructing instance of command {}.", name);
  }

  virtual ~GenericCommand() {
    Log("Destroying instance of command {}.", name());
  }

  // A method that gets called at the beginning of a command's lifecycle
  virtual void OnInit() = 0;

  // A method that gets called at the end of a command's lifecycle
  virtual void OnEnd(bool interrupted) = 0;

  // A method that gets called periodically during the command's lifecycle
  virtual void Periodic() = 0;

  // A native WPILib command lifecycle method. It logs initialization, start
  // time, and calls the custom implementation of GenericCommand, OnInit()
  void Initialize() override final {
    Log("Command {} initialized.", name());
    OnInit();

    command_start_time_ = funkit::wpilib::CurrentFPGATime();
  }

  // A native WPILib command lifecycle method. It logs the total time of the
  // command lifecycle, and calls the custom implementation of GenericCommand,
  // OnEnd()
  void End(bool interrupted) override final {
    pdcsu::units::second_t total_time =
        funkit::wpilib::CurrentFPGATime() - command_start_time_;

    Log("Command {} {}. Took {} ms to complete. Avg periodic {} ms. Peak "
        "periodic {} ms.",
        name(), interrupted ? "interrupted" : "finished",
        total_time.value() * 1000.0, avg_periodic_time_.value() * 1000.0,
        max_periodic_time_.value() * 1000.0);

    OnEnd(interrupted);
  }

  /**
   * Execute()
   *
   * A native WPILib command lifecycle method.
   * It logs the time to call a single periodic cycle and if the elapsed time
   * took too long, and finds the average periodic time of the command.
   *
   * It also calls the custom implementation of GenericCommand, Periodic()
   */
  void Execute() override final {
    const pdcsu::units::second_t start_time = funkit::wpilib::CurrentFPGATime();
    Periodic();
    const pdcsu::units::second_t end_time = funkit::wpilib::CurrentFPGATime();

    const auto elapsed_time = end_time - start_time;

    if (elapsed_time > pdcsu::units::second_t{0.003}) {
      Warn("Command {} periodic overrun. Took {} ms.", name(),
          elapsed_time.value() * 1000.0);
    }

    avg_periodic_time_ = pdcsu::units::second_t{
        (avg_periodic_time_.value() * num_periodic_loops_ +
            elapsed_time.value()) /
        (num_periodic_loops_ + 1)};
    max_periodic_time_ = std::max(max_periodic_time_, elapsed_time);

    num_periodic_loops_++;
  }

protected:
  RobotContainer& container_;

private:
  pdcsu::units::second_t avg_periodic_time_ = pdcsu::units::second_t{0};
  int num_periodic_loops_ = 0;

  pdcsu::units::second_t max_periodic_time_ = pdcsu::units::second_t{0};

  pdcsu::units::second_t command_start_time_ = pdcsu::units::second_t{0};
};

/**
 * GenericCommandGroup
 *
 * A templated class that creates a generalized command group.
 * It inherits from both CommandHelper and Loggable, following the
 * implementation of frc2::SequentialCommandGroup
 *
 * The GenericCommandGroup still acts as a single schedulable command, but
 * chains multiple commands together.
 */
template <typename RobotContainer, typename Subclass,
    wpi::DecayedDerivedFrom<frc2::Command>... Commands>
class GenericCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, Subclass>,
      public funkit::base::Loggable {
public:
  /**
   * Constructor for GenericCommandGroup.
   *
   * @param container: The robot container
   * @param name: The name of the command group
   * @param commands: The sequential list of commands
   *
   * Inherits from Loggable and initializes its member, container.
   * It sets the name of the command group and sequentially adds in commands
   * that it takes in from its parameters.
   */
  GenericCommandGroup(
      RobotContainer& container, std::string name, Commands&&... commands)
      : Loggable{name}, container_{container} {
    frc2::Command::SetName(name);

    frc2::SequentialCommandGroup::AddCommands(start_command_addition);
    frc2::SequentialCommandGroup::AddCommands(
        std::forward<Commands>(commands)...);
    frc2::SequentialCommandGroup::AddCommands(end_command_addition);

    Log("Constructing instance of command group {}.", name);
  }

protected:
  RobotContainer& container_;
  pdcsu::units::second_t command_start_time_ = pdcsu::units::second_t{0};

private:
  frc2::InstantCommand end_command_addition{[&] {
    // Log("Command group ending. Took {} ms to complete.",
    // (funkit::wpilib::CurrentFPGATime() - command_start_time_)
    //     .template to<double>()); TODO: Find out why we can't log
  }};

  frc2::InstantCommand start_command_addition{[&] {
    // Log("Command group starting {}", 1);
    command_start_time_ = funkit::wpilib::CurrentFPGATime();
  }};
};

}  // namespace funkit::robot
