#include "commands/teleop/drive_test.h"

#include <iostream>

// #include "funkit/control/hardware/TalonFX_interm.h"

DriveTest::DriveTest(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, DriveTest>{
          container, "drive_test"} {
  AddRequirements({&container_.drivetrain_});
}

void DriveTest::OnInit() {
  // start timer
  std::cout << "Initialized" << std::endl;
  timer.Start();
  auto readings = container_.drivetrain_.GetReadings();
  start_pos_ = readings.estimated_pose
                   .position;  // start pos should be a vector asw not just
  timer_count = 0;

  std::cout << "Command started" << std::endl;
}

void DriveTest::Periodic() {
  if (container_.control_input_.GetReadings().drive_test) {
    // current_pos_ =
    // container_.drivetrain_.GetReadings().estimated_pose.position; auto x_diff
    // = current_pos_[0] - start_pos_[0]; auto y_diff = current_pos_[1] -
    // start_pos_[1]; std::cout << "X difference:" << x_diff.value() <<
    // std::endl; std::cout << "Y difference:" << y_diff.value() << std::endl;
    timer.Start();
    current_pos_ = container_.drivetrain_.GetReadings().estimated_pose.position;
    // auto x_diff = current_pos_[0] - start_pos_[0];
    // auto y_diff = current_pos_[1] - start_pos_[1];
    funkit::robot::swerve::DrivetrainTarget target{};
    target.velocity = {
        0.0_fps_, container_.drivetrain_.GetPreferenceValue_double(
                      "drive_test/duty_cycle") *
                      17.4_fps_};
    target.angular_velocity = 0.0_degps_;
    target.use_dc = false;
    target.duty_cycle = 0.0;
    timer_count += 0.01;
    // std::cout << "current time" << timer_count << std::endl;
    auto current_vel =
        container_.drivetrain_.GetReadings().estimated_pose.velocity;
    if (current_vel.magnitude() > highestVel.magnitude()) {
      highestVel = current_vel;
    }
    pdcsu::units::fps2_t current_accel =
        container_.drivetrain_.GetReadings().acceleration;
    if (current_accel.value() > highestAccel.value()) {
      highestAccel = current_accel;
    }
    // if (x_diff > 10_ft_ || y_diff > 10_ft_) {
    //   units::second_t timestamp = timer.GetFPGATimestamp();
    //   timer.Stop();

    //   auto mag = highestVel.magnitude();
    //   std::cout << "max vel " << mag.to_base() << std::endl;
    //   std::cout << "max accel " << highestAccel.to_base() << std::endl;
    //   std::cout << "time secconds " << timer.Get().to<double>() << std::endl;
    //   std::cout << "second timer secconds " << timer_count << std::endl;

    // }

    container_.drivetrain_.SetTarget(target);
  } else {
    funkit::robot::swerve::DrivetrainTarget target{};
    target.velocity = {0.0_fps_, 0.0_fps_};
    target.angular_velocity = 0.0_degps_;
    target.use_dc = false;
    target.duty_cycle = 0.0;
    container_.drivetrain_.SetTarget(target);
  }
}

void DriveTest::OnEnd(bool interrupted) {
  units::second_t timestamp = timer.GetFPGATimestamp();
  timer.Stop();
  auto mag = highestVel.magnitude();
  // std::cout << "max vel " << mag.to_base() << std::endl;
  // std::cout << "max accel " << highestAccel.to_base() << std::endl;
  // std::cout << "time secconds " << timestamp.to<double>() << std::endl;
  current_pos_ = container_.drivetrain_.GetReadings().estimated_pose.position;
  auto x_diff = current_pos_[0] - start_pos_[0];
  auto y_diff = current_pos_[1] - start_pos_[1];
  std::cout << "X difference:" << x_diff.value() << std::endl;
  std::cout << "Y difference:" << y_diff.value() << std::endl;
  std::cout << "second timer secconds " << timer_count << std::endl;
  std::cout << "second timer secconds " << timer_count << std::endl;
  std::cout << "second timer secconds " << timer_count << std::endl;
  std::cout << "second timer secconds " << timer_count << std::endl;
  std::cout << "second timer secconds " << timer_count << std::endl;
  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0.0_fps_, 0.0_fps_};
  target.angular_velocity = 0.0_degps_;
  target.use_dc = false;
  target.duty_cycle = 0.0;
  container_.drivetrain_.SetTarget(target);
}

bool DriveTest::IsFinished() {
  current_pos_ = container_.drivetrain_.GetReadings().estimated_pose.position;
  auto x_diff = current_pos_[0] - start_pos_[0];
  auto y_diff = current_pos_[1] - start_pos_[1];

  // std::cout << "X difference:" << x_diff.value() << std::endl;
  // std::cout << "Y difference:" << y_diff.value() << std::endl;
  // std::cout << "FINISHED FINISHED FINISHED FINISHED FINISHED FINISHED
  // FINISHED FINISHED FINISHED FINISHED FINISHED FINISHED" << std::endl;
  return (x_diff > container_.drivetrain_
                       .GetPreferenceValue_unit_type<pdcsu::units::foot_t>(
                           "drive_test/distance") ||
          y_diff > container_.drivetrain_
                       .GetPreferenceValue_unit_type<pdcsu::units::foot_t>(
                           "drive_test/distance"));
  // return false;
}