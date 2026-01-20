#include "funkit/robot/swerve/drivetrain.h"

#include <thread>

#include "frc/DriverStation.h"
#include "frc/RobotBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "funkit/control/config/genome.h"
#include "funkit/math/constants.h"
#include "funkit/math/fieldpoints.h"
#include "funkit/robot/swerve/control/swerve_ol_calculator.h"
#include "funkit/robot/swerve/swerve_module.h"

namespace funkit::robot::swerve {

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainConfigs configs)
    : GenericSubsystem{"SwerveDrivetrain"}, configs_{configs}, modules_{} {
  if (std::holds_alternative<PigeonConnection>(configs_.imu_connection)) {
    const auto& pigeon_conn =
        std::get<PigeonConnection>(configs_.imu_connection);
    pigeon_.emplace(pigeon_conn.canID, ctre::phoenix6::CANBus{""});
    pigeon_->OptimizeBusUtilization();
    pigeon_->GetYaw().SetUpdateFrequency(100_Hz);
    pigeon_->GetAngularVelocityZWorld().SetUpdateFrequency(100_Hz);
    pigeon_->GetAccelerationX().SetUpdateFrequency(100_Hz);
    pigeon_->GetAccelerationY().SetUpdateFrequency(100_Hz);
  } else if (std::holds_alternative<NavXConnection>(configs_.imu_connection)) {
    const auto& navx_conn = std::get<NavXConnection>(configs_.imu_connection);
    auto connection_type = navx_conn.connection_type == NavXConnectionType::kMXP
                               ? studica::AHRS::kMXP_SPI
                               : studica::AHRS::kUSB1;
    navX_.emplace(connection_type, studica::AHRS::k200Hz);
  }

  for (int i = 0; i < 4; i++) {
    modules_[i] = std::make_unique<SwerveModuleSubsystem>(*this,
        configs_.module_unique_configs[i], configs_.module_common_config);
  }

  funkit::control::config::MotorGenome drive_genome_backup{
      .motor_current_limit = pdcsu::units::amp_t{160.0},
      .smart_current_limit = pdcsu::units::amp_t{120.0},
      .voltage_compensation = pdcsu::units::volt_t{16.0},
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};
  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "drive_genome", drive_genome_backup);

  funkit::control::config::MotorGenome steer_genome_backup{
      .motor_current_limit = pdcsu::units::amp_t{120.0},
      .smart_current_limit = pdcsu::units::amp_t{80.0},
      .voltage_compensation = pdcsu::units::volt_t{10.0},
      .brake_mode = false,
      .gains = {.kP = 2.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};
  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "steer_genome", steer_genome_backup);

  RegisterPreference("steer_gains/_kP", 2.0);
  RegisterPreference("steer_gains/_kI", 0.0);
  RegisterPreference("steer_gains/_kD", 0.0);
  RegisterPreference("steer_gains/_kF", 0.0);

  RegisterPreference("bearing_gains/_kP", 9);
  RegisterPreference("bearing_gains/_kI", 0.0);
  RegisterPreference("bearing_gains/_kD", -0.6);
  RegisterPreference("bearing_gains/deadband", pdcsu::units::degps_t{3.0});

  RegisterPreference("april_bearing_latency", pdcsu::units::ms_t{0});
  RegisterPreference("drive_latency", pdcsu::units::ms_t{0});

  RegisterPreference("max_speed", pdcsu::units::fps_t{15});
  RegisterPreference("max_omega", pdcsu::units::degps_t{180});
  RegisterPreference("max_omega_cut", pdcsu::units::degps_t{40});

  RegisterPreference("odom_fudge_factor", 1.077);
  RegisterPreference("odom_variance", 0.2);

  RegisterPreference("steer_lag", pdcsu::units::second_t{0.05});
  RegisterPreference("bearing_latency", pdcsu::units::second_t{0.01});

  RegisterPreference("pose_estimator/pose_variance", 0.1);
  RegisterPreference("pose_estimator/velocity_variance", 1.0);
  RegisterPreference("pose_estimator/accel_variance", 1.0);
  RegisterPreference("pose_estimator/override", false);

  RegisterPreference("april_tags/april_variance_coeff", 0.08);
  RegisterPreference("april_tags/triangular_variance_coeff", 0.000139);
  RegisterPreference("april_tags/fudge_latency1", pdcsu::units::ms_t{155.0});
  RegisterPreference("april_tags/fudge_latency2", pdcsu::units::ms_t{70.0});

  RegisterPreference("drive_to_point/kC", 5.0);
  RegisterPreference("drive_to_point/kA", 0.05);
  RegisterPreference("drive_to_point/kE", 5.0);
  RegisterPreference("drive_to_point/threshold", pdcsu::units::inch_t{6});
  RegisterPreference(
      "drive_to_point/bearing_threshold", pdcsu::units::degree_t{5});

  odometry_.setConstants(
      {.forward_wheelbase_dim = configs.wheelbase_forward_dim,
          .horizontal_wheelbase_dim = configs.wheelbase_horizontal_dim});
  ol_calculator_.setConstants({
      .wheelbase_horizontal_dim = configs.wheelbase_horizontal_dim,
      .wheelbase_forward_dim = configs.wheelbase_forward_dim,
  });

  std::vector<std::shared_ptr<nt::NetworkTable>> april_tables = {};
  for (size_t i = 0; i < configs.cams; i++) {
    april_tables.push_back(nt::NetworkTableInstance::GetDefault().GetTable(
        "AprilTagsCam" + std::to_string(i + 1)));
  }
  tag_pos_calculator.setConstants({.tag_locations = configs.april_locations,
      .camera_x_offsets = configs.camera_x_offsets,
      .camera_y_offsets = configs.camera_y_offsets,
      .cams = configs.cams,
      .april_tables = april_tables});

#ifndef _WIN32
  for (int i = 0; i < 20; i++) {
    MainField_.GetObject(std::to_string(i));
  }
#endif
  frc::SmartDashboard::PutData("MainField", &MainField_);
}

void DrivetrainSubsystem::Setup() {
  using namespace funkit::control::config;
  using namespace pdcsu::units;

  auto drive_genome =
      SubsystemGenomeHelper::LoadGenomePreferences(*this, "drive_genome");
  auto steer_genome =
      SubsystemGenomeHelper::LoadGenomePreferences(*this, "steer_genome");

  for (auto& module : modules_) {
    module->InitByParent();
    module->SetDriveGenome(drive_genome);
    module->SetSteerGenome(steer_genome);
    module->Setup();
  }
  ZeroBearing();
}

DrivetrainTarget DrivetrainSubsystem::ZeroTarget() const {
  return {{pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}},
      pdcsu::units::degps_t{0}};
}

bool DrivetrainSubsystem::VerifyHardware() {
  bool ok = true;
  for (auto& module : modules_) {
    ok &= module->VerifyHardware();
  }
  FUNKIT_VERIFY(ok, ok, "At least one module failed verification");
  return ok;
}

void DrivetrainSubsystem::ZeroBearing() {
  if (!is_initialized()) return;

  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;

  if (!frc::DriverStation::IsAutonomous()) {
    if (frc::DriverStation::GetAlliance() ==
        frc::DriverStation::Alliance::kBlue)
      bearing_offset_ = pdcsu::units::degree_t{180};
    else
      bearing_offset_ = pdcsu::units::degree_t{0};
  }
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("Gyro zero attempt {}/{}", attempts, kMaxAttempts);
    bool connected = false;
    if (pigeon_.has_value()) {
      connected =
          pigeon_->IsConnected() && pigeon_->GetYaw().GetStatus().IsOK();
      if (connected) {
        pigeon_->SetYaw(0_deg);
        Log("Zeroed bearing (Pigeon)");
        return;
      }
    } else if (navX_.has_value()) {
      connected = navX_->IsConnected() && !navX_->IsCalibrating();
      if (connected) {
        navX_->ZeroYaw();
        Log("Zeroed bearing (navX)");
        return;
      }
    }

    Warn("Attempt to zero failed, sleeping {} ms...", kSleepTimeMs);

    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
  Error("Unable to zero after {} attempts, forcing zero", kMaxAttempts);

  if (pigeon_.has_value()) {
    pigeon_->SetYaw(0_deg);
  } else if (navX_.has_value()) {
    navX_->ZeroYaw();
  }
  // for (SwerveModuleSubsystem* module : modules_) {
  //   module->ZeroWithCANcoder();
  // }

  pose_estimator.SetPoint(
      std::array<double, 2>{GetReadings().april_point[0].value(),
          GetReadings().april_point[1].value()});
}

void DrivetrainSubsystem::SetBearing(pdcsu::units::degree_t bearing) {
  bearing_offset_ = bearing - (GetReadings().pose.bearing - bearing_offset_);
}

void DrivetrainSubsystem::SetPosition(Vector2D position) {
  odometry_.SetPosition(position);
  pose_estimator.SetPoint({position[0].value(), position[1].value()});
}

void DrivetrainSubsystem::SetOdomBearing(pdcsu::units::degree_t odom_bearing) {
  odometry_.SetOdomBearing(odom_bearing);
}

void DrivetrainSubsystem::SetCANCoderOffsets() {
  for (auto& module : modules_) {
    module->SetCANCoderOffset();
  }
}

pdcsu::units::degps_t DrivetrainSubsystem::ApplyBearingPID(
    pdcsu::units::degree_t target_bearing) {
  pdcsu::units::degree_t bearing = GetReadings().pose.bearing;
  pdcsu::units::degps_t yaw_rate = GetReadings().yaw_rate;

  pdcsu::units::degree_t error =
      funkit::math::CoterminalDifference(target_bearing, bearing);

  Graph("bearing_pid/error", error);

  funkit::control::config::Gains gains{
      .kP = GetPreferenceValue_double("bearing_gains/_kP"),
      .kI = GetPreferenceValue_double("bearing_gains/_kI"),
      .kD = GetPreferenceValue_double("bearing_gains/_kD"),
      .kF = 0.0};

  double raw_output = gains.kP * error.value() + gains.kI * 0.0 +
                      gains.kD * yaw_rate.value() + gains.kF * 0.0;

  pdcsu::units::degps_t output{
      pdcsu::units::degps_t{1} *
      funkit::math::HorizontalDeadband(raw_output,
          GetPreferenceValue_unit_type<pdcsu::units::degps_t>(
              "bearing_gains/deadband")
              .value(),
          GetPreferenceValue_unit_type<pdcsu::units::degps_t>("max_omega")
              .value())};

  Graph("bearing_pid/output", output);

  return output;
}

pdcsu::units::degree_t DrivetrainSubsystem::GetBearing() {
  if (pigeon_.has_value()) {
    auto bearing_wpi =
        ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
            pigeon_->GetYaw(), pigeon_->GetAngularVelocityZWorld());
    return pdcsu::units::degree_t{bearing_wpi.to<double>()};
  } else if (navX_.has_value()) {
    return pdcsu::units::degree_t{navX_->GetAngle()};
  }
  throw std::runtime_error("Neither Pigeon nor navX IMU is available");
}

pdcsu::units::degps_t DrivetrainSubsystem::GetYawRate() {
  if (pigeon_.has_value()) {
    return pdcsu::units::degps_t{
        pigeon_->GetAngularVelocityZWorld().GetValue().to<double>()};
  } else if (navX_.has_value()) {
    return pdcsu::units::degps_t{navX_->GetRate()};
  }
  throw std::runtime_error("Neither Pigeon nor navX IMU is available");
}

pdcsu::util::math::uVec<pdcsu::units::fps2_t, 2>
DrivetrainSubsystem::GetAcceleration() {
  if (pigeon_.has_value()) {
    return {pdcsu::units::fps2_t{
                pigeon_->GetAccelerationX().GetValue().to<double>()},
        pdcsu::units::fps2_t{
            pigeon_->GetAccelerationY().GetValue().to<double>()}};
  } else if (navX_.has_value()) {
    static constexpr double g_to_fps2 =
        funkit::math::constants::physics::g.to<double>() * 3.28084;
    return {pdcsu::units::fps2_t{navX_->GetWorldLinearAccelX() * g_to_fps2},
        pdcsu::units::fps2_t{navX_->GetWorldLinearAccelY() * g_to_fps2}};
  }
  throw std::runtime_error("Neither Pigeon nor navX IMU is available");
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  cached_pose_variance_ =
      GetPreferenceValue_double("pose_estimator/pose_variance");
  cached_velocity_variance_ =
      GetPreferenceValue_double("pose_estimator/velocity_variance");
  cached_accel_variance_ =
      GetPreferenceValue_double("pose_estimator/accel_variance");
  pose_estimator.Update(
      cached_pose_variance_, cached_velocity_variance_, cached_accel_variance_);

  pdcsu::units::degree_t bearing = GetBearing();
  pdcsu::units::degps_t yaw_rate = GetYawRate();

  bearing += bearing_offset_;

  cached_bearing_latency_ =
      GetPreferenceValue_unit_type<pdcsu::units::second_t>("bearing_latency");
  bearing += cached_bearing_latency_ * yaw_rate;

  if (frc::RobotBase::IsSimulation()) { bearing = odometry_.GetOdomBearing(); }

  Graph("readings/bearing", bearing);
  Graph("readings/yaw_rate", yaw_rate);

  pdcsu::util::math::uVec<pdcsu::units::inch_t, 4> drive_positions{
      pdcsu::units::inch_t{0}, pdcsu::units::inch_t{0}, pdcsu::units::inch_t{0},
      pdcsu::units::inch_t{0}};
  std::array<pdcsu::units::degree_t, 4> steer_positions{};

  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity{
      pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}};

  for (int i = 0; i < 4; i++) {
    modules_[i]->UpdateReadings();
    SwerveModuleReadings r = modules_[i]->GetReadings();

    drive_positions[i] = r.drive_pos;
    steer_positions[i] = r.steer_pos;

    velocity += (pdcsu::util::math::uVec<pdcsu::units::fps_t, 2>{
        r.vel, r.steer_pos + bearing, true});
  }

  velocity /= 4.0;

  Graph("readings/velocity_x", velocity[0]);
  Graph("readings/velocity_y", velocity[1]);

  Graph("readings/velocity_mag", velocity.magnitude());

  cached_odom_fudge_factor_ = GetPreferenceValue_double("odom_fudge_factor");
  funkit::robot::swerve::odometry::SwerveOdometryOutput odom_output =
      odometry_.calculate({bearing, steer_positions, drive_positions,
          cached_odom_fudge_factor_});

  funkit::robot::swerve::odometry::SwervePose new_pose{
      .position = odom_output.position,
      .bearing = bearing,
      .velocity = velocity,
  };

  Vector2D delta_pos = new_pose.position - GetReadings().pose.position;

  if (delta_pos.magnitude().value() < 10.0) {
    cached_odom_variance_ = GetPreferenceValue_double("odom_variance");
    pose_estimator.AddOdometryMeasurement(
        {delta_pos[0].value(), delta_pos[1].value()}, cached_odom_variance_);
  }

  cached_april_variance_coeff_ =
      GetPreferenceValue_double("april_tags/april_variance_coeff");
  cached_triangular_variance_coeff_ =
      GetPreferenceValue_double("april_tags/triangular_variance_coeff");
  cached_fudge_latency1_ = GetPreferenceValue_unit_type<pdcsu::units::ms_t>(
      "april_tags/fudge_latency1");
  cached_fudge_latency2_ = GetPreferenceValue_unit_type<pdcsu::units::ms_t>(
      "april_tags/fudge_latency2");
  cached_april_bearing_latency_ =
      GetPreferenceValue_unit_type<pdcsu::units::ms_t>("april_bearing_latency");

  funkit::robot::calculators::ATCalculatorOutput tag_pos =
      tag_pos_calculator.calculate({new_pose, GetReadings().pose, yaw_rate,
          cached_april_variance_coeff_, cached_triangular_variance_coeff_,
          {cached_fudge_latency1_, cached_fudge_latency2_},
          cached_april_bearing_latency_});

  if (tag_pos.variance >= 0) {
    pose_estimator.AddVisionMeasurement(
        std::array<double, 2>{tag_pos.pos[0].value(), tag_pos.pos[1].value()},
        tag_pos.variance);
    see_tag_counter_ = 0;
  } else {
    see_tag_counter_++;
  }
  Graph("april_tags/see_tag_counter", see_tag_counter_);

  Graph("april_tags/april_pos_x", tag_pos.pos[0]);
  Graph("april_tags/april_pos_y", tag_pos.pos[1]);
  Graph("april_tags/april_variance", tag_pos.variance);

  if (first_loop) {
    pose_estimator.SetPoint(
        std::array<double, 2>{tag_pos.pos[0].value(), tag_pos.pos[1].value()});
    first_loop = false;
  }

  auto pose_vel = pose_estimator.velocity();
  funkit::robot::swerve::odometry::SwervePose estimated_pose{
      .position = {pdcsu::units::inch_t{pose_estimator.position()[0]},
          pdcsu::units::inch_t{pose_estimator.position()[1]}},
      .bearing = bearing,
      .velocity = {pdcsu::units::fps_t{pose_vel[0]},
          pdcsu::units::fps_t{pose_vel[1]}},
  };

  if (frc::RobotBase::IsSimulation()) {
    estimated_pose.position = odom_output.position;
    estimated_pose.velocity = velocity;
  }

  cached_pose_override_ = GetPreferenceValue_bool("pose_estimator/override");
  if (cached_pose_override_) { estimated_pose = new_pose; }

  Graph("estimated_pose/position_x", estimated_pose.position[0]);
  Graph("estimated_pose/position_y", estimated_pose.position[1]);
  Graph("estimated_pose/velocity_x", estimated_pose.velocity[0]);
  Graph("estimated_pose/velocity_y", estimated_pose.velocity[1]);
  Graph("estimated_pose/variance", pose_estimator.getVariance());

  Graph("readings/position_x", new_pose.position[0]);
  Graph("readings/position_y", new_pose.position[1]);
  Graph("readings/odom_bearing", odom_output.odom_bearing);

  pdcsu::util::math::uVec<pdcsu::units::fps2_t, 2> accl = GetAcceleration();

  Graph("readings/accel_x", accl[0]);
  Graph("readings/accel_y", accl[1]);

  accl = accl.rotate(bearing_offset_);
  pose_estimator.AddAccelerationMeasurement({accl[0].value(), accl[1].value()});

  pdcsu::units::fps2_t accel_mag{std::sqrt(
      accl[0].value() * accl[0].value() + accl[1].value() * accl[1].value())};
  // Graph("readings/accel_mag", accel_mag);

  // Record the current pose if path recording is active
  if (path_logger_.IsRecording()) { path_logger_.RecordPose(estimated_pose); }

  return {new_pose, tag_pos.pos, estimated_pose, yaw_rate, accel_mag,
      see_tag_counter_};
}

pdcsu::util::math::uVec<pdcsu::units::fps_t, 2>
DrivetrainSubsystem::compensateForSteerLag(
    pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> uncompensated) {
  cached_steer_lag_ =
      GetPreferenceValue_unit_type<pdcsu::units::second_t>("steer_lag");
  pdcsu::units::degree_t steer_lag_compensation = pdcsu::units::degree_t{
      -cached_steer_lag_.value() * GetReadings().yaw_rate.value() * 180.0 /
      3.14159265358979323846};

  Graph("target/steer_lag_compensation", steer_lag_compensation);

  return uncompensated.rotate(steer_lag_compensation, true);
}

pdcsu::util::math::uVec<pdcsu::units::fps_t, 2>
DrivetrainSubsystem::accelClampHelper(
    pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity,
    pdcsu::units::fps2_t accel_clamp) {
  if (accel_clamp.value() < 5.0) return velocity;

  auto motor_specs = funkit::control::base::MotorSpecificationPresets::get(
      configs_.module_common_config.motor_types);

  pdcsu::units::amp_t stall_current{motor_specs.stall_current};
  pdcsu::units::ohm_t winding_res = pdcsu::units::volt_t{12.0} / stall_current;

  double max_accel_corr_factor =
      (winding_res /
          (configs_.module_common_config.avg_resistance + winding_res))
          .value();

  pdcsu::units::fps2_t max_accel{configs_.max_accel};
  auto drive_reduction_pdcsu = configs_.module_common_config.drive_reduction;
  auto free_speed_rpm = motor_specs.free_speed;
  auto free_speed_wpi = units::revolutions_per_minute_t{free_speed_rpm.value()};
  auto drive_reduction_wpi =
      units::foot_t{drive_reduction_pdcsu.value()} / units::turn_t{1.0};
  auto accel_buffer_wpi = units::feet_per_second_squared_t{max_accel.value()} /
                          (max_accel_corr_factor * max_accel.value()) *
                          (free_speed_wpi * drive_reduction_wpi);
  pdcsu::units::fps_t accel_buffer{
      pdcsu::units::fps_t{accel_buffer_wpi.to<double>()}};

  auto delta =
      velocity.magnitude() - GetReadings().estimated_pose.velocity.magnitude();
  if (pdcsu::units::u_abs(delta).value() > accel_buffer.value()) {
    double delta_val = delta.value();
    double buffer_val = accel_buffer.value();
    velocity = GetReadings().estimated_pose.velocity +
               pdcsu::util::math::uVec<pdcsu::units::fps_t, 2>{
                   pdcsu::units::fps_t{std::copysign(buffer_val, delta_val)},
                   velocity.angle(true), true};
  }
  return velocity;
}

void DrivetrainSubsystem::WriteVelocitiesHelper(
    pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity,
    pdcsu::units::degps_t angular_velocity, bool cut_excess_steering,
    pdcsu::units::fps_t speed_limit) {
  pdcsu::units::degree_t bearing = GetReadings().pose.bearing;
  auto velocity_compensated = compensateForSteerLag(velocity);

  auto ol_calc_outputs = ol_calculator_.calculate({velocity_compensated,
      angular_velocity, bearing, speed_limit, cut_excess_steering});

  for (int i = 0; i < 4; i++) {
    modules_[i]->SetTarget(SwerveModuleOLControlTarget{
        ol_calc_outputs.drive_outputs[i], ol_calc_outputs.steer_outputs[i]});
  }
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  using namespace funkit::control::config;
  auto steer_genome =
      SubsystemGenomeHelper::LoadGenomePreferences(*this, "steer_genome");

  for (int i = 0; i < 4; i++) {
    modules_[i]->ModifySteerGenome(steer_genome);
  }

  cached_max_omega_cut_ =
      GetPreferenceValue_unit_type<pdcsu::units::degps_t>("max_omega_cut");
  cached_max_speed_ =
      GetPreferenceValue_unit_type<pdcsu::units::fps_t>("max_speed");

  pdcsu::units::degps_t cut_angular_vel = pdcsu::units::u_min(
      pdcsu::units::u_max(target.angular_velocity, -cached_max_omega_cut_),
      cached_max_omega_cut_);

  WriteVelocitiesHelper(accelClampHelper(target.velocity, target.accel_clamp),
      target.cut_excess_steering ? cut_angular_vel : target.angular_velocity,
      target.cut_excess_steering, cached_max_speed_);

  for (int i = 0; i < 4; i++)
    modules_[i]->UpdateHardware();

#ifndef _WIN32
  auto pose = GetReadings().estimated_pose;
  units::inch_t pos_x_wpi(pose.position[0].value());
  units::inch_t pos_y_wpi(
      (funkit::math::FieldPoint::field_size_y - pose.position[1]).value());
  units::degree_t bearing_wpi((degree_t{180} - pose.bearing).value());
  MainField_.SetRobotPose(pos_y_wpi, pos_x_wpi, bearing_wpi);
#endif
}

void DrivetrainSubsystem::StartPathRecording(const std::string& filename) {
  path_logger_.StartRecording(filename);
}

bool DrivetrainSubsystem::StopPathRecording() {
  return path_logger_.StopRecording();
}

bool DrivetrainSubsystem::IsPathRecording() const {
  return path_logger_.IsRecording();
}

void DrivetrainSubsystem::SetFieldObjectPose(const std::string& name,
    pdcsu::util::math::uVec<pdcsu::units::inch_t, 2> position,
    pdcsu::units::degree_t rotation) {
#ifndef _WIN32
  units::inch_t pos_x_wpi(position[0].value());
  units::inch_t pos_y_wpi(
      (funkit::math::FieldPoint::field_size_y - position[1]).value());
  units::degree_t rotation_wpi(rotation.value());
  auto* obj = MainField_.GetObject(name);
  if (obj) { obj->SetPose(pos_y_wpi, pos_x_wpi, rotation_wpi); }
#endif
}

}  // namespace funkit::robot::swerve