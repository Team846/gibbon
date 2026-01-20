#include "subsystems/hardware/DrivetrainConstructor.h"

#include <cmath>

#include "funkit/control/calculators/CircuitResistanceCalculator.h"
#include "funkit/control/config/genome.h"
#include "funkit/math/constants.h"
#include "pdcsu_units.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

namespace swerve = funkit::robot::swerve;
namespace control = funkit::control::base;
namespace control_calc = funkit::control::calculators;
namespace control_config = funkit::control::config;

using namespace pdcsu::units;

namespace {

struct WireConfig {
  pdcsu::units::inch_t length;
  unsigned int num_connectors;
};

struct ModulePorts {
  int cancoder_id;
  int drive_id;
  int steer_id;
};

struct UserSettableValues {
  std::variant<swerve::PigeonConnection, swerve::NavXConnection> imu_connection;
  pdcsu::units::inch_t wheel_diameter;
  double drive_gear_ratio;
  swerve::steer_conv_unit steer_reduction;
  pdcsu::units::inch_t wheel_contact_radius;
  double steer_inertia_coeff;
  double drive_friction;
  double steer_friction;
  struct {
    WireConfig FR;
    WireConfig FL;
    WireConfig BL;
    WireConfig BR;
  } wire_configs;
  struct {
    ModulePorts FR;
    ModulePorts FL;
    ModulePorts BL;
    ModulePorts BR;
  } module_ports;
  std::vector<pdcsu::units::inch_t> camera_x_offsets;
  std::vector<pdcsu::units::inch_t> camera_y_offsets;
  size_t num_cameras;
  std::map<int, funkit::robot::calculators::AprilTagData> april_locations;
};

//--------------------------------------------------------
// User Settable Values. Also set weight and dims in robot_constants.h
//--------------------------------------------------------
UserSettableValues GetUserSettableValues() {
  return UserSettableValues{
      .imu_connection =
          swerve::NavXConnection{
              funkit::robot::swerve::NavXConnectionType::kUSB},
      .wheel_diameter = inch_t{4},
      .drive_gear_ratio = 6.75,
      .steer_reduction = scalar_t{(7_tr / 150_tr).to<double>()},
      .wheel_contact_radius = inch_t{0.4},
      .steer_inertia_coeff = 0.285,
      .drive_friction = 0.02,
      .steer_friction = 0.11,
      .wire_configs = {.FR = {inch_t{15}, 0},
          .FL = {inch_t{15}, 1},
          .BL = {inch_t{35}, 2},
          .BR = {inch_t{42}, 2}},
      .module_ports = {.FR = {ports::drivetrain_::kFRCANCoder_CANID,
                           ports::drivetrain_::kFRDrive_CANID,
                           ports::drivetrain_::kFRSteer_CANID},
          .FL = {ports::drivetrain_::kFLCANCoder_CANID,
              ports::drivetrain_::kFLDrive_CANID,
              ports::drivetrain_::kFLSteer_CANID},
          .BL = {ports::drivetrain_::kBLCANCoder_CANID,
              ports::drivetrain_::kBLDrive_CANID,
              ports::drivetrain_::kBLSteer_CANID},
          .BR = {ports::drivetrain_::kBRCANCoder_CANID,
              ports::drivetrain_::kBRDrive_CANID,
              ports::drivetrain_::kBRSteer_CANID}},
      .camera_x_offsets = {inch_t{-6.25}, inch_t{-4.5}},
      .camera_y_offsets = {inch_t{4}, inch_t{-12.5}},
      .num_cameras = 2,
      .april_locations = {{1, {25.38_u_in, 183.58_u_in}},
          {2, {135.09_u_in, 182.11_u_in}}, {3, {144.85_u_in, 205.87_u_in}},
          {4, {158.85_u_in, 205.87_u_in}}, {5, {182.60_u_in, 182.11_u_in}},
          {6, {292.32_u_in, 183.58_u_in}}, {7, {292.32_u_in, 180.63_u_in}},
          {8, {182.60_u_in, 168.11_u_in}}, {9, {172.85_u_in, 492.88_u_in}},
          {10, {158.85_u_in, 158.34_u_in}}, {11, {135.09_u_in, 168.11_u_in}},
          {12, {25.38_u_in, 180.63_u_in}}, {13, {26.22_u_in, 0.30_u_in}},
          {14, {43.22_u_in, 0.30_u_in}}, {15, {147.47_u_in, 0.32_u_in}},
          {16, {164.47_u_in, 0.32_u_in}}, {17, {292.32_u_in, 467.63_u_in}},
          {18, {182.60_u_in, 469.11_u_in}}, {19, {172.85_u_in, 445.35_u_in}},
          {20, {158.85_u_in, 445.35_u_in}}, {21, {135.09_u_in, 469.11_u_in}},
          {22, {25.38_u_in, 467.63_u_in}}, {23, {25.38_u_in, 470.58_u_in}},
          {24, {135.09_u_in, 483.11_u_in}}, {25, {144.85_u_in, 492.88_u_in}},
          {26, {158.85_u_in, 492.88_u_in}}, {27, {182.60_u_in, 483.11_u_in}},
          {28, {292.32_u_in, 470.58_u_in}}, {29, {291.47_u_in, 650.92_u_in}},
          {30, {274.47_u_in, 650.92_u_in}}, {31, {170.22_u_in, 650.90_u_in}},
          {32, {153.22_u_in, 650.90_u_in}}}};  // TODO: Double check locations
}

//--------------------------------------------------------
// End of User Settable Values
//--------------------------------------------------------

ohm_t CalculateWireResistance(const WireConfig& config) {
  return control_calc::CircuitResistanceCalculator::calculate(
      config.length, control_calc::twelve_gauge, config.num_connectors);
}

swerve::SwerveModuleUniqueConfig CreateModuleConfig(const std::string& name,
    const ModulePorts& ports, const ohm_t& wire_resistance) {
  return swerve::SwerveModuleUniqueConfig{
      name, ports.cancoder_id, ports.drive_id, ports.steer_id, wire_resistance};
}

}  // namespace

DrivetrainConstructor::DrivetrainConstructor()
    : Loggable{"DrivetrainConstructor"} {
  RegisterPreference("drive_motor_current_limit", 160_u_A);
  RegisterPreference("steer_motor_current_limit", 120_u_A);
  RegisterPreference("drive_motor_smart_current_limit", 120_u_A);
  RegisterPreference("steer_motor_smart_current_limit", 80_u_A);

  RegisterPreference("drive_motor_voltage_compensation", 16_u_V);
  RegisterPreference("steer_motor_voltage_compensation", 10_u_V);
}

swerve::DrivetrainConfigs DrivetrainConstructor::getDrivetrainConfigs() {
  const UserSettableValues user_values = GetUserSettableValues();

  const control::MotorMonkeyType mmtype =
      control::MotorMonkeyType::TALON_FX_KRAKENX60;

  const pound_t robot_weight = robot_constants::total_weight;
  const double drive_reduction_value =
      (funkit::math::constants::pi * user_values.wheel_diameter.value()) /
      (user_values.drive_gear_ratio * 12.0);
  const swerve::drive_conv_unit drive_reduction{drive_reduction_value};
  const double drive_reduction_value_for_plant = drive_reduction_value;

  const kgm2_t relative_steer_inertia{
      user_values.steer_inertia_coeff * 0.453592 * 0.0254 * 0.0254};

  const double relative_drive_inertia_value =
      (robot_weight.value() / 4.0 *
          (user_values.wheel_diameter.value() *
              user_values.wheel_diameter.value() / 4.0) /
          (user_values.drive_gear_ratio * user_values.drive_gear_ratio)) *
      0.453592 * 0.0254 * 0.0254;
  const kgm2_t relative_drive_inertia{relative_drive_inertia_value};

  meter_t effective_torque_radius{
      (user_values.wheel_diameter / 2.0) / user_values.drive_gear_ratio};
  auto motor_specs = control::MotorSpecificationPresets::get(mmtype);
  nm_t stall_torque{motor_specs.stall_torque};
  newton_t max_force_per_wheel{stall_torque / effective_torque_radius};
  kg_t robot_mass{robot_weight.value() * 0.453592};
  auto max_accel_mps2 = (newton_t{4.0} * max_force_per_wheel) / robot_mass;

  pound_t quarter_robot_weight{robot_weight.value() * 0.25};
  const double steer_load_factor_value =
      (user_values.wheel_contact_radius.value() *
          user_values.steer_reduction.value() * quarter_robot_weight.value()) *
      0.112985 * 0.453592 * 0.0254;
  const nm_t steer_load_factor{steer_load_factor_value};

  const ohm_t wire_resistance_FR =
      CalculateWireResistance(user_values.wire_configs.FR);
  const ohm_t wire_resistance_FL =
      CalculateWireResistance(user_values.wire_configs.FL);
  const ohm_t wire_resistance_BL =
      CalculateWireResistance(user_values.wire_configs.BL);
  const ohm_t wire_resistance_BR =
      CalculateWireResistance(user_values.wire_configs.BR);

  auto total_resistance = wire_resistance_FR + wire_resistance_FL +
                          wire_resistance_BL + wire_resistance_BR;
  const ohm_t avg_resistance{total_resistance.value() / 4.0};

  const swerve::SwerveModuleUniqueConfig FR_config =
      CreateModuleConfig("FR", user_values.module_ports.FR, wire_resistance_FR);
  const swerve::SwerveModuleUniqueConfig FL_config =
      CreateModuleConfig("FL", user_values.module_ports.FL, wire_resistance_FL);
  const swerve::SwerveModuleUniqueConfig BL_config =
      CreateModuleConfig("BL", user_values.module_ports.BL, wire_resistance_BL);
  const swerve::SwerveModuleUniqueConfig BR_config =
      CreateModuleConfig("BR", user_values.module_ports.BR, wire_resistance_BR);

  control_config::MotorConstructionParameters drive_params;
  drive_params.can_id = 999;
  drive_params.inverted = false;
  drive_params.bus = "";

  control_config::MotorConstructionParameters steer_params;
  steer_params.can_id = 999;
  steer_params.inverted = false;
  steer_params.bus = "";

  using namespace pdcsu::units;
  using namespace pdcsu::util;

  auto drive_genome_voltage =
      GetPreferenceValue_unit_type<volt_t>("drive_motor_voltage_compensation");
  DefBLDC def_bldc{motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, drive_genome_voltage};

  UnitDivision<radian_t, meter_t> drive_gear_ratio{
      (drive_reduction_value_for_plant * 12.0) / M_PI};
  pdcsu::util::DefLinearSys drive_plant{def_bldc, 1, drive_gear_ratio,
      1.0_u_mps2, kg_t{0.0}, newton_t{user_values.drive_friction},
      UnitDivision<newton_t, rpm_t>{0.0}, ms_t{20.0}, avg_resistance};

  scalar_t steer_gear_ratio{user_values.steer_reduction.value()};
  pdcsu::util::DefArmSys steer_plant{def_bldc, 1, steer_gear_ratio,
      [](radian_t, radps_t) -> nm_t { return nm_t{0.0}; },
      relative_steer_inertia,
      nm_t{user_values.steer_friction * motor_specs.stall_torque.value()},
      UnitDivision<nm_t, rpm_t>{0.0}, ms_t{20.0}, avg_resistance};

  swerve::SwerveModuleCommonConfig module_common_config{
      .drive_params = drive_params,
      .steer_params = steer_params,
      .motor_types = mmtype,
      .steer_reduction = user_values.steer_reduction,
      .drive_reduction = drive_reduction,
      .avg_resistance = avg_resistance,
      .circuit_resistance = avg_resistance,
      .steer_load_factor = steer_load_factor,
      .drive_plant = drive_plant,
      .steer_plant = steer_plant,
      .bus = ""};

  const std::vector<inch_t>& camera_x_offsets = user_values.camera_x_offsets;
  const std::vector<inch_t>& camera_y_offsets = user_values.camera_y_offsets;

  swerve::DrivetrainConfigs configs{
      .imu_connection = user_values.imu_connection,
      .module_common_config = module_common_config,
      .module_unique_configs = {FR_config, FL_config, BL_config, BR_config},
      .wheelbase_horizontal_dim = robot_constants::base::wheelbase_x,
      .wheelbase_forward_dim = robot_constants::base::wheelbase_y,
      .max_speed = fps_t{0.0},
      .camera_x_offsets = camera_x_offsets,
      .camera_y_offsets = camera_y_offsets,
      .cams = user_values.num_cameras,
      .april_locations = user_values.april_locations,
      .max_accel = fps2_t{max_accel_mps2.value() * 3.28084}};

  return configs;
}