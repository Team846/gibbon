#include "funkit/robot/swerve/control/swerve_ol_calculator.h"

#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::control {

SwerveOpenLoopCalculatorOutput SwerveOpenLoopCalculator::calculate(
    SwerveOpenLoopCalculatorInputs inputs) {
  std::pair<double, double> kModuleLocationSigns[4] = {
      {+0.5, +0.5},  // FR
      {-0.5, +0.5},  // FL
      {-0.5, -0.5},  // BL
      {+0.5, -0.5},  // BR
  };

  std::array<pdcsu::util::math::uVec<pdcsu::units::fps_t, 2>, 4> module_targets;

  pdcsu::units::inch_t radius{
      std::hypot(constants_.wheelbase_horizontal_dim.value(),
          constants_.wheelbase_forward_dim.value())};

  auto calculate_module_targets = [&](pdcsu::units::radps_t rotation) {
    for (int i = 0; i < 4; i++) {
      pdcsu::util::math::uVec<pdcsu::units::inch_t, 2> location{
          pdcsu::units::inch_t{kModuleLocationSigns[i].first *
                               constants_.wheelbase_horizontal_dim.value()},
          pdcsu::units::inch_t{kModuleLocationSigns[i].second *
                               constants_.wheelbase_forward_dim.value()}};

      pdcsu::units::degree_t rot_direction =
          location.angle(false) - pdcsu::units::degree_t{90.0};

      double rot_rad = pdcsu::units::radian_t{rot_direction}.value();
      double rot_vel_radps = rotation.value();
      double radius_in = radius.value();
      double radius_ft = radius_in / 12.0;

      pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> rotation_vector{
          pdcsu::units::fps_t{rot_vel_radps * std::cos(rot_rad) * radius_ft},
          pdcsu::units::fps_t{rot_vel_radps * std::sin(rot_rad) * radius_ft}};

      module_targets[i] =
          inputs.translation_target.rotate(-inputs.bearing, true) +
          rotation_vector;
    }
  };

  auto get_max_mag = [&]() {
    pdcsu::units::fps_t max_mag{0};
    for (int i = 0; i < 4; i++) {
      if (module_targets[i].magnitude().value() > max_mag.value()) {
        max_mag = module_targets[i].magnitude();
      }
    }
    return max_mag;
  };

  auto rescale_outputs = [&](pdcsu::units::fps_t max_mag) {
    double scale = inputs.max_speed.value() / max_mag.value();
    for (int i = 0; i < 4; i++) {
      module_targets[i] *= scale;
    }
  };

  calculate_module_targets(inputs.rotation_target);

  if (inputs.cut_excess_steering) {
    pdcsu::units::radps_t rotation_target = inputs.rotation_target;
    pdcsu::units::radps_t abs_rotation_target =
        pdcsu::units::u_abs(inputs.rotation_target);
    pdcsu::units::fps_t max_mag;

    do {
      calculate_module_targets(rotation_target);

      max_mag = get_max_mag();

      if (max_mag.value() > inputs.max_speed.value()) {
        abs_rotation_target = pdcsu::units::radps_t{
            abs_rotation_target.value() - constants_.rotation_iter_dec.value()};
        rotation_target =
            pdcsu::units::u_copysign(abs_rotation_target, rotation_target);
        if (abs_rotation_target.value() < 0) {
          calculate_module_targets(pdcsu::units::radps_t{0});
          max_mag = get_max_mag();
          if (max_mag.value() > inputs.max_speed.value()) {
            rescale_outputs(get_max_mag());
          }
          break;
        }
      } else {
        break;
      }
    } while (max_mag.value() > inputs.max_speed.value());
  } else {
    pdcsu::units::fps_t max_mag = get_max_mag();
    if (max_mag.value() > inputs.max_speed.value()) {
      rescale_outputs(max_mag);
    }
  }

  SwerveOpenLoopCalculatorOutput output{};
  for (int i = 0; i < 4; i++) {
    output.drive_outputs[i] = module_targets[i].magnitude();
    output.steer_outputs[i] =
        module_targets[i].angle(true);
  }
  return output;
}

}  // namespace funkit::robot::swerve::control