#include "funkit/robot/calculators/AprilTagCalculator.h"

#include <algorithm>
#include <iostream>

#include "funkit/math/collection.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_units.h"

namespace funkit::robot::calculators {

pdcsu::units::degree_t AprilTagCalculator::turret_angle = 0.0_deg_;
pdcsu::units::degps_t AprilTagCalculator::turret_vel = 0.0_degps_;
inch_t AprilTagCalculator::view_turret_off_x = 0.0_in_;
inch_t AprilTagCalculator::view_turret_off_y = 0.0_in_;
degree_t AprilTagCalculator::view_full_turret_angle = 0_deg_;

ATCalculatorOutput AprilTagCalculator::calculate(ATCalculatorInput input) {
  ATCalculatorOutput output;

  std::vector<funkit::robot::calculators::AprilTagCamera> temp_cameras{};
  for (auto x : constants_.cameras)
    temp_cameras.push_back(x);

  if (constants_.turret_camera.has_value()) {
    auto turret_camera = constants_.turret_camera.value();
    Vector2D cam_offset =
        Vector2D{turret_camera.config.turret_x_offset,
            turret_camera.config.turret_y_offset} +
        Vector2D{turret_camera.config.x_offset, turret_camera.config.y_offset}
            .rotate(turret_angle, true);
    AprilTagCalculator::view_turret_off_x = cam_offset[0];
    AprilTagCalculator::view_turret_off_y = cam_offset[1];
    AprilTagCamera turret_tag_camera{
        .config =
            {
                .camera_id = turret_camera.config.camera_id,
                .x_offset = cam_offset[0],
                .y_offset = cam_offset[1],
            },
        .table = turret_camera.table,
        .equiv_turret = true,
    };
    temp_cameras.push_back(turret_tag_camera);
  }

  std::vector<Vector2D> m_positions{};
  std::vector<double> pure_variances{};

  for (size_t i = 0; i < temp_cameras.size(); i++) {
    const auto& camera = temp_cameras[i];
    auto cam_table = camera.table;
    const auto& config = camera.config;

    pdcsu::units::second_t delay =
        funkit::wpilib::CurrentFPGATime() -
        pdcsu::units::second_t{
            cam_table->GetEntry("tl").GetLastChange() / 1000000.0};
    pdcsu::units::second_t tl =
        pdcsu::units::second_t(cam_table->GetNumber("tl", -1));
    if (delay > 3.5 * funkit::robot::GenericRobot::kPeriod) { continue; }

    std::vector<double> tx_nums = cam_table->GetNumberArray("tx", {});
    std::vector<double> distances_num =
        cam_table->GetNumberArray("distances", {});
    std::vector<pdcsu::units::degree_t> tx;
    std::vector<pdcsu::units::inch_t> distances;
    for (double tx_num : tx_nums) {
      tx.push_back(pdcsu::units::degree_t{tx_num});
    };
    for (double distance_num : distances_num) {
      distances.push_back(pdcsu::units::inch_t{distance_num});
    };

    std::vector<double> tags = cam_table->GetNumberArray("tags", {});
    pdcsu::units::degree_t bearingAtCapture =
        input.pose.bearing -
        input.angular_velocity * (tl - input.bearing_latency);
    degree_t imuBearingAtCapture = bearingAtCapture;
    if (camera.equiv_turret) {
      bearingAtCapture += turret_angle;
      bearingAtCapture -= turret_vel * (tl - input.bearing_latency);
      if (u_abs(turret_vel) > 45_degps_) {
        continue;
      }
      view_full_turret_angle = bearingAtCapture;
    }

    if (!(tags.size() == distances.size() && tags.size() == tx.size())) {
      continue;
    }
    for (size_t j = 0; j < tags.size(); j++) {
      if (constants_.tag_locations.contains(tags.at(j)) &&
          distances.at(j).value() < 300.0) {
        Vector2D cam_to_tag{distances.at(j), tx.at(j) + bearingAtCapture, true};
        Vector2D tag_pos{constants_.tag_locations[tags.at(j)].x_pos,
            constants_.tag_locations[tags.at(j)].y_pos};
        Vector2D uncomp_cam_pos = tag_pos - cam_to_tag;

        auto vel_avg_x =
            (input.pose.velocity[0] + input.old_pose.velocity[0]) / 2.0;
        auto vel_avg_y =
            (input.pose.velocity[1] + input.old_pose.velocity[1]) / 2.0;
        auto latency = tl;
        if (input.fudge_latency.contains(config.camera_id)) {
          latency += input.fudge_latency.at(config.camera_id);
        }
        Vector2D velComp = {vel_avg_x * latency, vel_avg_y * latency};

        Vector2D comp_cam_pos = uncomp_cam_pos + velComp;

        Vector2D center_to_cam =
            Vector2D{
                config.x_offset,
                config.y_offset,
            }
                .rotate(imuBearingAtCapture, true);

        Vector2D comp_center_pos = comp_cam_pos - center_to_cam;

        if (distances.at(j).value() < 300.0) {
          m_positions.push_back(comp_center_pos);
          inch_t distance_i = distances.at(j);
          double var_i =
              input.aprilVarianceCoeff *
                  (std::sqrt(distances.at(j).value() + 1.0) / 30.0 +
                      input.pose.velocity.magnitude().value() / 12.0 +
                      input.angular_velocity.value() *
                          std::sqrt(distance_i.value()) / 30.0) +
              0.5;
          if (camera.equiv_turret) { var_i *= 2.0; }
          pure_variances.push_back(var_i);
        }
      }
    }
  }

  if (m_positions.size() == 0) { return {input.pose.position, -1}; }

  double sum_w = 0;
  Vector2D sum_wp{0_in_, 0_in_};
  for (size_t i = 0; i < m_positions.size(); i++) {
    double w = 1.0 / std::max(pure_variances[i], 1e-9);
    sum_w += w;
    sum_wp[0] += w * m_positions[i][0];
    sum_wp[1] += w * m_positions[i][1];
  }
  output.pos = sum_wp / sum_w;

  output.variance = 1.0 / sum_w;
  correction = output.pos - input.pose.position;
  return output;
}
}