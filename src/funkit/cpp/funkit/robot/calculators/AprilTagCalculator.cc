#include "funkit/robot/calculators/AprilTagCalculator.h"

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

ATCalculatorOutput AprilTagCalculator::calculate(ATCalculatorInput input) {
  ATCalculatorOutput output;

  double totalTagWeight = 0;
  double variance = 0;

  std::vector<std::vector<Line>> sight_lines;

  std::vector<funkit::robot::calculators::AprilTagCamera> temp_cameras{};
  for (auto x : constants_.cameras)
    temp_cameras.push_back(x);

  if (constants_.turret_camera.has_value()) {
    auto turret_camera = constants_.turret_camera.value();
    Vector2D cam_offset =
        Vector2D{turret_camera.config.turret_x_offset,
            turret_camera.config.turret_y_offset}
            .rotate(-turret_angle, true) +
        Vector2D{turret_camera.config.x_offset, turret_camera.config.y_offset};
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

  for (size_t i = 0; i < temp_cameras.size(); i++) {
    sight_lines.push_back({});
  }
  int tagsSeen = 0;

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
    if (camera.equiv_turret) {
      bearingAtCapture += turret_angle;
      bearingAtCapture -= turret_vel * (tl - input.bearing_latency);
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
        Vector2D velComp = {
            pdcsu::units::inch_t{vel_avg_x.value() * latency.value() * 12.0},
            pdcsu::units::inch_t{vel_avg_y.value() * latency.value() * 12.0}};

        Vector2D comp_cam_pos = uncomp_cam_pos + velComp;

        pdcsu::units::degree_t comp_angle = (tag_pos - comp_cam_pos).angle();
        Line sight_line{tag_pos, comp_angle};

        Vector2D center_to_cam{
            config.x_offset,
            config.y_offset,
        };

        sight_line.translate(center_to_cam.rotate(bearingAtCapture) * -1.0);
        sight_lines[i].push_back(sight_line);
        if (distances.at(j).value() < 300.0) {
          output.pos = (getPos(bearingAtCapture, tx.at(j), distances.at(j),
                            tags.at(j), config) +
                        velComp);
          output.variance = std::max(
              (input.aprilVarianceCoeff * std::pow(distances.at(j).value(), 3) *
                  std::pow(
                      1 + input.pose.velocity.magnitude().value() / 12.0, 2) *
                  std::pow(1 + input.angular_velocity.value() / 300.0, 2)),
              0.0000000001);

          correction = output.pos - input.pose.position;
          return output;
        }
        double weight = 48.0 / distances.at(j).value();
        output.pos = output.pos + (getPos(bearingAtCapture, tx.at(j),
                                       distances.at(j), tags.at(j), config) +
                                      velComp) *
                                      weight;

        variance +=
            1 /
            std::max(
                (input.aprilVarianceCoeff *
                    std::pow(distances.at(j).value(), 3) *
                    std::pow(
                        1 + input.pose.velocity.magnitude().value() / 12.0, 2) *
                    std::pow(1 + input.angular_velocity.value() / 300.0, 2)),
                0.0000000001);
        totalTagWeight += weight;

        tagsSeen++;
      }
    }
  }

  if (tagsSeen == 1) {
    output.variance = 1 / variance;
    output.pos = output.pos / totalTagWeight;
    correction = output.pos - input.pose.position;
    return output;
  }

  Line first_line = {{pdcsu::units::inch_t{-1}, pdcsu::units::inch_t{-1}},
      pdcsu::units::degree_t{0}};
  for (size_t i = 0; i < temp_cameras.size(); i++) {
    if (sight_lines[i].size() > 0) {
      if (first_line.point[0].value() != -1.0) {
        // Double Cam Triangulation!
        output.pos = first_line.intersect(sight_lines[i].at(0));
        output.variance = std::max(
            (input.triangularVarianceCoeff *
                std::pow(1 + input.pose.velocity.magnitude().value() / 12, 2) *
                std::pow(1 + input.angular_velocity.value() / 300, 2)),
            0.0000000001);  // TODO: make better

        correction = output.pos - input.pose.position;
        return output;
      } else {
        first_line = sight_lines[i].at(0);
      }
    }
  }

  // Single Cam?
  for (size_t i = 0; i < temp_cameras.size(); i++) {
    if (sight_lines[i].size() > 1) {
      // Double Tag, Single Cam Triangulation!
      output.pos = sight_lines[i].at(1).intersect(sight_lines[i].at(0));
      output.variance = std::max(
          (input.triangularVarianceCoeff *
              std::pow(1 + input.pose.velocity.magnitude().value() / 12, 2) *
              std::pow(1 + input.angular_velocity.value() / 300, 2)),
          0.0000000001);  // TODO: make better

      correction = output.pos - input.pose.position;
      return output;
    }
  }
  return {input.pose.position + correction, -1};
}

Vector2D AprilTagCalculator::getPos(pdcsu::units::degree_t bearing,
    pdcsu::units::degree_t theta, pdcsu::units::inch_t distance, int tag,
    const AprilTagCameraConfig& config) {
  double theta_rad = pdcsu::units::radian_t{theta}.value();
  Vector2D cam_to_tag{
      pdcsu::units::inch_t{distance.value() * std::sin(theta_rad)},
      pdcsu::units::inch_t{distance.value() * std::cos(theta_rad)},
  };
  Vector2D center_to_cam{
      config.x_offset,
      config.y_offset,
  };

  Vector2D local_tag_pos = center_to_cam + cam_to_tag;
  local_tag_pos = local_tag_pos.rotate(bearing);

  return {
      constants_.tag_locations[tag].x_pos - local_tag_pos[0],
      constants_.tag_locations[tag].y_pos - local_tag_pos[1],
  };
}
}