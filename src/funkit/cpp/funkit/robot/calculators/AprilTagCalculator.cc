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

void AprilTagCalculator::AddToHistory(pdcsu::units::second_t time,
    Vector2D position, pdcsu::units::degree_t bearing,
    pdcsu::units::degree_t turret_angle) {
  odom_history_.push_back({time, position, bearing, turret_angle});
  while (odom_history_.size() > kMaxHistorySize) {
    odom_history_.pop_front();
  }
}

Vector2D AprilTagCalculator::InterpolatePosition(
    pdcsu::units::second_t time) const {
  if (odom_history_.empty()) { return Vector2D{0_in_, 0_in_}; }
  if (odom_history_.size() == 1) { return odom_history_.front().position; }
  if (time.value() <= odom_history_.front().time.value()) {
    return odom_history_.front().position;
  }
  if (time.value() >= odom_history_.back().time.value()) {
    return odom_history_.back().position;
  }
  for (size_t i = 0; i + 1 < odom_history_.size(); i++) {
    const auto& a = odom_history_[i];
    const auto& b = odom_history_[i + 1];
    if (time.value() >= a.time.value() && time.value() <= b.time.value()) {
      double denom = b.time.value() - a.time.value();
      double frac =
          (denom <= 0) ? 0.0 : (time.value() - a.time.value()) / denom;
      return Vector2D{inch_t{(1.0 - frac) * a.position[0].value() +
                             frac * b.position[0].value()},
          inch_t{(1.0 - frac) * a.position[1].value() +
                 frac * b.position[1].value()}};
    }
  }
  return odom_history_.back().position;
}

// pdcsu::units::degree_t AprilTagCalculator::InterpolateRobotBearing(
//     pdcsu::units::second_t time) const {
//   if (odom_history_.empty()) { return 0_deg_; }
//   if (odom_history_.size() == 1) { return odom_history_.front().bearing; }
//   if (time.value() <= odom_history_.front().time.value()) {
//     return odom_history_.front().bearing;
//   }
//   if (time.value() >= odom_history_.back().time.value()) {
//     return odom_history_.back().bearing;
//   }
//   for (size_t i = 0; i + 1 < odom_history_.size(); i++) {
//     const auto& a = odom_history_[i];
//     const auto& b = odom_history_[i + 1];
//     if (time >= a.time && time <= b.time) {
//       second_t denom = b.time - a.time;
//       auto frac =
//           (denom <= 0_s_) ? 0.0_u_ : (time - a.time) / denom;
//       degree_t diff = b.bearing - a.bearing;
//       if (diff > 180.0) { diff -= 360.0; }
//       if (diff < -180.0) { diff += 360.0; }
//       return degree_t{a.bearing.value() + frac * diff};
//     }
//   }
//   return odom_history_.back().bearing;
// }

// pdcsu::units::degree_t AprilTagCalculator::InterpolateTurretAngle(
//     pdcsu::units::second_t time) const {
//   if (odom_history_.empty()) { return 0_deg_; }
//   if (odom_history_.size() == 1) { return odom_history_.front().turret_angle;
//   } if (time.value() <= odom_history_.front().time.value()) {
//     return odom_history_.front().turret_angle;
//   }
//   if (time.value() >= odom_history_.back().time.value()) {
//     return odom_history_.back().turret_angle;
//   }
//   for (size_t i = 0; i + 1 < odom_history_.size(); i++) {
//     const auto& a = odom_history_[i];
//     const auto& b = odom_history_[i + 1];
//     if (time.value() >= a.time.value() && time.value() <= b.time.value()) {
//       double denom = b.time.value() - a.time.value();
//       double frac =
//           (denom <= 0) ? 0.0 : (time.value() - a.time.value()) / denom;
//       double diff = b.turret_angle.value() - a.turret_angle.value();
//       if (diff > 180.0) { diff -= 360.0; }
//       if (diff < -180.0) { diff += 360.0; }
//       return degree_t{a.turret_angle.value() + frac * diff};
//     }
//   }
//   return odom_history_.back().turret_angle;
// }

ATCalculatorOutput AprilTagCalculator::calculate(ATCalculatorInput input) {
  ATCalculatorOutput output;
  pdcsu::units::second_t now = funkit::wpilib::CurrentFPGATime();
  AddToHistory(now, input.odom_pose.position, input.odom_pose.bearing,
      AprilTagCalculator::turret_angle);

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
        now - pdcsu::units::second_t{
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

    pdcsu::units::second_t fudge{0};
    if (input.fudge_latency.contains(config.camera_id)) {
      fudge = input.fudge_latency.at(config.camera_id);
    }
    pdcsu::units::second_t effective_latency = tl + fudge;
    pdcsu::units::second_t capture_time = now - effective_latency;

    std::vector<double> tags = cam_table->GetNumberArray("tags", {});
    pdcsu::units::degree_t imuBearingAtCapture =
        input.pose.bearing - input.angular_velocity * effective_latency;
    pdcsu::units::degree_t bearingAtCapture;
    if (camera.equiv_turret) {
      bearingAtCapture =
          imuBearingAtCapture + turret_angle + turret_vel * effective_latency;
      view_full_turret_angle = bearingAtCapture;
    } else {
      bearingAtCapture = imuBearingAtCapture;
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

        Vector2D center_to_cam =
            Vector2D{
                config.x_offset,
                config.y_offset,
            }
                .rotate(imuBearingAtCapture, true);

        Vector2D position_at_capture = uncomp_cam_pos - center_to_cam;

        Vector2D position_at_capture_from_history =
            InterpolatePosition(capture_time);
        Vector2D position_map =
            input.odom_pose.position - position_at_capture_from_history;
        Vector2D position_compensated = position_at_capture + position_map;

        if (distances.at(j).value() < 300.0) {
          m_positions.push_back(position_compensated);
          inch_t distance_i = distances.at(j);
          double var_i =
              input.aprilVarianceCoeff *
                  (std::sqrt(distances.at(j).value() + 1.0) / 30.0 +
                      input.pose.velocity.magnitude().value() / 12.0 +
                      (input.angular_velocity +
                          (camera.equiv_turret ? turret_vel : 0_degps_))
                              .value() *
                          std::sqrt(distance_i.value()) / 25.0) +
              0.5;
          if (camera.equiv_turret) { var_i *= 2.0; }
          pure_variances.push_back(var_i);
        }
      }
    }
  }

  if (m_positions.size() == 0) { return {input.odom_pose.position, -1}; }

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
  correction = output.pos - input.odom_pose.position;
  return output;
}
}
