#include "funkit/robot/calculators/AprilTagCalculator.h"

#include <algorithm>
#include <iostream>

#include "funkit/math/collection.h"
#include "funkit/math/fieldpoints.h"
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
  if (time <= odom_history_.front().time) {
    return odom_history_.front().position;
  }
  if (time >= odom_history_.back().time) {
    return odom_history_.back().position;
  }
  for (size_t i = 0; i + 1 < odom_history_.size(); i++) {
    const auto& a = odom_history_[i];
    const auto& b = odom_history_[i + 1];
    if (time >= a.time && time <= b.time) {
      second_t denom = b.time - a.time;
      double frac = (denom <= 0_s_) ? 0.0 : ((time - a.time) / denom).value();
      return Vector2D{(1.0 - frac) * a.position[0] + frac * b.position[0],
          (1.0 - frac) * a.position[1] + frac * b.position[1]};
    }
  }
  return odom_history_.back().position;
}

pdcsu::units::degree_t AprilTagCalculator::InterpolateRobotBearing(
    pdcsu::units::second_t time) const {
  if (odom_history_.empty()) { return 0_deg_; }
  if (odom_history_.size() == 1) { return odom_history_.front().bearing; }
  if (time <= odom_history_.front().time) {
    return odom_history_.front().bearing;
  }
  if (time >= odom_history_.back().time) {
    return odom_history_.back().bearing;
  }
  for (size_t i = 0; i + 1 < odom_history_.size(); i++) {
    const auto& a = odom_history_[i];
    const auto& b = odom_history_[i + 1];
    if (time >= a.time && time <= b.time) {
      second_t denom = b.time - a.time;
      auto frac = (denom <= 0_s_) ? 0.0_u_ : (time - a.time) / denom;
      degree_t diff = b.bearing - a.bearing;
      if (diff > 180.0_deg_) { diff -= 360.0_deg_; }
      if (diff < -180.0_deg_) { diff += 360.0_deg_; }
      return a.bearing + frac * diff;
    }
  }
  return odom_history_.back().bearing;
}

pdcsu::units::degree_t AprilTagCalculator::InterpolateTurretAngle(
    pdcsu::units::second_t time) const {
  if (odom_history_.empty()) { return 0_deg_; }
  if (odom_history_.size() == 1) { return odom_history_.front().turret_angle; }
  if (time <= odom_history_.front().time) {
    return odom_history_.front().turret_angle;
  }
  if (time >= odom_history_.back().time) {
    return odom_history_.back().turret_angle;
  }
  for (size_t i = 0; i + 1 < odom_history_.size(); i++) {
    const auto& a = odom_history_[i];
    const auto& b = odom_history_[i + 1];
    if (time >= a.time && time <= b.time) {
      second_t denom = b.time - a.time;
      double frac = (denom <= 0_s_) ? 0.0 : ((time - a.time) / denom).value();
      degree_t diff = b.turret_angle - a.turret_angle;
      if (diff > 180.0_deg_) { diff -= 360.0_deg_; }
      if (diff < -180.0_deg_) { diff += 360.0_deg_; }
      return a.turret_angle + frac * diff;
    }
  }
  return odom_history_.back().turret_angle;
}

ATCalculatorOutput AprilTagCalculator::calculate(ATCalculatorInput input) {
  ATCalculatorOutput output{};
  pdcsu::units::second_t now = funkit::wpilib::CurrentFPGATime();
  AddToHistory(now, input.pose.position, input.pose.bearing,
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
  std::vector<Vector2D> cam_vectors{};
  std::vector<Vector2D> field_positions{};
  std::vector<double> bearing_weights{};

  for (size_t i = 0; i < temp_cameras.size(); i++) {
    const auto& camera = temp_cameras[i];
    auto cam_table = camera.table;
    const auto& config = camera.config;

    pdcsu::units::second_t delay =
        now - pdcsu::units::second_t{
                  cam_table->GetEntry("tl").GetLastChange() / 1000000.0};
    pdcsu::units::second_t tl =
        pdcsu::units::second_t(cam_table->GetNumber("tl", -1));

    if (delay > 3_s_) { output.camera_disconnect = true; }

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
    pdcsu::units::second_t effective_latency = tl + fudge + delay;
    if (effective_latency > 200_ms_) { continue; }
    pdcsu::units::second_t capture_time = now - effective_latency;

    std::vector<double> tags = cam_table->GetNumberArray("tags", {});
    pdcsu::units::degree_t imuBearingAtCapture = InterpolateRobotBearing(
        capture_time);  // input.pose.bearing - input.angular_velocity *
                        // effective_latency;
    pdcsu::units::degree_t bearingAtCapture;
    if (camera.equiv_turret) {
      bearingAtCapture =
          imuBearingAtCapture +
          InterpolateTurretAngle(
              capture_time);  // turret_angle + turret_vel * effective_latency;
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
        Vector2D cam_to_tag_field_frame{
            distances.at(j), tx.at(j) + bearingAtCapture, true};
        Vector2D tag_pos{constants_.tag_locations[tags.at(j)].x_pos,
            constants_.tag_locations[tags.at(j)].y_pos};
        Vector2D uncomp_cam_pos = tag_pos - cam_to_tag_field_frame;

        Vector2D center_to_cam;
        if (camera.equiv_turret && constants_.turret_camera.has_value()) {
          const auto& tc = constants_.turret_camera.value();
          pdcsu::units::degree_t turret_at_capture =
              InterpolateTurretAngle(capture_time);
          Vector2D cam_offset_at_capture =
              Vector2D{tc.config.turret_x_offset, tc.config.turret_y_offset} +
              Vector2D{tc.config.x_offset, tc.config.y_offset}.rotate(
                  turret_at_capture, true);
          center_to_cam =
              cam_offset_at_capture.rotate(imuBearingAtCapture, true);
        } else {
          center_to_cam = Vector2D{config.x_offset, config.y_offset}.rotate(
              imuBearingAtCapture, true);
        }

        Vector2D position_at_capture = uncomp_cam_pos - center_to_cam;

        Vector2D position_at_capture_from_history =
            InterpolatePosition(capture_time);
        Vector2D position_map =
            input.pose.position - position_at_capture_from_history;
        Vector2D position_compensated = position_at_capture + position_map;

        if (distances.at(j).value() < 300.0) {
          m_positions.push_back(position_compensated);
          inch_t distance_i = distances.at(j);
          double var_i =
              input.aprilVarianceCoeff *
                  (std::sqrt(distances.at(j).value() + 1.0) / 30.0 +
                      input.pose.velocity.magnitude().value() / 12.0 +
                      (input.angular_velocity +
                          (camera.equiv_turret ? 4.0 * turret_vel : 0_degps_))
                              .value() *
                          std::sqrt(distance_i.value()) / 25.0) +
              0.5;
          if (camera.equiv_turret) {
            var_i *= 2.0;
          } else {
            var_i *= 4.0;
          }

          pure_variances.push_back(var_i);

          Vector2D cam_to_tag_cam_frame{distances.at(j),
              tx.at(j) + (camera.equiv_turret
                                 ? InterpolateTurretAngle(capture_time)
                                 : 0_deg_),
              true};
          cam_vectors.push_back(cam_to_tag_cam_frame);
          field_positions.push_back(tag_pos);

          double w_bearing = 1.0 / std::max(var_i, 1e-9);
          bearing_weights.push_back(w_bearing);
        }
      }
    }
  }

  if (m_positions.size() == 0) {
    output.pos = input.pose.position;
    output.variance = -1;
    output.bearing_from_tags = 0_deg_;
    output.bearing_from_tags_valid = false;
    return output;
  }

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

  output.bearing_from_tags_valid = false;
  output.bearing_from_tags = 0_deg_;
  if (cam_vectors.size() >= 2) {
    std::vector<pdcsu::units::degree_t> pair_bearings{};
    std::vector<double> pair_weights{};

    for (size_t a = 0; a + 1 < cam_vectors.size(); ++a) {
      for (size_t b = a + 1; b < cam_vectors.size(); ++b) {
        Vector2D field_vec = field_positions[b] - field_positions[a];
        Vector2D cam_vec = cam_vectors[b] - cam_vectors[a];

        auto field_len = field_vec.magnitude();
        auto cam_len = cam_vec.magnitude();
        if (field_len.value() <= 40.0 || cam_len.value() <= 40.0) { continue; }

        degree_t field_angle = field_vec.angle(true);
        degree_t cam_angle = cam_vec.angle(true);

        degree_t delta = field_angle - cam_angle;
        if (delta > 180.0_deg_) { delta -= 360.0_deg_; }
        if (delta < -180.0_deg_) { delta += 360.0_deg_; }

        double w = (field_len.value() * cam_len.value()) *
                   (0.5 * (bearing_weights[a] + bearing_weights[b]));
        if (w <= 0.0) { continue; }

        pair_bearings.push_back(delta);
        pair_weights.push_back(w);
      }
    }

    if (!pair_bearings.empty()) {
      double sum_w_bearing = 0.0;
      double sum_cos = 0.0;
      double sum_sin = 0.0;
      for (size_t i = 0; i < pair_bearings.size(); i++) {
        double w = pair_weights[i];
        sum_w_bearing += w;
        sum_cos += w * u_cos(pair_bearings[i]);
        sum_sin += w * u_sin(pair_bearings[i]);
      }
      if (sum_w_bearing > 0.0) {
        double mean_rad = std::atan2(sum_sin, sum_cos);
        output.bearing_from_tags = pdcsu::units::radian_t{mean_rad};
        output.bearing_from_tags_valid = true;
      }
    }
  }

  if (output.pos[0] < 0_in_ || output.pos[1] < 0_in_ ||
      output.pos[0] > funkit::math::FieldPoint::field_size_x ||
      output.pos[1] > funkit::math::FieldPoint::field_size_y) {
    output.variance *= 5.0;
  }

  correction = output.pos - input.pose.position;

  return output;
}
}
