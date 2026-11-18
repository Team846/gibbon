#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "funkit/math/calculator.h"
#include "funkit/robot/swerve/odometry/swerve_pose.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::calculators {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

struct ATCalculatorInput {
  funkit::robot::swerve::odometry::SwervePose pose;
  pdcsu::units::degps_t angular_velocity;

  double aprilVarianceCoeff;
  std::map<size_t, pdcsu::units::second_t> fudge_latency;

  bool priority_tags_blue = false;
  double priority_variance_scale = 1.0;

  pdcsu::units::degree_t heading_offset{0};

  bool use_synced_capture = false;
  pdcsu::units::second_t synced_capture_delay{0};
};

struct TagObservation {
  Vector2D tag_pos;
  Vector2D cam_offset;
  Vector2D odom_since_capture;
  pdcsu::units::degree_t gyro_bearing_at_capture;
  pdcsu::units::degree_t yaw_offset;
  pdcsu::units::degree_t tx;
  pdcsu::units::inch_t distance;
  double variance;
};

struct AprilTagCameraResult {
  Vector2D pos;
  double variance = -1.0;
  int tag_count = 0;
};

struct AprilTagCameraTiming {
  double latency_ms = -1.0;
  double synced_latency_ms = -1.0;
  double sync_rtt_ms = -1.0;
  bool used_synced = false;
};

struct ATCalculatorOutput {
  Vector2D pos;
  double variance;
  pdcsu::units::degree_t bearing_from_tags;
  bool bearing_from_tags_valid;
  bool camera_disconnect = false;
  int new_frames = 0;
  std::map<size_t, AprilTagCameraResult> camera_results{};
  std::map<size_t, AprilTagCameraTiming> camera_timing{};
  std::vector<TagObservation> observations{};
};

struct AprilTagData {
  pdcsu::units::inch_t x_pos;
  pdcsu::units::inch_t y_pos;
};

struct AprilTagCameraConfig {
  size_t camera_id;
  pdcsu::units::inch_t x_offset;
  pdcsu::units::inch_t y_offset;
};

struct AprilTagCamera {
  AprilTagCameraConfig config;
  std::shared_ptr<nt::NetworkTable> table;
  bool equiv_turret = false;
};

struct TurretTagCameraConfig {
  size_t camera_id;
  pdcsu::units::inch_t turret_x_offset;
  pdcsu::units::inch_t turret_y_offset;
  pdcsu::units::inch_t x_offset;
  pdcsu::units::inch_t y_offset;
};

struct TurretTagCamera {
  TurretTagCameraConfig config;
  std::shared_ptr<nt::NetworkTable> table;
};

struct TagFrame {
  pdcsu::units::second_t arrival;
  std::vector<double> result;
};

struct TagFrameQueue {
  nt::DoubleArraySubscriber subscriber;
  nt::DoubleSubscriber sync_rtt;
  std::mutex mutex;
  std::vector<TagFrame> pending;
  std::vector<TagFrame> draining;
  pdcsu::units::second_t last_arrival{0};
};

struct ATCalculatorConstants {
  std::map<size_t, AprilTagData> tag_locations;
  std::vector<AprilTagCamera> cameras;
  std::optional<TurretTagCamera> turret_camera;
  std::set<size_t> priority_tags_red{};
  std::set<size_t> priority_tags_blue{};
};

class AprilTagCalculator : public funkit::math::Calculator<ATCalculatorInput,
                               ATCalculatorOutput, ATCalculatorConstants> {
public:
  AprilTagCalculator() {};

  ATCalculatorOutput calculate(ATCalculatorInput input) override;

  static pdcsu::units::degree_t turret_angle;
  static pdcsu::units::degps_t turret_vel;

  static inch_t view_turret_off_x;
  static inch_t view_turret_off_y;
  static degree_t view_full_turret_angle;

  struct HistoryEntry {
    pdcsu::units::second_t time;
    Vector2D position;
    pdcsu::units::degree_t bearing;
    pdcsu::units::degree_t turret_angle;
  };
  std::deque<HistoryEntry> odom_history_{};
  static constexpr size_t kMaxHistorySize = 500;

  void AddToHistory(pdcsu::units::second_t time, Vector2D position,
      pdcsu::units::degree_t bearing, pdcsu::units::degree_t turret_angle);
  Vector2D InterpolatePosition(pdcsu::units::second_t time) const;
  pdcsu::units::degree_t InterpolateRobotBearing(
      pdcsu::units::second_t time) const;
  pdcsu::units::degree_t InterpolateTurretAngle(
      pdcsu::units::second_t time) const;

  Vector2D correction;

private:
  static constexpr size_t kMaxPendingFrames = 32;
  TagFrameQueue& GetFrameQueue(const AprilTagCamera& camera);
  std::map<size_t, std::shared_ptr<TagFrameQueue>> frame_queues_{};
};
}