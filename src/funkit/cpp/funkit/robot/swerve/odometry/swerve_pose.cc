#include "funkit/robot/swerve/odometry/swerve_pose.h"

namespace funkit::robot::swerve::odometry {

SwervePose SwervePose::rotate(pdcsu::units::degree_t angle) const {
  return SwervePose{
      position.rotate(angle), bearing + angle, velocity.rotate(angle)};
}

SwervePose SwervePose::translate(Vector2D translation) const {
  return SwervePose{position + translation, bearing, velocity};
}

SwervePose SwervePose::extrapolate(pdcsu::units::second_t time) const {
  auto delta_x = pdcsu::units::inch_t{(velocity[0] * time).value() * 12.0};
  auto delta_y = pdcsu::units::inch_t{(velocity[1] * time).value() * 12.0};
  return SwervePose{position + Vector2D{delta_x, delta_y}, bearing, velocity};
}

SwervePose SwervePose::operator+(const SwervePose& other) const {
  return SwervePose{position + other.position, bearing + other.bearing,
      velocity + other.velocity};
}

SwervePose SwervePose::operator-(const SwervePose& other) const {
  return SwervePose{position - other.position, bearing - other.bearing,
      velocity - other.velocity};
}

}  // namespace funkit::robot::swerve::odometry
