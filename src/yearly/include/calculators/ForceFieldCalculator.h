#pragma once

#include <optional>
#include <vector>

#include "funkit/base/Loggable.h"
#include "funkit/math/fieldpoints.h"
#include "pdcsu_units.h"
#include "subsystems/robot_constants.h"
#include "subsystems/robot_container.h"

class ForceFieldCalculator {
public:
  struct ForceVector {
    double pos_x;
    double pos_y;
    double dir_x;
    double dir_y;
  };

  static void Setup();

  static pdcsu::util::math::uVec<fps_t, 2> ApplyForceFields(
      pdcsu::util::math::uVec<fps_t, 2> velocity,
      const RobotContainer* container_);

  static const std::vector<ForceVector>& GetHubForceVectors() {
    return kHubForceVectors_;
  }

private:
  static pdcsu::util::math::uVec<fps_t, 2> ApplyWallForceField(
      pdcsu::util::math::uVec<fps_t, 2> velocity,
      const RobotContainer* container_);

  static pdcsu::util::math::uVec<fps_t, 2> ApplyHubForceField(
      pdcsu::util::math::uVec<fps_t, 2> velocity,
      const RobotContainer* container_);

  static constexpr double kRedHubLeft = 156.61;
  static constexpr double kRedHubRight = 203.61;
  static constexpr double kRedHubFront = 135.35;
  static constexpr double kRedHubBack = 182.35;

  static constexpr double kFieldSizeX = 317.69;
  static constexpr double kFieldSizeY = 651.22;

  static constexpr double kBlueHubFront = kFieldSizeX - kRedHubBack;
  static constexpr double kBlueHubBack = kFieldSizeX - kRedHubFront;
  static constexpr double kBlueHubLeft = kFieldSizeY - kRedHubRight;
  static constexpr double kBlueHubRight = kFieldSizeY - kRedHubLeft;

  static std::vector<ForceVector> CreateHubForceVectors();
  static std::vector<ForceVector> kHubForceVectors_;
  static std::optional<funkit::base::Loggable> loggable_opt_;
};
