#pragma once

#include <chrono>

namespace funkit::math {

/**
 * RampRateLimiter
 *
 * A class that limits the rate of change of a value.
 */
class RampRateLimiter {
public:
  RampRateLimiter();

  /**
   * limit()
   *
   * Limits the maximum change in a value. If a value exceeds the rateLimit,
   * apply only the maximum rate
   */
  double limit(double value, double rateLimit);

private:
  double m_lastValue = 0.0;
  std::chrono::milliseconds m_lastTime;
};

}