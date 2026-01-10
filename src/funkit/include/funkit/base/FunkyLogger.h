#pragma once

#include <fmt/core.h>
#include <frc/Timer.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>

#include "funkit/base/FunkyLogSystem.h"

namespace funkit::base {

class FunkyLogger {
private:
  std::string pname_;

  float format_dp(float num, int num_places = 2) const {
    float value = (int)(num * std::pow(10, num_places) + 0.5);
    return ((float)value) / std::pow(10, num_places);
  }

  template <typename... T>
  void HandleLogMessage(
      int type, fmt::format_string<T...> fmt, T&&... args) const {
    LogMessage msg;

    msg.type = type;
    msg.sender = pname_;

    std::string temp_content = fmt::format(
        std::forward<fmt::format_string<T...>>(fmt), std::forward<T>(args)...);
    temp_content.erase(
        std::remove(temp_content.begin(), temp_content.end(), ';'),
        temp_content.end());

    msg.content = temp_content;

    msg.char_count = msg.sender.size() + msg.content.size();
    msg.timestamp = format_dp(frc::Timer::GetFPGATimestamp().to<double>(), 1);

    msg.period = FunkyLogSystem::getPeriod();
    msg.period_timestamp =
        format_dp(frc::Timer::GetMatchTime().to<double>(), 1);

    FunkyLogSystem::AddMessage(msg);
  }

public:
  FunkyLogger(std::string_view pname) : pname_{pname} {};

  template <typename... T>
  void Log(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(0, fmt, std::forward<T>(args)...);
  }

  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(1, fmt, std::forward<T>(args)...);
  }

  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(2, fmt, std::forward<T>(args)...);
  }
};

}  // namespace funkit::base
