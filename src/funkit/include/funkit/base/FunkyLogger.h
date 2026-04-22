#pragma once

#include <fmt/core.h>
#include <frc/Timer.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>

#include "funkit/base/FunkyLogSystem.h"

namespace funkit::base {

/**
 * FunkyLogger
 *
 * Class that formats and handles log messages. Meant to be used for Loggable
 * objects
 */
class FunkyLogger {
private:
  std::string pname_;

  /**
   * format_dp()
   *
   * Compresses/formats a float into the specified number of decimal places
   */
  float format_dp(float num, int num_places = 2) const {
    float value = (int)(num * std::pow(10, num_places) + 0.5);
    return ((float)value) / std::pow(10, num_places);
  }

  /**
   * HandleLogMessage()
   *
   * @param type: The type of message being handled
   * @param fmt: A formatted string
   * @param args: Some number of arguments
   *
   * Formats/scrubs a LogMessage to be usable for FunkyLogSystem
   */
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
  // Constructor for FunkyLogger that initializes its members
  FunkyLogger(std::string_view pname) : pname_{pname} {};

  // Calls HandleLogMessage for messages of type 0
  template <typename... T>
  void Log(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(0, fmt, std::forward<T>(args)...);
  }

  // Calls HandleLogMessage for messages of type 1 (Warning Logs)
  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(1, fmt, std::forward<T>(args)...);
  }

  // Calls HandleLogMessage for messages of type 2 (Error Logs)
  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(2, fmt, std::forward<T>(args)...);
  }
};

}  // namespace funkit::base
