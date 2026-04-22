#pragma once

#include <fmt/core.h>
#include <frc/Preferences.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include <string_view>
#include <type_traits>
#include <unordered_set>

#include "funkit/base/FunkyLogger.h"
#include "pdcsu_units.h"

namespace funkit::base {
namespace detail {
template <typename T> struct is_pdcsu_unit : std::false_type {};
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
struct is_pdcsu_unit<
    pdcsu::units::Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>>
    : std::true_type {};
template <typename T>
inline constexpr bool is_pdcsu_unit_v = is_pdcsu_unit<T>::value;
}

/**
 * Loggable
 *
 * A class that provides logging functionality to any class that inherits from
 * it.
 */
class Loggable {
public:
  Loggable(const Loggable& parent_, std::string name)
      : name_{fmt::format("{}/{}", parent_.name(), name)},
        logger(fmt::format("{}/{}", parent_.name(), name)) {}

  Loggable(std::string name) : name_{name}, logger(name_) {}

  const std::string& name() const { return name_; }

  // Calls the Log function on its FunkyLogger
  template <typename... T>
  void Log(fmt::format_string<T...> fmt, T&&... args) const {
    logger.Log(fmt, std::forward<T>(args)...);
  }
  // Calls the Warn function on its FunkyLogger and increases the warn count
  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) {
    logger.Warn(fmt, std::forward<T>(args)...);
    warn_count_++;
  }

  // Calls the Error function on its FunkyLogger and increases the error count
  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) {
    logger.Error(fmt, std::forward<T>(args)...);
    error_count_++;
  }

  static void SetFMSConnected(bool connected) { fms_connected_ = connected; }

  static bool IsFMSConnected() { return fms_connected_; }

  // Returns if graphing should happen depending on fms connection
  static bool ShouldGraph() { return !fms_connected_; }

  // Puts a double entry on the smart dashboard.
  void Graph(std::string_view key, double value, bool persist = false) const;

  // Puts an integer entry on the smart dashboard.
  void Graph(std::string_view key, int value, bool persist = false) const;

  // Puts a boolean entry on the smart dashboard.
  void Graph(std::string_view key, bool value, bool persist = false) const;

  // Puts a string entry on the smart dashboard.
  void Graph(std::string_view key, const std::string& value,
      bool persist = false) const;

  // A templated function to Graph an object only if its of PDCSU unit type
  template <typename U>
  void Graph(std::string_view key, U value, bool persist = false) const {
    if constexpr (detail::is_pdcsu_unit_v<U>) {
      if (!persist && !ShouldGraph()) return;
      std::string modkey = fmt::format("{} ({})", key, value.dims());
      Graph(modkey, value.value(), persist);
    } else {
      static_assert(detail::is_pdcsu_unit_v<U>, "must be a PDCSU unit type");
    }
  }

  // A templated function to register a preference only if its of PDCSU unit
  // type
  template <typename U>
  void RegisterPreference(std::string_view key, U fallback) {
    if constexpr (detail::is_pdcsu_unit_v<U>) {
      std::string modkey = fmt::format("{} ({})", key, fallback.dims());
      RegisterPreference(modkey, fallback.value());
    } else {
      static_assert(detail::is_pdcsu_unit_v<U>, "must be a PDCSU unit type");
    }
  }

  // Creates a double preference.
  void RegisterPreference(std::string_view key, double fallback);

  // Creates a boolean preference.
  void RegisterPreference(std::string_view key, bool fallback);

  // Creates an integer preference.
  void RegisterPreference(std::string_view key, int fallback);

  // Creates a string preference.
  void RegisterPreference(std::string_view key, const std::string& fallback);

  // A templated function that gets the preference value only if its of PDCSU
  // unit type.
  template <typename U> U GetPreferenceValue_unit_type(std::string_view key) {
    if constexpr (detail::is_pdcsu_unit_v<U>) {
      U sample{};
      std::string modkey = fmt::format("{} ({})", key, sample.dims());
      return U{GetPreferenceValue_double(modkey)};
    } else {
      static_assert(detail::is_pdcsu_unit_v<U>, "must be a PDCSU unit type");
    }
  }

  // Returns the value of the preference for a double.
  double GetPreferenceValue_double(std::string_view key);

  // Returns the value of the preference for a boolean.
  bool GetPreferenceValue_bool(std::string_view key);

  // Returns the value of the preference for an integer.
  int GetPreferenceValue_int(std::string_view key);

  // Returns the value of the preference for a string.
  std::string GetPreferenceValue_string(std::string_view key);

  // A templated function that sets the preference value only if its of PDCSU
  // unit type.
  template <typename U> void SetPreferenceValue(std::string_view key, U value) {
    if constexpr (detail::is_pdcsu_unit_v<U>) {
      std::string modkey = fmt::format("{} ({})", key, value.dims());
      SetPreferenceValue(modkey, value.value());
    } else {
      static_assert(detail::is_pdcsu_unit_v<U>, "must be a PDCSU unit type");
    }
  }

  // Sets but does NOT initialize the value of the preference for a double.
  void SetPreferenceValue(std::string_view key, double value);

  // Sets but does NOT initialize the value of the preference for a boolean.
  void SetPreferenceValue(std::string_view key, bool value);

  // Sets but does NOT initialize the value of the preference for an integer.
  void SetPreferenceValue(std::string_view key, int value);

  // Sets but does NOT initialize the value of the preference for a string.
  void SetPreferenceValue(std::string_view key, const std::string& value);

  // Returns the total warning logs
  static unsigned int GetWarnCount();
  // Returns the total error logs
  static unsigned int GetErrorCount();

  // Joins together two strings
  static std::string Join(const std::string& p, const std::string& n);

  // Returns a list of keys that reference unused preferences
  static std::vector<std::string> ListKeysToPrune();

  // Logs and removes all unused preferences
  static void PrunePreferences(const Loggable* caller);

private:
  bool CheckPreferenceKeyExists(std::string_view key);

  const std::string name_;

  static std::unordered_set<std::string_view> used_preferences_;

  static unsigned int warn_count_;
  static unsigned int error_count_;

  static bool fms_connected_;

  funkit::base::FunkyLogger logger;
};

}  // namespace funkit::base
