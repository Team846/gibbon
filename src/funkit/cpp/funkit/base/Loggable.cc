#include "funkit/base/Loggable.h"

#include <fmt/core.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <sstream>

namespace funkit::base {

std::string Loggable::Join(const std::string& p, const std::string& n) {
  return p + "/" + n;
}

unsigned int Loggable::GetWarnCount() { return warn_count_; }

unsigned int Loggable::GetErrorCount() { return error_count_; }

std::unordered_set<std::string> Loggable::used_preferences_{};

unsigned int Loggable::warn_count_ = 0;
unsigned int Loggable::error_count_ = 0;

bool Loggable::fms_connected_ = false;

void Loggable::Graph(std::string_view key, double value, bool persist) const {
  if (!persist && !ShouldGraph()) return;
  auto it = graph_doubles_.find(key);
  if (it == graph_doubles_.end()) {
    it = graph_doubles_
             .try_emplace(std::string{key},
                 nt::NetworkTableInstance::GetDefault()
                     .GetTable("SmartDashboard")
                     ->GetDoubleTopic(fmt::format("{}/{}", name_, key))
                     .Publish())
             .first;
  }
  it->second.Set(value);
}

void Loggable::Graph(std::string_view key, int value, bool persist) const {
  if (!persist && !ShouldGraph()) return;
  frc::SmartDashboard::PutNumber(fmt::format("{}/{}", name_, key), value);
}

void Loggable::Graph(std::string_view key, bool value, bool persist) const {
  if (!persist && !ShouldGraph()) return;
  frc::SmartDashboard::PutBoolean(fmt::format("{}/{}", name_, key), value);
}

void Loggable::Graph(
    std::string_view key, const std::string& value, bool persist) const {
  if (!persist && !ShouldGraph()) return;
  frc::SmartDashboard::PutString(fmt::format("{}/{}", name_, key), value);
}

void Loggable::RegisterPreference(std::string_view key, double fallback) {
  std::string fullkey = fmt::format("{}/{}", name_, key);
  frc::Preferences::InitDouble(fullkey, fallback);
  if (frc::Preferences::GetDouble(fullkey, 0.0) != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

void Loggable::RegisterPreference(std::string_view key, bool fallback) {
  std::string fullkey = fmt::format("{}/{}", name_, key);
  frc::Preferences::InitBoolean(fullkey, fallback);
  if (frc::Preferences::GetBoolean(fullkey, false) != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

void Loggable::RegisterPreference(std::string_view key, int fallback) {
  std::string fullkey = fmt::format("{}/{}", name_, key);
  frc::Preferences::InitInt(fullkey, fallback);
  if (frc::Preferences::GetInt(fullkey, 0) != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

void Loggable::RegisterPreference(
    std::string_view key, const std::string& fallback) {
  std::string fullkey = fmt::format("{}/{}", name_, key);
  frc::Preferences::InitString(fullkey, fallback);
  if (frc::Preferences::GetString(fullkey, "") != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

const std::string& Loggable::ResolveUnitKey(
    std::string_view key, std::string_view dims) const {
  auto it = unit_key_cache_.find(key);
  if (it == unit_key_cache_.end()) {
    it = unit_key_cache_
             .try_emplace(std::string{key}, fmt::format("{} ({})", key, dims))
             .first;
  }
  return it->second;
}

bool Loggable::CheckPreferenceKeyExists(std::string_view key) {
  std::string fullkey = fmt::format("{}/{}", name_, key);
  if (!frc::Preferences::ContainsKey(fullkey)) {
    Warn("Attempted to access uninitialized preference {}", key);
    return false;
  }
  return true;
}

double Loggable::GetPreferenceValue_double(std::string_view key) {
  auto it = pref_doubles_.find(key);
  if (it == pref_doubles_.end()) {
    if (!CheckPreferenceKeyExists(key)) { return 0; }
    it = pref_doubles_
             .try_emplace(std::string{key},
                 nt::NetworkTableInstance::GetDefault()
                     .GetTable("Preferences")
                     ->GetDoubleTopic(fmt::format("{}/{}", name_, key))
                     .GetEntry(0.0))
             .first;
  }
  return it->second.Get();
}

bool Loggable::GetPreferenceValue_bool(std::string_view key) {
  if (!CheckPreferenceKeyExists(key)) { return false; }
  return frc::Preferences::GetBoolean(fmt::format("{}/{}", name_, key));
}

int Loggable::GetPreferenceValue_int(std::string_view key) {
  if (!CheckPreferenceKeyExists(key)) { return 0; }
  return frc::Preferences::GetInt(fmt::format("{}/{}", name_, key));
}

std::string Loggable::GetPreferenceValue_string(std::string_view key) {
  if (!CheckPreferenceKeyExists(key)) { return ""; }
  return frc::Preferences::GetString(fmt::format("{}/{}", name_, key));
}

void Loggable::SetPreferenceValue(std::string_view key, double value) {
  frc::Preferences::SetDouble(fmt::format("{}/{}", name_, key), value);
}

void Loggable::SetPreferenceValue(std::string_view key, bool value) {
  frc::Preferences::SetBoolean(fmt::format("{}/{}", name_, key), value);
}

void Loggable::SetPreferenceValue(std::string_view key, int value) {
  frc::Preferences::SetInt(fmt::format("{}/{}", name_, key), value);
}

void Loggable::SetPreferenceValue(
    std::string_view key, const std::string& value) {
  frc::Preferences::SetString(fmt::format("{}/{}", name_, key), value);
}

std::vector<std::string> Loggable::ListKeysToPrune() {
  std::vector<std::string> keys_to_prune{};
  for (auto& key : frc::Preferences::GetKeys()) {
    if (used_preferences_.find(key) == used_preferences_.end()) {
      keys_to_prune.push_back(key);
    }
  }
  return keys_to_prune;
}

void Loggable::PrunePreferences(const Loggable* caller) {
  for (auto& key : ListKeysToPrune()) {
    caller->Log("Pruning unused preference {}", key);
    frc::Preferences::Remove(key);
  }
}

}  // namespace funkit::base