#include "funkit/control/base/current_logger.h"

#include <frc/Timer.h>
#include <units/time.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "funkit/base/Loggable.h"

namespace funkit::control::base {

CurrentLogger::CurrentLogger(std::string name) : Loggable(name) {
  RegisterPreference("sampling_rate_ms", 10);
}

void CurrentLogger::StartRecording(const std::string& filename) {
  if (is_recording_) {
    Warn("Already recording current. Stop current recording first.");
    return;
  }

  current_filename_ = filename;
  recorded_currents_.clear();
  recorded_currents_.reserve(10000);
  is_recording_ = true;
  start_time_ = frc::Timer::GetFPGATimestamp();
  // Log("Started recording current data to {}.csv", current_filename_);
  Graph("recording", true);
}

bool CurrentLogger::StopRecording() {
  if (!is_recording_) {
    Warn("Not currently recording current data.");
    return false;
  }

  is_recording_ = false;
  Graph("recording", false);

  bool success = SaveRecording();

  if (success) {
    Log("Successfully saved current data to {}.csv with {} points",
        current_filename_, recorded_currents_.size());
  } else {
    Error("Failed to save current data to {}.csv", current_filename_);
  }

  return success;
}

void CurrentLogger::RecordCurrent(const pdcsu::units::amp_t& current) {
  if (!is_recording_) { return; }

  CurrentRecord record;
  record.timestamp = frc::Timer::GetFPGATimestamp();
  record.current = current;

  recorded_currents_.push_back(record);
}

bool CurrentLogger::IsRecording() const { return is_recording_; }

bool CurrentLogger::SaveRecording() {
  if (recorded_currents_.empty()) {
    Warn("No current data to save.");
    return false;
  }

  try {
    std::filesystem::create_directories(GetSavePath());

    std::string filePath = GetSavePath() + "/" + current_filename_ + ".csv";

    std::ofstream file(filePath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
      Error("Failed to open file for writing: {}", filePath);
      return false;
    }

    file << "timestamp,current" << std::endl;

    for (const auto& record : recorded_currents_) {
      file << record.timestamp.to<double>() << "," << record.current.value()
           << std::endl;
    }

    file.close();
    return true;
  } catch (const std::exception& e) {
    Error("Exception while saving current data: {}", e.what());
    return false;
  }
}

std::string CurrentLogger::GetSavePath() const {
  return "/home/lvuser/current_logs";
}
// cp lvuser@10.8.46.2:/home/lvuser/current_logs/*.csv ./
// ssh lvuser@10.8.46.2 "ls -la /home/lvuser/current_logs"
}  // namespace funkit::control::base
