#pragma once

#include <frc/Timer.h>
#include <units/time.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "funkit/base/Loggable.h"
#include "pdcsu_units.h"

namespace funkit::control::base {

class CurrentLogger : public funkit::base::Loggable {
public:
  CurrentLogger(std::string name = "CurrentLogger");

  /**
   * Starts recording motor current data.
   *
   * @param filename The filename to save the current data to (without
   * extension).
   */
  void StartRecording(const std::string& filename);

  /**
   * Stops recording motor current and saves it to a file.
   *
   * @return True if the current log was successfully saved, false otherwise.
   */
  bool StopRecording();

  /**
   * Adds a current sample to the recording.
   *
   * @param current The measured motor current.
   */
  void RecordCurrent(const pdcsu::units::amp_t& current);

  /**
   * Checks if the logger is currently recording.
   *
   * @return True if recording, false otherwise.
   */
  bool IsRecording() const;

private:
  struct CurrentRecord {
    units::second_t timestamp;
    pdcsu::units::amp_t current;
  };

  std::vector<CurrentRecord> recorded_currents_;
  bool is_recording_ = false;
  std::string current_filename_;
  units::second_t start_time_;

  /**
   * Saves the recorded current readings to a CSV file.
   *
   * @return True if the current readings were successfully saved, false
   * otherwise.
   */
  bool SaveRecording();

  /**
   * Gets the path where current log files should be saved on the RoboRIO.
   *
   * @return The directory to save files to.
   */
  std::string GetSavePath() const;
};

}  // namespace funkit::control::base