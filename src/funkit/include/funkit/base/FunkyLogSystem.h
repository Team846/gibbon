#pragma once

#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>

#include "funkit/base/fserver.h"

namespace funkit::base {

struct LogMessage {
  int type;                 // 0 for log, 1 for warning, 2 for error
  std::string sender;       // loggable name
  std::string content;      // formatted message
  double timestamp;         // system timestamp of the message
  int period;               // 0 for disabled, 1 for teleop, 2 for autonomous
  double period_timestamp;  // game timestamp of the message
  int char_count;           // for both sender and content

  /*
  Returns a string representation of the LogMessage struct. Use for file writes,
  logging to DS. Does NOT include newline character.
  @return: string representation of the LogMessage struct
  */
  std::string pack() const {
    return fmt::format("{};{};{};{};{};{}", type, sender, content, timestamp,
        period, period_timestamp);
  }
};

class FunkyLogSystem {
public:
  /*
  Spawns a new logger thread
  @param rateLimit: maximum chars/second
  */
  static void Start(int rateLimit);

  /*
  Call in main loop to update the logger with GameState (e.g. teleop, auto,
  disabled). The GameState is used in outgoing log messages.
  @param newGameState: 0 for disabled, 1 for teleop, 2 for auto
  */
  static void SetGameState(int newGameState) { gameState = newGameState; }

  static int getPeriod() { return gameState; };

  /*
  Adds a message to the sending queue. Messages will not be sent out unless
  logger has been started. Blocking operation while sending thread is reading
  from message queue.
  @param msg: LogMessage struct
  */
  static void AddMessage(LogMessage msg) {
    std::lock_guard<std::mutex> lock(mtx);
    messages.push(std::move(msg));
  }

private:
  static uintmax_t MIN_SPACE;

  static void LogThread(int rateLimit, std::string logFileName);

  static int gameState;
  static std::queue<LogMessage> messages;

  static std::mutex mtx;

  static LoggingServer server;
};

}  // namespace funkit::base