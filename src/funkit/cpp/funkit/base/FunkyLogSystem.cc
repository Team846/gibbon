#include "funkit/base/FunkyLogSystem.h"

#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "funkit/base/compression.h"

namespace funkit::base {

LoggingServer FunkyLogSystem::server{};

std::mutex FunkyLogSystem::mtx{};

int FunkyLogSystem::gameState = 0;
std::queue<LogMessage> FunkyLogSystem::messages{};

void FunkyLogSystem::LogThread(int rateLimit, std::string logFileName) {
  std::string logPath = fmt::format("/home/lvuser/{}", logFileName);
  for (;;) {
    auto start_time = std::chrono::system_clock::now();

    int runningCharCounter = 0;

    std::vector<LogMessage> messages_to_process;
    messages_to_process.reserve(100);

    {
      std::lock_guard<std::mutex> lock(mtx);

      while (!FunkyLogSystem::messages.empty()) {
        LogMessage msg = FunkyLogSystem::messages.front();
        runningCharCounter += msg.char_count;
        if (runningCharCounter > rateLimit) { break; }

        messages_to_process.push_back(std::move(msg));
        FunkyLogSystem::messages.pop();
      }
    }

    if (!messages_to_process.empty()) {
      std::string logBundle;
      logBundle.reserve(runningCharCounter + messages_to_process.size());

      for (const auto& msg : messages_to_process) {
        logBundle += msg.pack();
        logBundle += "\n";
      }

      std::ofstream log_out(
          logPath, std::fstream::in | std::fstream::out | std::fstream::trunc);
      log_out << logBundle << std::endl;
      log_out.close();

      server.AddMessage(Compression::compress(logBundle));
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500) -
        (std::chrono::system_clock::now() - start_time));
  }
}

void FunkyLogSystem::Start(int rateLimit) {
  server.Start(5808);

  std::time_t now = std::time(nullptr);
  std::tm* now_tm = std::localtime(&now);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d_%H-%M-%S");
  std::string time_str = ss.str();

  std::string logFileName = fmt::format("log_{}.log846", time_str);

  std::thread logger_thread{FunkyLogSystem::LogThread, rateLimit, logFileName};
  logger_thread.detach();
}

}  // namespace funkit::base