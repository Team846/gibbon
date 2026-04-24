#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace funkit::base {

static constexpr uint16_t kTimeSyncID = 0x8461;
static constexpr uint16_t kAprilTagSyncedID = 0x8462;
static constexpr uint16_t kLatencyProbeID = 0x8463;
static constexpr uint16_t kAprilTagSyncedV2ID = 0x8464;

struct TagDetection {
  uint8_t tag_id;
  float theta;
  float r;
};

struct CameraFrame {
  uint8_t camera_id;
  uint16_t frame_num;
  float latency;
  std::vector<TagDetection> detections;
  double receive_time;
  bool has_fpga_capture_time = false;
  double fpga_capture_time = 0.0;
};

/*
UDP Reciver for apriltags
*/
class ReceiverServer {
public:
  ReceiverServer();
  ~ReceiverServer();

  void Start(int port);
  void Stop();
  std::shared_ptr<const CameraFrame> GetLatestFrame(uint8_t camera_id);

private:
  void ReceiverLoop();
  void HandleTimeSync(
      const uint8_t* buf, int n, const sockaddr_in& sender, socklen_t len);
  void HandleDetection(const uint8_t* buf, int n, double recv_time);

  std::map<uint8_t, std::shared_ptr<const CameraFrame>> frames_;
  std::mutex mtx_;
  int sockfd_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace funkit::base
