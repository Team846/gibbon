#include "funkit/base/freciver.h"

#include <array>
#include <chrono>
#include <cstring>

#include "funkit/wpilib/time.h"

#ifndef _WIN32
#include <poll.h>
#endif

namespace funkit::base {

namespace {

constexpr int kDetHdr = 18;
constexpr int kTagSz = 9;
constexpr int kDetHdrV2 = 12;
constexpr int kTagSzV2 = 5;
constexpr int kPingSz = 10;
constexpr int kPongSz = 18;
constexpr int kBindRetryAttempts = 20;
constexpr int kBindRetryDelayMs = 100;
constexpr double kFrameResetGapSeconds = 1.0;

inline bool BindWithRetry(int sockfd, const sockaddr_in& bind_addr) {
  for (int attempt = 0; attempt < kBindRetryAttempts; ++attempt) {
    if (bind(sockfd, reinterpret_cast<const sockaddr*>(&bind_addr),
            sizeof(bind_addr)) == 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kBindRetryDelayMs));
  }
  return false;
}

template <typename T> inline T ReadBE(const uint8_t* d) {
  std::array<uint8_t, sizeof(T)> b{};
  for (size_t i = 0; i < sizeof(T); ++i)
    b[i] = d[sizeof(T) - 1 - i];
  T v;
  std::memcpy(&v, b.data(), sizeof(T));
  return v;
}

template <typename T> inline void WriteBE(uint8_t* d, T v) {
  std::array<uint8_t, sizeof(T)> b{};
  std::memcpy(b.data(), &v, sizeof(T));
  for (size_t i = 0; i < sizeof(T); ++i)
    d[i] = b[sizeof(T) - 1 - i];
}

inline bool IsFrameNewer(uint16_t incoming, uint16_t previous) {
  const uint16_t diff = static_cast<uint16_t>(incoming - previous);
  return diff != 0 && diff < 0x8000;
}

}  // namespace

ReceiverServer::ReceiverServer() {
#ifdef _WIN32
  WSADATA wd;
  WSAStartup(MAKEWORD(2, 2), &wd);
#endif
}

ReceiverServer::~ReceiverServer() { Stop(); }

void ReceiverServer::Start(int port) {
  if (running_.load()) return;

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) throw std::runtime_error("Socket creation failed");

  int reuse = 1;
  setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR,
      reinterpret_cast<const char*>(&reuse), sizeof(reuse));
#ifndef _WIN32
#ifdef SO_REUSEPORT
  setsockopt(sockfd_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif
#endif

  int rcvbuf = 65536;
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF,
      reinterpret_cast<const char*>(&rcvbuf), sizeof(rcvbuf));

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_addr.s_addr = INADDR_ANY;
  sa.sin_port = htons(port);

  if (!BindWithRetry(sockfd_, sa)) {
    throw std::runtime_error("Socket bind failed");
  }

  running_.store(true);
  thread_ = std::thread(&ReceiverServer::ReceiverLoop, this);
}

void ReceiverServer::Stop() {
  running_.store(false);
  if (sockfd_ >= 0) {
#ifdef _WIN32
    closesocket(sockfd_);
#else
    close(sockfd_);
#endif
    sockfd_ = -1;
  }
  if (thread_.joinable()) thread_.join();
#ifdef _WIN32
  WSACleanup();
#endif
}

std::shared_ptr<const CameraFrame> ReceiverServer::GetLatestFrame(
    uint8_t camera_id) {
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = frames_.find(camera_id);
  return it != frames_.end() ? it->second : nullptr;
}

std::optional<CameraFrameDebug> ReceiverServer::GetFrameDebug(uint8_t camera_id) {
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = frames_.find(camera_id);
  if (it == frames_.end()) { return std::nullopt; }

  CameraFrameDebug debug{};
  debug.frame_num = it->second->frame_num;
  debug.receive_time = it->second->receive_time;
  auto drop_it = stale_drop_counts_.find(camera_id);
  debug.stale_drop_count =
      drop_it == stale_drop_counts_.end() ? 0U : drop_it->second;
  return debug;
}

void ReceiverServer::HandleTimeSync(
    const uint8_t* buf, int n, const sockaddr_in& sender, socklen_t len) {
  if (n < kPingSz) return;
  uint64_t echo = ReadBE<uint64_t>(buf + 2);
  uint64_t fpga_us =
      static_cast<uint64_t>(funkit::wpilib::CurrentFPGATime().value() * 1e6);

  uint8_t pong[kPongSz];
  WriteBE<uint16_t>(pong, kTimeSyncID);
  WriteBE<uint64_t>(pong + 2, fpga_us);
  WriteBE<uint64_t>(pong + 10, echo);
  sendto(sockfd_, reinterpret_cast<const char*>(pong), kPongSz, 0,
      reinterpret_cast<const sockaddr*>(&sender), len);
}

void ReceiverServer::HandleDetection(
    const uint8_t* buf, int n, double recv_time) {
  const uint16_t magic = ReadBE<uint16_t>(buf);

  uint8_t cam_id = 0;
  uint16_t frame = 0;
  double fpga_cap = 0.0;
  float latency = 0.0f;
  uint8_t num = 0;
  bool has_fpga_capture_time = false;

  std::vector<TagDetection> dets{};

  if (magic == kAprilTagSyncedID) {
    if (n < kDetHdr) return;
    cam_id = buf[2];
    frame = ReadBE<uint16_t>(buf + 3);
    fpga_cap = ReadBE<double>(buf + 5);
    latency = ReadBE<float>(buf + 13);
    num = buf[17];
    if (n < kDetHdr + num * kTagSz) return;
    has_fpga_capture_time = fpga_cap > 1e-9;

    dets.reserve(num);
    for (int t = 0; t < num; ++t) {
      const int off = kDetHdr + t * kTagSz;
      dets.emplace_back(TagDetection{buf[off], ReadBE<float>(buf + off + 1),
          ReadBE<float>(buf + off + 5)});
    }
  } else if (magic == kAprilTagSyncedV2ID) {
    if (n < kDetHdrV2) return;
    cam_id = buf[2];
    frame = ReadBE<uint16_t>(buf + 3);
    const uint32_t fpga_ms = ReadBE<uint32_t>(buf + 5);
    const uint16_t latency_tenth_ms = ReadBE<uint16_t>(buf + 9);
    num = buf[11];
    if (n < kDetHdrV2 + num * kTagSzV2) return;

    fpga_cap = static_cast<double>(fpga_ms) / 1000.0;
    latency = static_cast<float>(latency_tenth_ms) / 10000.0f;
    has_fpga_capture_time = fpga_ms != 0;

    dets.reserve(num);
    for (int t = 0; t < num; ++t) {
      const int off = kDetHdrV2 + t * kTagSzV2;
      const uint8_t tag_id = buf[off];
      const int16_t theta_tenth_deg = ReadBE<int16_t>(buf + off + 1);
      const uint16_t r_tenth_in = ReadBE<uint16_t>(buf + off + 3);
      dets.emplace_back(
          TagDetection{tag_id, static_cast<float>(theta_tenth_deg) / 10.0f,
              static_cast<float>(r_tenth_in) / 10.0f});
    }
  } else {
    return;
  }

  std::lock_guard<std::mutex> lk(mtx_);
  auto it = frames_.find(cam_id);
  if (it != frames_.end()) {
    const auto& prev = *(it->second);
    const bool newer_frame = IsFrameNewer(frame, prev.frame_num);
    const bool allow_reset = recv_time - prev.receive_time > kFrameResetGapSeconds;
    if (!newer_frame && !allow_reset) {
      stale_drop_counts_[cam_id]++;
      return;
    }
  }

  frames_[cam_id] = std::make_shared<CameraFrame>(CameraFrame{cam_id, frame,
      latency, std::move(dets), recv_time, has_fpga_capture_time, fpga_cap});
}

void ReceiverServer::ReceiverLoop() {
  uint8_t buf[kDetHdr + 255 * kTagSz];

  while (running_.load()) {
#ifdef _WIN32
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sockfd_, &fds);
    timeval tv{0, 100000};
    if (select(sockfd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;
#else
    pollfd pfd{sockfd_, POLLIN, 0};
    if (poll(&pfd, 1, 100) <= 0) continue;
#endif

    sockaddr_in sa{};
    socklen_t sl = sizeof(sa);
    int n = recvfrom(sockfd_, reinterpret_cast<char*>(buf), sizeof(buf), 0,
        reinterpret_cast<sockaddr*>(&sa), &sl);
    if (n < 2) continue;

    double t = funkit::wpilib::CurrentFPGATime().value();
    uint16_t magic = ReadBE<uint16_t>(buf);

    if (magic == kTimeSyncID)
      HandleTimeSync(buf, n, sa, sl);
    else if (magic == kLatencyProbeID)
      sendto(sockfd_, reinterpret_cast<const char*>(buf), n, 0,
          reinterpret_cast<const sockaddr*>(&sa), sl);
    else if ((magic == kAprilTagSyncedID && n >= kDetHdr) ||
             (magic == kAprilTagSyncedV2ID && n >= kDetHdrV2))
      HandleDetection(buf, n, t);
  }
}

}  // namespace funkit::base
