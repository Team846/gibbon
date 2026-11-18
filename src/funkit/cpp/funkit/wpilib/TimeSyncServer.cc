#include "funkit/wpilib/TimeSyncServer.h"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

#include "funkit/wpilib/time.h"

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace funkit::wpilib {

namespace {

constexpr uint16_t kTimeSyncIden = 0x8461;
constexpr int kPingSize = 10;
constexpr int kPongSize = 18;

uint64_t ReadU64BE(const uint8_t* d) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++)
    v = (v << 8) | d[i];
  return v;
}

void WriteU64BE(uint8_t* d, uint64_t v) {
  for (int i = 7; i >= 0; i--) {
    d[i] = static_cast<uint8_t>(v & 0xFF);
    v >>= 8;
  }
}

std::atomic<bool> started{false};

}  // namespace

#ifdef _WIN32

void TimeSyncServer::Start(int port) { (void)port; }

#else

void TimeSyncServer::Start(int port) {
  if (started.exchange(true)) return;

  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "[TimeSyncServer] socket() failed; time sync disabled"
              << std::endl;
    return;
  }
  int reuse = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind(sockfd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) <
      0) {
    std::cerr << "[TimeSyncServer] bind to UDP " << port
              << " failed; time sync disabled" << std::endl;
    close(sockfd);
    return;
  }

  std::thread([sockfd] {
    uint8_t buf[64];
    for (;;) {
      pollfd pfd{sockfd, POLLIN, 0};
      if (poll(&pfd, 1, 1000) <= 0) continue;
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);
      ssize_t n = recvfrom(sockfd, buf, sizeof(buf), 0,
          reinterpret_cast<sockaddr*>(&sender), &sender_len);
      if (n < kPingSize) continue;
      if (((buf[0] << 8) | buf[1]) != kTimeSyncIden) continue;

      uint64_t fpga_us = static_cast<uint64_t>(CurrentFPGATime().value() * 1e6);
      uint8_t pong[kPongSize];
      pong[0] = kTimeSyncIden >> 8;
      pong[1] = kTimeSyncIden & 0xFF;
      WriteU64BE(pong + 2, fpga_us);
      WriteU64BE(pong + 10, ReadU64BE(buf + 2));
      sendto(sockfd, pong, kPongSize, 0,
          reinterpret_cast<const sockaddr*>(&sender), sender_len);
    }
  }).detach();
}

#endif

}  // namespace funkit::wpilib
