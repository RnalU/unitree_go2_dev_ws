#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace go2_udp_wifi_bridge {

struct Datagram {
  std::string topic;
  std::string type;
  std::string message_id;
  uint32_t chunk_index{0};
  uint32_t total_chunks{1};
  std::vector<uint8_t> payload;
};

class Packetizer {
public:
  explicit Packetizer(size_t max_datagram_size = 60000);

  std::vector<std::vector<uint8_t>> encode(const std::string& topic,
                                           const std::string& msg_type,
                                           const uint8_t* data,
                                           size_t size) const;

  Datagram decode(const uint8_t* data, size_t size) const;

private:
  size_t max_datagram_size_;
  size_t max_payload_size_;
};

class ReassemblyBuffer {
public:
  explicit ReassemblyBuffer(double timeout_sec = 2.0);

  // Returns true when full message assembled; result placed into output.
  bool add_chunk(const Datagram& datagram, std::vector<uint8_t>& output);

private:
  struct PartialMessage {
    double last_update{0.0};
    uint32_t total_chunks{1};
    std::vector<std::vector<uint8_t>> chunks;
  };

  void purge_expired(double now);

  double timeout_;  // seconds
  std::mutex mutex_;
  std::unordered_map<std::string, PartialMessage> messages_;
};

}  // namespace go2_udp_wifi_bridge
