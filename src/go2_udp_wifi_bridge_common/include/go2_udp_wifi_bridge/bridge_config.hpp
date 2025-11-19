#pragma once

#include <string>
#include <vector>

namespace go2_udp_wifi_bridge {

enum class StreamDirection {
  Send,
  Receive
};

struct TransportConfig {
  std::string local_ip{"0.0.0.0"};
    std::string default_remote_ip{""};
  uint16_t default_remote_port{55000};
  uint16_t default_listen_port{56000};
  size_t max_datagram_size{60000};
  double queue_poll_interval{0.01};
  size_t socket_buffer_bytes{4 * 1024 * 1024};
};

struct StreamConfig {
  std::string name;
  StreamDirection direction{StreamDirection::Send};
  std::string local_topic;
  std::string remote_topic;
  std::string msg_type;
  std::string remote_ip;
  uint16_t remote_port{0};
  uint16_t listen_port{0};
  size_t qos_depth{10};
};

struct BridgeConfig {
  TransportConfig transport;
  std::vector<StreamConfig> streams;
};

BridgeConfig load_bridge_config(const std::string &path);

const char* to_string(StreamDirection direction);

}  // namespace go2_udp_wifi_bridge
