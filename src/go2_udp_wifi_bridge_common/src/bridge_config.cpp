#include "go2_udp_wifi_bridge/bridge_config.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace go2_udp_wifi_bridge {

namespace {
StreamDirection parse_direction(const std::string &value) {
  auto lowered = value;
  for (auto &ch : lowered) {
    ch = static_cast<char>(std::tolower(ch));
  }
  if (lowered == "send") {
    return StreamDirection::Send;
  }
  if (lowered == "receive") {
    return StreamDirection::Receive;
  }
  throw std::runtime_error("Unsupported direction value: " + value);
}
}

BridgeConfig load_bridge_config(const std::string &path) {
  YAML::Node root = YAML::LoadFile(path);
  if (!root) {
    throw std::runtime_error("Failed to load config file: " + path);
  }

  BridgeConfig config;
  if (auto transport = root["transport"]) {
    config.transport.local_ip = transport["local_ip"].as<std::string>(config.transport.local_ip);
    config.transport.default_remote_ip =
        transport["default_remote_ip"].as<std::string>(config.transport.default_remote_ip);
    config.transport.default_remote_port =
        transport["default_remote_port"].as<uint16_t>(config.transport.default_remote_port);
    config.transport.default_listen_port =
        transport["default_listen_port"].as<uint16_t>(config.transport.default_listen_port);
    config.transport.max_datagram_size =
        transport["max_datagram_size"].as<size_t>(config.transport.max_datagram_size);
    config.transport.queue_poll_interval =
        transport["queue_poll_interval"].as<double>(config.transport.queue_poll_interval);
    config.transport.socket_buffer_bytes =
        transport["socket_buffer_bytes"].as<size_t>(config.transport.socket_buffer_bytes);
  }

  auto streams = root["streams"];
  if (!streams || !streams.IsSequence() || streams.size() == 0) {
    throw std::runtime_error("Bridge config must contain at least one stream entry");
  }

  for (const auto &entry : streams) {
    StreamConfig stream;
    stream.name = entry["name"].as<std::string>("stream");
    stream.direction = parse_direction(entry["direction"].as<std::string>("send"));
    if (!entry["local_topic"] || !entry["msg_type"]) {
      throw std::runtime_error("Stream " + stream.name + " must define local_topic and msg_type");
    }
    stream.local_topic = entry["local_topic"].as<std::string>();
    stream.msg_type = entry["msg_type"].as<std::string>();
    stream.remote_topic = entry["remote_topic"].as<std::string>(stream.local_topic);
    stream.remote_ip = entry["remote_ip"].as<std::string>(config.transport.default_remote_ip);
    stream.remote_port = entry["remote_port"].as<uint16_t>(config.transport.default_remote_port);
    stream.listen_port = entry["listen_port"].as<uint16_t>(config.transport.default_listen_port);
    stream.qos_depth = entry["qos_depth"].as<size_t>(10);

    if (stream.remote_ip.empty()) {
      throw std::runtime_error("Stream " + stream.name + " missing remote_ip and no default provided");
    }

    config.streams.emplace_back(std::move(stream));
  }

  return config;
}

const char *to_string(StreamDirection direction) {
  return direction == StreamDirection::Send ? "send" : "receive";
}

}  // namespace go2_udp_wifi_bridge
