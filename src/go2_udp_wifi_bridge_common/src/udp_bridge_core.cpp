#include "go2_udp_wifi_bridge/udp_bridge_core.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <utility>

namespace go2_udp_wifi_bridge {

namespace {
constexpr size_t kMaxProcessPerTick = 256;

int create_udp_socket(const std::string &local_ip, uint16_t port, size_t buffer_bytes) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("Failed to create UDP socket");
  }

  int enable = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  if (buffer_bytes > 0) {
    const int buf = static_cast<int>(buffer_bytes);
    ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buf, sizeof(buf));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buf, sizeof(buf));
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (local_ip.empty() || local_ip == "0.0.0.0") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    if (::inet_pton(AF_INET, local_ip.c_str(), &addr.sin_addr) <= 0) {
      ::close(fd);
      throw std::runtime_error("Invalid local IP address: " + local_ip);
    }
  }

  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    throw std::runtime_error("Failed to bind UDP socket on port " + std::to_string(port));
  }
  return fd;
}

sockaddr_in make_remote_addr(const std::string &ip, uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
    throw std::runtime_error("Invalid remote IP address: " + ip);
  }
  return addr;
}

std::string make_endpoint_key(const std::string &ip, uint16_t port) {
  return ip + ":" + std::to_string(port);
}

}  // namespace

UdpBridgeCore::UdpBridgeCore(rclcpp::Node &node, BridgeConfig config)
    : node_(node),
      config_(std::move(config)),
      packetizer_(config_.transport.max_datagram_size),
      reassembly_(2.0) {
  register_default_types();
}

void UdpBridgeCore::register_default_types() {
  register_type<geometry_msgs::msg::Twist>("geometry_msgs/msg/Twist");
  register_type<nav_msgs::msg::Odometry>("nav_msgs/msg/Odometry");
  register_type<sensor_msgs::msg::JointState>("sensor_msgs/msg/JointState");
  register_type<sensor_msgs::msg::PointCloud2>("sensor_msgs/msg/PointCloud2");
  register_type<go2_interfaces::msg::LowCmd>("go2_interfaces/msg/LowCmd");
}

UdpBridgeCore::~UdpBridgeCore() { stop(); }

void UdpBridgeCore::start() {
  if (started_.exchange(true)) {
    return;
  }
  setup_streams();
  const auto period = std::chrono::duration<double>(config_.transport.queue_poll_interval);
  timer_ = node_.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UdpBridgeCore::process_queue, this));
}

void UdpBridgeCore::stop() {
  if (!started_.exchange(false)) {
    return;
  }
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  for (auto &listener_pair : listeners_) {
    auto &listener = listener_pair.second;
    if (listener && listener->running.load()) {
      listener->running.store(false);
      ::shutdown(listener->socket_fd, SHUT_RDWR);
      if (listener->thread.joinable()) {
        listener->thread.join();
      }
      ::close(listener->socket_fd);
    }
  }
  listeners_.clear();

  std::lock_guard<std::mutex> lock(send_socket_mutex_);
  for (auto &entry : send_sockets_) {
    if (entry.second.fd >= 0) {
      ::close(entry.second.fd);
    }
  }
  send_sockets_.clear();
}

void UdpBridgeCore::setup_streams() {
  for (const auto &stream : config_.streams) {
    if (stream.direction == StreamDirection::Send) {
      create_send_stream(stream);
    } else {
      create_receive_stream(stream);
    }
  }
}

void UdpBridgeCore::create_send_stream(const StreamConfig &config) {
  auto it = send_factories_.find(config.msg_type);
  if (it == send_factories_.end()) {
    RCLCPP_ERROR(node_.get_logger(), "No registered send factory for message type %s", config.msg_type.c_str());
    return;
  }
  it->second(config);
}

void UdpBridgeCore::create_receive_stream(const StreamConfig &config) {
  auto it = receive_factories_.find(config.msg_type);
  if (it == receive_factories_.end()) {
    RCLCPP_ERROR(node_.get_logger(), "No registered receive factory for message type %s", config.msg_type.c_str());
    return;
  }
  it->second(config);
}

void UdpBridgeCore::handle_outbound_serialized(const StreamConfig &config, const rcl_serialized_message_t &serialized) {
  try {
    auto datagrams = packetizer_.encode(config.remote_topic, config.msg_type, serialized.buffer, serialized.buffer_length);
    auto &socket = get_send_socket(config.remote_ip, config.remote_port);
    for (auto &payload : datagrams) {
      ssize_t sent = ::sendto(socket.fd, payload.data(), payload.size(), 0,
                              reinterpret_cast<sockaddr *>(&socket.addr), sizeof(sockaddr_in));
      if (sent < 0) {
        RCLCPP_WARN(node_.get_logger(), "UDP send failed for %s:%u - %s",
                    config.remote_ip.c_str(), config.remote_port, std::strerror(errno));
        break;
      }
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to send stream %s: %s", config.name.c_str(), ex.what());
  }
}

void UdpBridgeCore::ensure_listener(uint16_t port) {
  if (listeners_.count(port)) {
    return;
  }
  auto listener = std::make_unique<Listener>();
  listener->socket_fd = create_udp_socket(config_.transport.local_ip, port, config_.transport.socket_buffer_bytes);
  listener->running.store(true);
  auto running_flag = &listener->running;
  int socket_fd = listener->socket_fd;
  listener->thread = std::thread([this, port, socket_fd, running_flag]() { listener_loop(port, socket_fd, running_flag); });
  listeners_[port] = std::move(listener);
}

void UdpBridgeCore::listener_loop(uint16_t port, int socket_fd, std::atomic<bool> *running_flag) {
  std::vector<uint8_t> buffer(config_.transport.max_datagram_size);
  while (running_flag->load()) {
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);
    ssize_t received = ::recvfrom(socket_fd, buffer.data(), buffer.size(), 0,
                                  reinterpret_cast<sockaddr *>(&addr), &addr_len);
    if (received <= 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    try {
      Datagram datagram = packetizer_.decode(buffer.data(), static_cast<size_t>(received));
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.push_back(PendingDatagram{std::move(datagram)});
    } catch (const std::exception &ex) {
      RCLCPP_WARN(node_.get_logger(), "Dropping UDP packet on %u: %s", port, ex.what());
    }
  }
}

void UdpBridgeCore::process_queue() {
  std::deque<PendingDatagram> local;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    size_t count = 0;
    while (!queue_.empty() && count < kMaxProcessPerTick) {
      local.push_back(std::move(queue_.front()));
      queue_.pop_front();
      ++count;
    }
  }

  for (auto &pending : local) {
    std::vector<uint8_t> payload;
    if (!reassembly_.add_chunk(pending.datagram, payload)) {
      continue;
    }
    auto it = receive_streams_by_topic_.find(pending.datagram.topic);
    if (it == receive_streams_by_topic_.end()) {
      continue;
    }
    it->second.publish_fn(payload);
  }
}

UdpBridgeCore::SendSocket &UdpBridgeCore::get_send_socket(const std::string &remote_ip, uint16_t port) {
  const auto key = make_endpoint_key(remote_ip, port);
  std::lock_guard<std::mutex> lock(send_socket_mutex_);
  auto &entry = send_sockets_[key];
  if (entry.fd < 0) {
    entry.fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (entry.fd < 0) {
      throw std::runtime_error("Failed to create send socket for " + key);
    }
    if (config_.transport.socket_buffer_bytes > 0) {
      const int buf = static_cast<int>(config_.transport.socket_buffer_bytes);
      ::setsockopt(entry.fd, SOL_SOCKET, SO_SNDBUF, &buf, sizeof(buf));
    }
    entry.addr = make_remote_addr(remote_ip, port);
  }
  return entry;
}

}  // namespace go2_udp_wifi_bridge
