#pragma once

#include <atomic>
#include <deque>
#include <map>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <go2_interfaces/msg/low_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "go2_udp_wifi_bridge/bridge_config.hpp"
#include "go2_udp_wifi_bridge/packetizer.hpp"

namespace go2_udp_wifi_bridge {

class UdpBridgeCore {
public:
  UdpBridgeCore(rclcpp::Node &node, BridgeConfig config);
  ~UdpBridgeCore();

  void start();
  void stop();

private:
  struct SendStream {
    StreamConfig config;
    rclcpp::SubscriptionBase::SharedPtr subscription;
  };

  struct ReceiveStream {
    StreamConfig config;
    std::function<void(const std::vector<uint8_t>&)> publish_fn;
  };

  struct Listener {
    int socket_fd{-1};
    std::thread thread;
    std::atomic<bool> running{false};
  };

  struct PendingDatagram {
    Datagram datagram;
  };

  struct SendSocket {
    int fd{-1};
    sockaddr_in addr{};
  };

  void setup_streams();
  void register_default_types();

  template<typename MsgT>
  void register_type(const std::string &msg_type);

  void create_send_stream(const StreamConfig &config);
  void create_receive_stream(const StreamConfig &config);

  template<typename MsgT>
  void create_send_stream_typed(const StreamConfig &config);

  template<typename MsgT>
  void create_receive_stream_typed(const StreamConfig &config);

  void handle_outbound_serialized(const StreamConfig &config, const rcl_serialized_message_t &serialized);

  void ensure_listener(uint16_t port);
  void listener_loop(uint16_t port, int socket_fd, std::atomic<bool> *running_flag);

  void process_queue();

  SendSocket &get_send_socket(const std::string &remote_ip, uint16_t port);

  rclcpp::Node &node_;
  BridgeConfig config_;
  Packetizer packetizer_;
  ReassemblyBuffer reassembly_;

  std::vector<SendStream> send_streams_;
  std::unordered_map<std::string, ReceiveStream> receive_streams_by_topic_;
  std::unordered_map<std::string, std::function<void(const StreamConfig &)>> send_factories_;
  std::unordered_map<std::string, std::function<void(const StreamConfig &)>> receive_factories_;
  std::map<uint16_t, std::unique_ptr<Listener>> listeners_;
  std::map<std::string, SendSocket> send_sockets_;
  std::mutex send_socket_mutex_;

  std::deque<PendingDatagram> queue_;
  std::mutex queue_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> started_{false};
};

template<typename MsgT>
inline void UdpBridgeCore::register_type(const std::string &msg_type) {
  send_factories_[msg_type] = [this](const StreamConfig &config) {
    this->create_send_stream_typed<MsgT>(config);
  };
  receive_factories_[msg_type] = [this](const StreamConfig &config) {
    this->create_receive_stream_typed<MsgT>(config);
  };
}

template<typename MsgT>
inline void UdpBridgeCore::create_send_stream_typed(const StreamConfig &config) {
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).keep_last(config.qos_depth);
  auto subscription = node_.create_subscription<MsgT>(
      config.local_topic, qos,
      [this, config](const typename MsgT::SharedPtr msg) {
        rclcpp::Serialization<MsgT> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(msg.get(), &serialized_msg);
        this->handle_outbound_serialized(config, serialized_msg.get_rcl_serialized_message());
      });
  send_streams_.push_back(SendStream{config, subscription});
  RCLCPP_INFO(node_.get_logger(), "Forwarding %s -> %s:%u as %s",
              config.local_topic.c_str(), config.remote_ip.c_str(), config.remote_port, config.remote_topic.c_str());
}

template<typename MsgT>
inline void UdpBridgeCore::create_receive_stream_typed(const StreamConfig &config) {
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).keep_last(config.qos_depth);
  auto publisher = node_.create_publisher<MsgT>(config.local_topic, qos);
  receive_streams_by_topic_[config.remote_topic] = ReceiveStream{
      config,
      [publisher](const std::vector<uint8_t> &payload) {
        rclcpp::SerializedMessage serialized(payload.size());
        auto &rcl_msg = serialized.get_rcl_serialized_message();
        if (!payload.empty()) {
          std::memcpy(rcl_msg.buffer, payload.data(), payload.size());
        }
        rcl_msg.buffer_length = payload.size();
        MsgT message;
        rclcpp::Serialization<MsgT> serializer;
          serializer.deserialize_message(&serialized, &message);
        publisher->publish(message);
      }};
  ensure_listener(config.listen_port);
  RCLCPP_INFO(node_.get_logger(), "Listening on %u for %s -> %s",
              config.listen_port, config.remote_topic.c_str(), config.local_topic.c_str());
}

}  // namespace go2_udp_wifi_bridge
