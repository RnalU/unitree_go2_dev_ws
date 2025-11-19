#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "go2_udp_wifi_bridge/bridge_config.hpp"
#include "go2_udp_wifi_bridge/udp_bridge_core.hpp"

namespace {
const char *kNodeName = "go2_udp_bridge_lower_cpp";
}

class LowerBridgeCppNode : public rclcpp::Node {
public:
  LowerBridgeCppNode()
      : rclcpp::Node(kNodeName) {
    const auto default_config = ament_index_cpp::get_package_share_directory("go2_udp_wifi_bridge_lower_cpp") +
                               std::string("/config/lower_bridge_cpp.example.yaml");
    const auto config_file = this->declare_parameter<std::string>("config_file", default_config);
    auto config = go2_udp_wifi_bridge::load_bridge_config(config_file);
    bridge_core_ = std::make_unique<go2_udp_wifi_bridge::UdpBridgeCore>(*this, std::move(config));
    bridge_core_->start();
    RCLCPP_INFO(this->get_logger(), "Lower C++ UDP bridge started with config %s", config_file.c_str());
  }

  ~LowerBridgeCppNode() override { bridge_core_.reset(); }

private:
  std::unique_ptr<go2_udp_wifi_bridge::UdpBridgeCore> bridge_core_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LowerBridgeCppNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "Bridge crashed: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
