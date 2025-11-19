# go2_udp_wifi_bridge_lower_cpp

C++ implementation of the Go2 UDP/Wi-Fi bridge that runs on the expansion board ("lower" side). It reuses the
common `go2_udp_wifi_bridge_common` library for packetization and configuration, while using `rclcpp` for maximum
performance.

## Usage

```bash
colcon build --packages-select go2_udp_wifi_bridge_common go2_udp_wifi_bridge_lower_cpp
source install/setup.bash
ros2 run go2_udp_wifi_bridge_lower_cpp lower_bridge_cpp_node --ros-args -p config_file:=/path/to/lower_bridge_cpp.yaml
```

The provided `config/lower_bridge_cpp.example.yaml` matches the default Python config. Adjust IPs/ports/topics to suit
your deployment. To use a launch file instead:

```bash
ros2 launch go2_udp_wifi_bridge_lower_cpp lower_bridge_cpp.launch.py config_file:=/path/to/lower_bridge_cpp.yaml
```

## Key Differences vs Python Version
- Built with `rclcpp`, uses multi-threaded listeners and larger socket buffers for higher throughput.
- Default `max_datagram_size` reduced to 32 KB to mitigate IP fragmentation.
- Adds `socket_buffer_bytes` to YAML to set both send/receive socket buffers (defaults to 8 MB).

Python nodes remain available for legacy workflows; simply choose which executable to launch.
