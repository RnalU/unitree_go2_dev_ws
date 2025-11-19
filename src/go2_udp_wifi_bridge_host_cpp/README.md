# go2_udp_wifi_bridge_host_cpp

Host-side C++ implementation of the Go2 UDP/Wi-Fi bridge. It pairs with `go2_udp_wifi_bridge_lower_cpp` to exchange
high-bandwidth topics and latency-sensitive commands between the laptop and the Unitree Go2 expansion board.

## Usage

```bash
colcon build --packages-select go2_udp_wifi_bridge_common go2_udp_wifi_bridge_host_cpp
source install/setup.bash
ros2 run go2_udp_wifi_bridge_host_cpp host_bridge_cpp_node --ros-args -p config_file:=/path/to/host_bridge_cpp.yaml
```

Or via launch file:

```bash
ros2 launch go2_udp_wifi_bridge_host_cpp host_bridge_cpp.launch.py config_file:=/path/to/host_bridge_cpp.yaml
```

The sample configuration mirrors the Python version; adjust IPs/ports/topics to match the lower-side config. You can run
Python host nodes alongside the C++ nodes if you want to compare throughput.
