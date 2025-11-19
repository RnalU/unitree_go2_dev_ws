# Go2 UDP Wi-Fi Bridge

This folder contains the ROS 2 package that runs on the Unitree Go2 expansion board (`go2_UDP_WIFI_bridge_Lower`).
A mirrored package lives in `../go2_UDP_WIFI_bridge_Host` for the laptop/host side. Together they provide a
configurable UDP-based data bridge for a handful of latency-sensitive topics (voxel map, odometry, joint states,
and velocity or joint-control commands).

## Features
- YAML-driven topic mapping with per-stream direction (`send` or `receive`).
- Chunking and reassembly logic so large messages (e.g., `PointCloud2` or `VoxelMapCompressed`) can traverse UDP without MTU issues.
- Independent configs for the robot and host, allowing topic renaming between sides.
- Minimal launch files and entry points so each side can be started with `ros2 launch` or `ros2 run`.

## Quick Start
1. Copy the example config on each side and adjust IP/port/topic names:
   ```bash
   cp $(ros2 pkg prefix go2_UDP_WIFI_bridge_Lower)/config/lower_bridge.example.yaml ~/lower_bridge.yaml
   cp $(ros2 pkg prefix go2_UDP_WIFI_bridge_Host)/config/host_bridge.example.yaml ~/host_bridge.yaml
   ```
2. On the Go2 expansion SBC:
   ```bash
   ros2 run go2_UDP_WIFI_bridge_Lower lower_bridge_node --ros-args -p config_file:=/path/to/lower_bridge.yaml
   ```
3. On the laptop/host:
   ```bash
   ros2 run go2_UDP_WIFI_bridge_Host host_bridge_node --ros-args -p config_file:=/path/to/host_bridge.yaml
   ```

When the configs share the same `remote_topic` strings, messages are serialized with `rclpy` and transported over UDP.
Small control topics (`Twist`, `LowCmd`) remain low-latency, while larger sensor topics are automatically chunked and
reassembled before being re-published on the receiving side.

## Extending
Add additional entries to the `streams` list for any other ROS 2 message that should be bridged. The only
requirements are:
- Provide fully-qualified ROS 2 message type names (e.g., `geometry_msgs/msg/Twist`).
- Ensure both sides include matching `remote_topic` strings so the receiver knows how to decode the inbound payload.
- Pick unique UDP ports for high-bandwidth streams to avoid starving control traffic.
