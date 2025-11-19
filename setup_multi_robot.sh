#!/bin/bash
# ROS2 多机器人通信配置脚本
# 使用方法: source setup_multi_robot.sh

echo "=== ROS2 多机器通信配置 ==="

# 1. 设置 ROS_DOMAIN_ID (0-101之间，两台机器必须相同)
export ROS_DOMAIN_ID=42
echo "✓ ROS_DOMAIN_ID 设置为: $ROS_DOMAIN_ID"

# 2. 设置 ROS_LOCALHOST_ONLY=0 (允许跨机器通信)
export ROS_LOCALHOST_ONLY=0
echo "✓ ROS_LOCALHOST_ONLY 设置为: $ROS_LOCALHOST_ONLY (允许网络通信)"

# 3. 配置 Cyclone DDS (推荐使用，性能更好)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✓ DDS 实现设置为: $RMW_IMPLEMENTATION"

# 4. 显示本机IP
MY_IP=$(hostname -I | awk '{print $1}')
echo "✓ 本机IP: $MY_IP"

# 5. 显示当前配置
echo ""
echo "=== 当前ROS2网络配置 ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "本机IP: $MY_IP"
echo ""
echo "⚠️  请确保机器狗(172.16.6.156)也设置了相同的 ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo ""
echo "测试连接命令:"
echo "  ros2 topic list        # 查看所有话题"
echo "  ros2 node list         # 查看所有节点"
echo "  ros2 topic echo /xxx   # 订阅某个话题"
echo "  ping 172.16.6.156      # 测试网络连通性"
