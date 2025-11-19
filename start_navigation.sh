#!/bin/bash
# 完整的多机器人通信启动脚本

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=== Unitree 机器狗导航系统 - 多机通信配置 ==="
echo ""

# Source ROS2 环境
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "✓ ROS2 Foxy 环境已加载"
else
    echo "✗ 错误: 未找到 ROS2 Foxy"
    exit 1
fi

# Source 工作空间
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "✓ 工作空间已加载"
else
    echo "⚠️  警告: 工作空间未编译，请先运行 'colcon build'"
fi

# 设置多机通信参数
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds_config.xml

echo "✓ 多机通信参数已设置"
echo ""
echo "网络配置:"
echo "  本机IP: 172.16.6.53"
echo "  机器狗IP: 172.16.6.156"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI: $CYCLONEDDS_URI"
echo ""

# 测试网络连通性
echo "=== 测试网络连通性 ==="
if ping -c 2 -W 2 172.16.6.156 > /dev/null 2>&1; then
    echo "✓ 可以 ping 通机器狗 (172.16.6.156)"
else
    echo "✗ 无法 ping 通机器狗 (172.16.6.156)"
    echo "  请检查:"
    echo "  1. 两台机器是否在同一网络"
    echo "  2. 防火墙是否允许通信"
fi
echo ""

# 检查防火墙状态
echo "=== 检查防火墙状态 ==="
if command -v ufw &> /dev/null; then
    UFW_STATUS=$(sudo ufw status | grep -i "Status:" | awk '{print $2}')
    if [ "$UFW_STATUS" == "active" ]; then
        echo "⚠️  UFW 防火墙已启用，可能会阻止 ROS2 通信"
        echo "   建议临时关闭: sudo ufw disable"
        echo "   或添加规则: sudo ufw allow from 172.16.6.0/24"
    else
        echo "✓ UFW 防火墙未启用"
    fi
else
    echo "ℹ️  未检测到 UFW 防火墙"
fi
echo ""

echo "=== 环境配置完成 ==="
echo ""
echo "下一步操作:"
echo "1. 在机器狗上也设置 ROS_DOMAIN_ID=42"
echo "2. 运行 'ros2 topic list' 查看话题"
echo "3. 启动导航: ros2 launch nav_launcher unitree_navigation.launch.py"
echo ""
