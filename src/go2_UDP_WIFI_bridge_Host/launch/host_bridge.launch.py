from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/home/ymc/ros2_nav_unitree_ws/src/go2_UDP_WIFI_bridge_Host/config/host_bridge.example.yaml',
        description='Path to YAML config for the host UDP bridge'
    )

    config = LaunchConfiguration('config_file')

    bridge_node = Node(
        package='go2_UDP_WIFI_bridge_Host',
        executable='host_bridge_node',
        name='go2_udp_bridge_host',
        parameters=[{'config_file': config}],
        output='screen'
    )

    return LaunchDescription([config_arg, bridge_node])
