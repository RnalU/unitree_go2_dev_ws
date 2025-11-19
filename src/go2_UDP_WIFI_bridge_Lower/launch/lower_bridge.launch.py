from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to YAML config for the lower UDP bridge'
    )

    config = LaunchConfiguration('config_file')

    bridge_node = Node(
        package='go2_UDP_WIFI_bridge_Lower',
        executable='lower_bridge_node',
        name='go2_udp_bridge_lower',
        parameters=[{'config_file': config}],
        output='screen'
    )

    return LaunchDescription([config_arg, bridge_node])
