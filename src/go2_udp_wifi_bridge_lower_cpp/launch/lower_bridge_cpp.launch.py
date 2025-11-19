from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to YAML config for the lower C++ UDP bridge'
    )

    config = LaunchConfiguration('config_file')

    node = Node(
        package='go2_udp_wifi_bridge_lower_cpp',
        executable='lower_bridge_cpp_node',
        name='go2_udp_bridge_lower_cpp',
        parameters=[{'config_file': config}],
        output='screen'
    )

    return LaunchDescription([config_arg, node])
