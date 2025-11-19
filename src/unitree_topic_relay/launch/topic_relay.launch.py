#!/usr/bin/env python3
"""
Unitree Topic Relay Launch File
启动话题转发节点

使用方法：
1. 在扩展板上运行：
   ros2 launch unitree_topic_relay topic_relay.launch.py

2. 使用自定义配置文件：
   ros2 launch unitree_topic_relay topic_relay.launch.py params_file:=/path/to/your/params.yaml

ROS2 Launch文件说明：
- ROS1使用XML格式的.launch文件
- ROS2使用Python格式的.launch.py文件
- 优势：更灵活，可以使用Python的条件判断、循环等
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成启动描述
    
    ROS2 Launch关键概念：
    1. LaunchDescription: 包含所有启动动作的容器
    2. DeclareLaunchArgument: 声明可配置的启动参数
    3. LaunchConfiguration: 获取启动参数的值
    4. Node: 启动一个ROS2节点
    """
    
    # 声明启动参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('unitree_topic_relay'),
            'config',
            'topic_relay_params.yaml'
        ]),
        description='参数配置文件的完整路径'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error)'
    )
    
    # 创建话题转发节点
    topic_relay_node = Node(
        package='unitree_topic_relay',
        executable='topic_relay_node',
        name='unitree_topic_relay_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,  # 启用终端仿真，使输出更美观
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            '  Unitree Topic Relay Node 正在启动...\n',
            '  该节点将转发以下话题：\n',
            '    - 激光雷达数据 (LaserScan)\n',
            '    - 里程计数据 (Odometry)\n',
            '    - IMU数据 (Imu)\n',
            '    - TF变换 (TFMessage)\n',
            '    - 速度指令 (Twist)\n',
            '=' * 60, '\n'
        ]
    )
    
    return LaunchDescription([
        # 声明参数
        params_file_arg,
        log_level_arg,
        
        # 启动信息
        start_info,
        
        # 启动节点
        topic_relay_node,
    ])
