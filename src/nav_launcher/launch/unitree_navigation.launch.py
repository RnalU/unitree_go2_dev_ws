#!/usr/bin/env python3
# Copyright (c) 2025
# ROS2 Navigation Launch File for Unitree Robot

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    # 获取包路径
    nav_launcher_dir = get_package_share_directory('nav_launcher')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 配置文件路径
    default_params_file = os.path.join(nav_launcher_dir, 'params', 'unitree_nav2_params_DWB.yaml')
    default_map_file = os.path.join(nav_launcher_dir, 'maps', 'office_map.yaml')
    default_rviz_config = os.path.join(nav_launcher_dir, 'rviz', 'nav2_default.rviz')
    
    # 调试：打印路径
    print(f"[DEBUG] Params file: {default_params_file}")
    print(f"[DEBUG] Map file: {default_map_file}")
    print(f"[DEBUG] RVIZ config: {default_rviz_config}")
    
    # 声明启动参数
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_localization = LaunchConfiguration('use_localization')
    slam = LaunchConfiguration('slam')
    
    # 声明启动参数
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='命名空间（用于多机器人）')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='是否使用命名空间')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间（Gazebo）')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 参数配置文件的完整路径')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='是否自动启动导航栈')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='是否使用组合节点（提高性能）')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='地图文件的完整路径')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RVIZ2')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RVIZ配置文件路径')
    
    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization',
        default_value='True',
        description='是否启用定位（AMCL）')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='是否启用SLAM建图')
    
    # 初始位姿参数
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='机器人初始位置X坐标(米)')
    
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='机器人初始位置Y坐标(米)')
    
    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='机器人初始朝向(弧度)')
    
    declare_auto_set_initial_pose_cmd = DeclareLaunchArgument(
        'auto_set_initial_pose',
        default_value='true',
        description='是否自动设置初始位姿')
    
    # 启动 Nav2 导航栈
    # 添加 cmd_vel 话题重映射：/cmd_vel -> /host/cmd_vel
    nav2_bringup_launch = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        
        # 重映射 cmd_vel 话题到 Unitree 机器人的速度命令话题
        SetRemap(src='/cmd_vel', dst='/host/cmd_vel'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'slam': slam,
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_localization': use_localization,
            }.items()
        ),
    ])
    
    # 启动 RVIZ2
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 自动设置初始位姿节点
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    auto_set_initial_pose = LaunchConfiguration('auto_set_initial_pose')
    
    set_initial_pose_cmd = Node(
        condition=IfCondition(auto_set_initial_pose),
        package='nav_launcher',
        executable='set_initial_pose',
        name='initial_pose_setter',
        output='screen',
        parameters=[{
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'delay_sec': 3.0  # 延迟3秒等待AMCL启动
        }]
    )
    
    # 创建并返回启动描述
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)
    ld.add_action(declare_auto_set_initial_pose_cmd)
    
    # 添加节点和包含的启动文件
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_cmd)
    ld.add_action(set_initial_pose_cmd)
    
    return ld
