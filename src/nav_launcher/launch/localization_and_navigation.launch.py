#!/usr/bin/env python3
# 简化版启动文件 - 仅启动定位（AMCL）和导航

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_launcher_dir = get_package_share_directory('nav_launcher')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_launcher_dir, 'maps', 'my_map.yaml'),
        description='地图文件路径')
    
    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_launcher_dir, 'params', 'unitree_nav2_params.yaml'),
        description='参数文件路径')
    
    # 启动定位
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )
    
    # 启动导航
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    
    return ld
