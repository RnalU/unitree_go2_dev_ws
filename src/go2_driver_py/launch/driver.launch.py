from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


import os

def generate_launch_description():

    go2_descrition_pkg = get_package_share_directory("go2_description")
    go2_driver_pkg = get_package_share_directory("go2_driver_py")

    # 声明一个布尔参数，用于控制是否启动 rviz
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false2",  # 默认值为 true
    )

    return LaunchDescription([
        use_rviz,
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(go2_descrition_pkg,"launch","display.launch.py")
            ),
            launch_arguments=[("use_joint_state_publisher","false")]
        ),
        Node(
            package="go2_driver_py",
            executable="driver",
            output="screen",
            parameters=[os.path.join(go2_driver_pkg,"params","driver.yaml")]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d",os.path.join(go2_driver_pkg,"rviz","display.rviz")]
        ),
        # 静态坐标变换
        # 官方URDF文件中的雷达坐标系是radar，发布的点云中使用的坐标系是utlidar_lidar,
        # 为了雷达能正常显示,可以直接修改URDF中的坐标系为utlidar_lidar，
        # 或者可以将utlidar_lidar等位姿的变换至radar
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0.1', '0', '0.3', '0', '0', '0', 'radar', 'utlidar_lidar']
        ),
        # 速度指令转换节点，可以将geometry_msgs::msg::Twist 转换成 unitree_api::msg::Request 消息
        Node(
            package="go2_twist_bridge_py",
            executable="twist_bridge"
        )
    ])
