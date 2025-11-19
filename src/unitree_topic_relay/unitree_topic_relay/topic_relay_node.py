#!/usr/bin/env python3
"""
Unitree Topic Relay Node
话题转发节点 - 将宇树内部主板的话题转发到扩展板网络

功能说明：
该节点订阅来自宇树内部主板的话题，并将其重新发布为标准ROS2 Navigation话题
这样可以让通过WiFi连接的远程笔记本电脑接收到这些话题

ROS2关键概念：
1. QoS (Quality of Service): ROS2引入了QoS配置，用于控制消息的可靠性、持久性等
   - RELIABLE: 保证消息送达
   - BEST_EFFORT: 尽力而为，不保证送达（适合高频传感器数据）
   
2. rclpy vs rospy: ROS2使用rclpy替代了ROS1的rospy
   - 初始化方式不同: rclpy.init() vs rospy.init_node()
   - 节点创建: 继承Node类 vs 直接调用rospy函数
   
3. 话题订阅/发布: 
   - ROS2使用create_subscription()和create_publisher()
   - 需要指定QoS配置
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# 导入常用的消息类型
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage


class TopicRelayNode(Node):
    """
    话题转发节点
    
    该节点创建多个订阅者和发布者对，实现话题的转发功能
    """
    
    def __init__(self):
        super().__init__('unitree_topic_relay_node')
        
        self.get_logger().info('====================================')
        self.get_logger().info('Unitree Topic Relay Node 启动中...')
        self.get_logger().info('====================================')
        
        # 声明参数 - ROS2中参数是节点本地的，不像ROS1有全局参数服务器
        self.declare_parameters()
        
        # 创建QoS配置
        self.sensor_qos = self.create_sensor_qos()
        self.reliable_qos = self.create_reliable_qos()
        
        # 初始化转发器
        self.setup_relays()
        
        self.get_logger().info('所有话题转发器已初始化完成！')
        self.get_logger().info('====================================')
    
    def declare_parameters(self):
        """
        声明ROS2参数
        
        ROS2与ROS1的重要区别：
        - ROS1: 使用全局参数服务器，任何节点都可以访问
        - ROS2: 参数是节点本地的，需要先声明才能使用
        """
        # 激光雷达话题映射
        self.declare_parameter('lidar_input_topic', '/utlidar/scan')  # 宇树原始话题
        self.declare_parameter('lidar_output_topic', '/scan')  # 标准Nav2话题
        
        # 里程计话题映射
        self.declare_parameter('odom_input_topic', '/utlidar/robot_odom')
        self.declare_parameter('odom_output_topic', '/odom')
        
        # IMU话题映射
        self.declare_parameter('imu_input_topic', '/utlidar/imu')
        self.declare_parameter('imu_output_topic', '/imu')
        
        # TF话题映射
        self.declare_parameter('tf_input_topic', '/tf')
        self.declare_parameter('tf_output_topic', '/tf_relay')
        
        self.declare_parameter('tf_static_input_topic', '/tf_static')
        self.declare_parameter('tf_static_output_topic', '/tf_static_relay')
        
        # 速度指令话题映射（反向，从笔记本到机器人）
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel_unitree')
        
        # 是否启用各个转发器
        self.declare_parameter('enable_lidar_relay', True)
        self.declare_parameter('enable_odom_relay', True)
        self.declare_parameter('enable_imu_relay', True)
        self.declare_parameter('enable_tf_relay', True)
        self.declare_parameter('enable_cmd_vel_relay', True)
    
    def create_sensor_qos(self):
        """
        创建传感器数据的QoS配置
        
        ROS2 QoS关键概念：
        - BEST_EFFORT: 适合高频传感器数据，不保证送达
        - VOLATILE: 不保存历史数据
        - KEEP_LAST: 只保留最新的N条消息
        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 尽力而为
            durability=DurabilityPolicy.VOLATILE,  # 易失性
            history=HistoryPolicy.KEEP_LAST,  # 保留最新的
            depth=10  # 队列深度
        )
        return qos
    
    def create_reliable_qos(self):
        """
        创建可靠的QoS配置
        
        用于重要的控制指令和静态变换
        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 暂态本地
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        return qos
    
    def setup_relays(self):
        """设置所有的话题转发器"""
        
        # 1. 激光雷达转发
        if self.get_parameter('enable_lidar_relay').value:
            self.setup_lidar_relay()
        
        # 2. 里程计转发
        if self.get_parameter('enable_odom_relay').value:
            self.setup_odom_relay()
        
        # 3. IMU转发
        if self.get_parameter('enable_imu_relay').value:
            self.setup_imu_relay()
        
        # 4. TF转发
        if self.get_parameter('enable_tf_relay').value:
            self.setup_tf_relay()
        
        # 5. 速度指令转发
        if self.get_parameter('enable_cmd_vel_relay').value:
            self.setup_cmd_vel_relay()
    
    def setup_lidar_relay(self):
        """设置激光雷达数据转发"""
        input_topic = self.get_parameter('lidar_input_topic').value
        output_topic = self.get_parameter('lidar_output_topic').value
        
        # 创建发布者
        self.lidar_pub = self.create_publisher(
            LaserScan,
            output_topic,
            self.sensor_qos
        )
        
        # 创建订阅者
        self.lidar_sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.lidar_callback,
            self.sensor_qos
        )
        
        self.get_logger().info(f'✓ 激光雷达转发: {input_topic} -> {output_topic}')
    
    def setup_odom_relay(self):
        """设置里程计数据转发"""
        input_topic = self.get_parameter('odom_input_topic').value
        output_topic = self.get_parameter('odom_output_topic').value
        
        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            self.sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            self.sensor_qos
        )
        
        self.get_logger().info(f'✓ 里程计转发: {input_topic} -> {output_topic}')
    
    def setup_imu_relay(self):
        """设置IMU数据转发"""
        input_topic = self.get_parameter('imu_input_topic').value
        output_topic = self.get_parameter('imu_output_topic').value
        
        self.imu_pub = self.create_publisher(
            Imu,
            output_topic,
            self.sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            self.sensor_qos
        )
        
        self.get_logger().info(f'✓ IMU转发: {input_topic} -> {output_topic}')
    
    def setup_tf_relay(self):
        """设置TF数据转发"""
        # TF
        tf_input = self.get_parameter('tf_input_topic').value
        tf_output = self.get_parameter('tf_output_topic').value
        
        self.tf_pub = self.create_publisher(
            TFMessage,
            tf_output,
            self.reliable_qos
        )
        
        self.tf_sub = self.create_subscription(
            TFMessage,
            tf_input,
            self.tf_callback,
            self.reliable_qos
        )
        
        # TF Static
        tf_static_input = self.get_parameter('tf_static_input_topic').value
        tf_static_output = self.get_parameter('tf_static_output_topic').value
        
        self.tf_static_pub = self.create_publisher(
            TFMessage,
            tf_static_output,
            self.reliable_qos
        )
        
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            tf_static_input,
            self.tf_static_callback,
            self.reliable_qos
        )
        
        self.get_logger().info(f'✓ TF转发: {tf_input} -> {tf_output}')
        self.get_logger().info(f'✓ TF Static转发: {tf_static_input} -> {tf_static_output}')
    
    def setup_cmd_vel_relay(self):
        """设置速度指令转发（反向：从笔记本到机器人）"""
        input_topic = self.get_parameter('cmd_vel_input_topic').value
        output_topic = self.get_parameter('cmd_vel_output_topic').value
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            output_topic,
            self.reliable_qos
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            self.reliable_qos
        )
        
        self.get_logger().info(f'✓ 速度指令转发: {input_topic} -> {output_topic}')
    
    # ========================================================================
    # 回调函数 - 接收到消息后立即转发
    # ========================================================================
    
    def lidar_callback(self, msg):
        """激光雷达回调"""
        self.lidar_pub.publish(msg)
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.odom_pub.publish(msg)
    
    def imu_callback(self, msg):
        """IMU回调"""
        self.imu_pub.publish(msg)
    
    def tf_callback(self, msg):
        """TF回调"""
        self.tf_pub.publish(msg)
    
    def tf_static_callback(self, msg):
        """TF Static回调"""
        self.tf_static_pub.publish(msg)
    
    def cmd_vel_callback(self, msg):
        """速度指令回调"""
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    """
    主函数
    
    ROS2与ROS1的区别：
    - ROS1: rospy.init_node(), rospy.spin()
    - ROS2: rclpy.init(), node.spin(), rclpy.shutdown()
    """
    # 初始化ROS2
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = TopicRelayNode()
        
        # 自旋（处理回调）
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
