#!/usr/bin/env python3
"""
自动设置AMCL初始位姿的节点
在导航启动时自动发布初始位姿,避免手动设置
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # 声明参数
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('delay_sec', 2.0)  # 延迟发布(等待AMCL启动)
        
        # 获取参数
        self.x = self.get_parameter('initial_pose_x').value
        self.y = self.get_parameter('initial_pose_y').value
        self.yaw = self.get_parameter('initial_pose_yaw').value
        delay = self.get_parameter('delay_sec').value
        
        # 创建发布者
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.get_logger().info(
            f'Initial pose setter started. Will publish pose at '
            f'({self.x}, {self.y}, {self.yaw}) after {delay} seconds'
        )
        
        # 延迟后发布初始位姿
        self.timer = self.create_timer(delay, self.publish_initial_pose)
        self.published = False
        
    def publish_initial_pose(self):
        """发布初始位姿"""
        if self.published:
            return
            
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # 设置位置
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # 设置朝向(从yaw转换为四元数)
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # 设置协方差矩阵(6x6,表示位姿的不确定性)
        # 对角线元素:x, y, z, roll, pitch, yaw的方差
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,    # x方差
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,    # y方差
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # z方差(不使用)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # roll方差(不使用)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # pitch方差(不使用)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06854  # yaw方差
        ]
        
        # 发布
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published initial pose: x={self.x}, y={self.y}, yaw={self.yaw}'
        )
        
        self.published = True
        self.timer.cancel()
        
        # 再等待1秒后退出节点
        self.create_timer(1.0, self.shutdown_node)
        
    def shutdown_node(self):
        """关闭节点"""
        self.get_logger().info('Initial pose published successfully. Shutting down.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
