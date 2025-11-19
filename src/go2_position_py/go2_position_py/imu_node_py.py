import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState

class PositionLoggerNode(Node):
    def __init__(self):
        super().__init__('position_logger_node')
        
        # （可选注释）订阅运动状态话题（/lf/sportmodestate），如需启用可删除注释并调整回调
        self.odom_sub1 = self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self.odom_callback1,
            10
        )
        
        # 订阅位置话题（/unitree/slam_relocation/odom）
        self.odom_sub2 = self.create_subscription(
            Odometry,
            '/unitree/slam_relocation/odom',
            self.odom_callback2,
            10
        )
        
        self.get_logger().info('Position Logger Node has been started.')

    # （可选注释）运动状态话题回调，如需启用可删除注释并修正数据解析
    def odom_callback1(self, msg):
        """处理第一个里程计话题的回调函数，打印坐标"""
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]
        
        self.get_logger().info(
            f"话题1 (/lf/sportmodestate) 坐标: X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}"
        )

    def odom_callback2(self, msg):
        """处理 /unitree/slam_relocation/odom 话题，打印位置坐标"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        self.get_logger().info(
            f"话题2 (/unitree/slam_relocation/odom) 坐标: X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    position_logger = PositionLoggerNode()
    rclpy.spin(position_logger)
    position_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

