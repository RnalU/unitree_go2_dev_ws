import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Driver(Node):
    def __init__(self):
        super().__init__('driver_py')

        # 初始化参数
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')

        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 发布里程计消息
        self.sport_mode_state_suber_ = self.create_subscription(
            SportModeState, 'lf/sportmodestate', self.state_callback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        # 发布关节消息
        self.joint_names_ = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]
        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 10)
        self.low_state_suber_ = self.create_subscription(
            LowState, 'lf/lowstate', self.low_state_callback, 10)

    def state_callback(self, data):
        # 创建里程计消息
        odom_msg = Odometry()

        # 设置时间戳
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 设置位置
        odom_msg.pose.pose.position.x = float(data.position[0])
        odom_msg.pose.pose.position.y = float(data.position[1])
        odom_msg.pose.pose.position.z = float(data.position[2])

        # 设置姿态
        odom_msg.pose.pose.orientation.w = float(data.imu_state.quaternion[0])
        odom_msg.pose.pose.orientation.x = float(data.imu_state.quaternion[1])
        odom_msg.pose.pose.orientation.y = float(data.imu_state.quaternion[2])
        odom_msg.pose.pose.orientation.z = float(data.imu_state.quaternion[3])

        # 设置线速度
        odom_msg.twist.twist.linear.x = float(data.velocity[0])
        odom_msg.twist.twist.linear.y = float(data.velocity[1])
        odom_msg.twist.twist.linear.z = float(data.velocity[2])

        # 设置角速度
        odom_msg.twist.twist.angular.z = float(data.yaw_speed)

        # 发布里程计消息
        self.odom_pub_.publish(odom_msg)

        # 根据参数选择是否发布坐标变换
        if self.publish_odom_tf:
            transformStamped = TransformStamped()

            # 设置时间戳
            transformStamped.header.stamp = self.get_clock().now().to_msg()
            transformStamped.header.frame_id = self.odom_frame
            transformStamped.child_frame_id = self.base_frame

            # 设置平移
            transformStamped.transform.translation.x = float(data.position[0])
            transformStamped.transform.translation.y = float(data.position[1])
            transformStamped.transform.translation.z = float(data.position[2])

            # 设置旋转
            transformStamped.transform.rotation = odom_msg.pose.pose.orientation

            # 发布坐标变换
            self.tf_broadcaster_.sendTransform(transformStamped)

    def low_state_callback(self, data):
        # 填充关节状态消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names_
        ms = data.motor_state
        for i in range(12):
            joint_state_msg.position.append(float(ms[i].q))  # 确保转换为float
        self.joint_state_pub_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
