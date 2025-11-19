# 1.导包
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
import json
from .sport_model import ROBOT_SPORT_API_IDS

class Twist2Request(Node):
    def __init__(self):
        super().__init__('twist2request_node_py')
        self.get_logger().info("Convert geometry_msgs.msg.Twist to unitree_api.msg.Request.")

        # 3-1. 创建四组机器人速度指令发布对象
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # 3-2. 创建ROS2速度指令订阅对象
        self.twist_sub = self.create_subscription(
            Twist, 'cmd_vel', self.twist_to_request, 10
        )

    # 3-3. 在订阅对象的回调函数中将Twist转换成Request并发布
    def twist_to_request(self, twist):
        request = Request()
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]

        # 转换 (只需要x、y的线速度和z的角速度)
        x = twist.linear.x
        y = twist.linear.y
        th = twist.angular.z

        if x != 0 or y != 0 or th != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]

            # 设置线速度与角速度
            js = {"x": x, "y": y, "z": th}
            request.parameter = json.dumps(js)
            self.get_logger().info(f"Current speed: {request.parameter}")

        request.header.identity.api_id = api_id
        self.req_pub.publish(request)

def main(args=None):
    # 2. 初始化ROS2客户端
    rclpy.init(args=args)

    # 4. 调用spin函数，并传入节点对象
    node = Twist2Request()
    rclpy.spin(node)

    # 5. 资源释放
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
