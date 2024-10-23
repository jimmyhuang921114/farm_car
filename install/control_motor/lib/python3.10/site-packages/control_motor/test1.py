import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('twist_subscriber')
        # 訂閱 /rc_receiver 主題上的 Twist 訊息
        self.subscription = self.create_subscription(
            Twist,
            '/rc_receiver',
            self.twist_callback,
            10)
        self.subscription  # 防止未使用的變數警告

    def twist_callback(self, msg):
        # 當接收到 Twist 訊息時呼叫這個回調函式
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 輸出接收到的線速度和角速度
        self.get_logger().info(f'Received Twist Message - Linear X: {linear_x}, Angular Z: {angular_z}')

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
