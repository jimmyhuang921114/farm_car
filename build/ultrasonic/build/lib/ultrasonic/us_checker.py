import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

class UltrasonicStateNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_state_node')
        
        # 訂閱 us_raw_data topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/us/raw_data',
            self.listener_callback,
            10
        )
        
        # 發布 us_state topic
        self.publisher_ = self.create_publisher(Bool, '/us/state', 10)
        
        self.get_logger().info('Ultrasonic State Node has started.')
    
    def listener_callback(self, msg):
        # 僅檢查第 4 與第 5 個感測器值（index 3 和 4）
        if len(msg.data) >= 8:
            state = msg.data[3] >= 0.1 and msg.data[4] >= 0.1
        else:
            self.get_logger().warn('Received data too short.')
            state = False

        # 發布結果
        bool_msg = Bool()
        bool_msg.data = state
        self.publisher_.publish(bool_msg)
        self.get_logger().info(f'Published us_state: {state}')


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

