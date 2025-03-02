import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import numpy as np

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node_nls')

        self.state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, vx, vy]
        self.P = np.eye(4) * 500
        self.A = np.array([[1, 0, 0.1, 0],
                           [0, 1, 0, 0.1],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.Q = np.eye(4) * 0.1
        self.R = np.eye(2) * 5.0

        self.subscription = self.create_subscription(
            Point, '/uwb_raw_position_nls', self.filter_callback, 10)
        self.publisher = self.create_publisher(
            PointStamped, '/uwb_filtered_position_nls', 10)

    def filter_callback(self, msg):
        measured_position = np.array([msg.x, msg.y])

        # Prediction step
        self.state = self.A @ self.state
        self.P = self.A @ self.P @ self.A.T + self.Q

        # Update step
        z = measured_position
        y = z - (self.H @ self.state)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        # Publish filtered position
        filtered_position = PointStamped()
        filtered_position.header.stamp = self.get_clock().now().to_msg()
        filtered_position.point.x = self.state[0]
        filtered_position.point.y = self.state[1]
        filtered_position.point.z = msg.z
        self.publisher.publish(filtered_position)

        self.get_logger().info(
            f'Filtered position: x={filtered_position.point.x:.2f}, '
            f'y={filtered_position.point.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
