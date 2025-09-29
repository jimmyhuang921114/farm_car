import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import numpy as np

class PositionFilter(Node):
    def __init__(self):
        super().__init__('NewPositionFilter')

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
            Point, '/new/uwb/position', self.filter_callback, 10)
        self.publisher = self.create_publisher(
            PointStamped, '/new/uwb/filter_position', 10)

    def filter_callback(self, msg):
        measured_position = np.array([msg.x, msg.y])

        self.state = self.A @ self.state
        self.P = self.A @ self.P @ self.A.T + self.Q

        z = measured_position
        y = z - (self.H @ self.state)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        filtered_position = PointStamped()
        filtered_position.header.stamp = self.get_clock().now().to_msg()
        filtered_position.point.x = self.state[0]
        filtered_position.point.y = self.state[1]
        filtered_position.point.z = msg.z
        self.publisher.publish(filtered_position)


def main(args=None):
    rclpy.init(args=args)
    node = PositionFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
