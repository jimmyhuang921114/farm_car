# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# import numpy as np

# class PositionFilter(Node):
#     def __init__(self):
#         super().__init__('PositionFilter')

#         self.state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, vx, vy]
#         self.P = np.eye(4) * 500
#         self.A = np.array([[1, 0, 0.1, 0],
#                            [0, 1, 0, 0.1],
#                            [0, 0, 1, 0],
#                            [0, 0, 0, 1]])
#         self.H = np.array([[1, 0, 0, 0],
#                            [0, 1, 0, 0]])
#         self.Q = np.eye(4) * 0.1
#         self.R = np.eye(2) * 5.0

#         self.subscription = self.create_subscription(
#             Point, '/uwb/position', self.filter_callback, 10)
#         self.publisher = self.create_publisher(
#             Point, '/target', 10)

#     def filter_callback(self, msg):
#         measured_position = np.array([msg.x, msg.y])

#         self.state = self.A @ self.state
#         self.P = self.A @ self.P @ self.A.T + self.Q

#         z = measured_position
#         y = z - (self.H @ self.state)
#         S = self.H @ self.P @ self.H.T + self.R
#         K = self.P @ self.H.T @ np.linalg.inv(S)
#         self.state += K @ y
#         self.P = (np.eye(4) - K @ self.H) @ self.P

#         filtered_position = Point()
#         # filtered_position.header.stamp = self.get_clock().now().to_msg()
#         filtered_position.point.x = self.state[0]
#         filtered_position.point.y = self.state[1]
#         filtered_position.point.z = msg.z
#         self.publisher.publish(filtered_position)
#         print(f"x: {self.state[0]:.2f}, y: {self.state[1]:.2f}")



# def main(args=None):
#     rclpy.init(args=args)
#     node = PositionFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np

class PositionFilter(Node):
    def __init__(self):
        super().__init__('PositionFilter')

        # 狀態：[x, y, vx, vy]
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(4) * 500  # 初始協方差
        self.A = np.array([
            [1, 0, 0.1, 0],
            [0, 1, 0, 0.1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])  # 狀態轉移矩陣
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])  # 觀測矩陣
        self.Q = np.eye(4) * 0.1  # 系統雜訊
        self.R = np.eye(2) * 5.0  # 觀測雜訊

        self.subscription = self.create_subscription(
            Point,
            '/uwb/position',
            self.filter_callback,
            10
        )
        self.publisher = self.create_publisher(
            Point,
            '/target',
            10
        )

    def filter_callback(self, msg):
        # 取得量測值
        measured_position = np.array([msg.x, msg.y])

        # 預測步驟
        self.state = self.A @ self.state
        self.P = self.A @ self.P @ self.A.T + self.Q

        # 更新步驟
        z = measured_position
        y = z - (self.H @ self.state)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        # 發佈平滑後的位置
        filtered_position = Point()
        filtered_position.x = self.state[0]/100
        filtered_position.y = self.state[1]/100
        filtered_position.z = msg.z  # z 不變，直接沿用原始數據

        self.publisher.publish(filtered_position)

        # 印出平滑後的座標
        print(f"x: {self.state[0]:.2f}, y: {self.state[1]:.2f}")

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
