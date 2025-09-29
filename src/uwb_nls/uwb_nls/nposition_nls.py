#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.optimize import least_squares
from rclpy.qos import qos_profile_sensor_data

class Position(Node):
    def __init__(self):
        super().__init__('NewPosition')
        # Publisher: 發布估算後的位置到 /uwb/position
        self.publisher_ = self.create_publisher(Point, '/new/uwb/position', 10)

        # Subscriber: 接收四個 UWB 距離，來源為 /uwb/raw_data
        self.sub_raw = self.create_subscription(
            Float32MultiArray,
            '/new/uwb/filter_data',
            self.raw_callback,
            qos_profile_sensor_data
        )

        # 儲存最新距離資料，對應感測器編號 0、1、2、3
        self.latest_data = {0: None, 1: None, 2: None, 3: None}

        # 感測器在 ROS 座標系中的固定位置
        self.sensor_positions = {
            0: np.array([   0.0, -26.0,  0.0]),
            1: np.array([   0.0,  26.0,  0.0]),
            2: np.array([-102.0, -26.0, 15.0]),
            3: np.array([-102.0,  26.0, 15.0]),
        }

        # 以 0.1 秒頻率跑 timer_callback 進行定位
        self.timer = self.create_timer(0.1, self.timer_callback)

    def raw_callback(self, msg: Float32MultiArray):
        """
        接收 /uwb/raw_data 發來的 Float32MultiArray，
        假設 data[0]~data[3] 分別是感測器 0,1,2,3 的距離
        """
        if len(msg.data) >= 4:
            self.latest_data[0] = msg.data[0]
            self.latest_data[1] = msg.data[1]
            self.latest_data[2] = msg.data[2]
            self.latest_data[3] = msg.data[3]

    def timer_callback(self):
        # 篩出有效且大於 0 的距離資料
        keys = [k for k, v in self.latest_data.items() if v is not None and v > 0]
        if len(keys) < 3:
            return

        positions = np.array([self.sensor_positions[k] for k in keys])
        distances = np.array([self.latest_data[k] for k in keys], dtype=float)

        def residuals(x, positions, distances):
            return [np.linalg.norm(x - p) - d for p, d in zip(positions, distances)]

        guess = np.mean(positions, axis=0)
        try:
            result = least_squares(residuals, guess, args=(positions, distances))
            est = result.x
        except Exception as e:
            self.get_logger().warning(f'定位演算法失敗：{e}')
            return

        # 在命令視窗打印計算結果
        self.get_logger().info(
            f'Estimated position → x: {est[0]:.3f}, y: {est[1]:.3f}, z: {est[2]:.3f}'
        )

        # 建立並發布 Point 訊息（直接對應 X, Y, Z）
        msg = Point()
        msg.x = float(est[0])
        msg.y = float(est[1])
        msg.z = float(est[2])
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Position()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
