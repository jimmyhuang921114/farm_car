#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32  # 改用 Float32
import numpy as np
from scipy.optimize import least_squares
from rclpy.qos import qos_profile_sensor_data  # 使用 sensor_data QoS

class UWBPositionEstimator(Node):
    def __init__(self):
        super().__init__('uwb_position_estimator')
        # 建立發布器，發布至 /uwb_raw_position（格式為 Point）
        self.publisher_ = self.create_publisher(Point, '/uwb_raw_position', 10)

        # 訂閱各個感應器的過濾後數據 (Float32)
        self.sub_0 = self.create_subscription(Float32, '/uwb_filtered_data_0', self.callback_0, qos_profile_sensor_data)
        self.sub_2 = self.create_subscription(Float32, '/uwb_filtered_data_2', self.callback_2, qos_profile_sensor_data)
        self.sub_3 = self.create_subscription(Float32, '/uwb_filtered_data_3', self.callback_3, qos_profile_sensor_data)
        self.sub_5 = self.create_subscription(Float32, '/uwb_filtered_data_5', self.callback_5, qos_profile_sensor_data)

        # 用來保存各感應器最新數據（若沒有數據則為 None）
        self.latest_data = {0: None, 2: None, 3: None, 5: None}

        # 定義各感應器的固定全局座標
        self.sensor_positions = {
            0: np.array([0.0,   30.0,  0.0]),
            2: np.array([0.0,  -30.0,  0.0]),
            3: np.array([-97.0, 30.0, 15.0]),
            5: np.array([-97.0, -30.0, 15.0])
        }

        # 設定 Timer，每 0.1 秒觸發一次定位計算
        self.timer = self.create_timer(0.1, self.timer_callback)

    # 各感應器 callback：更新最新數據並記錄日誌
    def callback_0(self, msg):
        self.latest_data[0] = msg.data
        self.get_logger().info(f"Sensor 0 received data: {msg.data}")

    def callback_2(self, msg):
        self.latest_data[2] = msg.data
        self.get_logger().info(f"Sensor 2 received data: {msg.data}")

    def callback_3(self, msg):
        self.latest_data[3] = msg.data
        self.get_logger().info(f"Sensor 3 received data: {msg.data}")

    def callback_5(self, msg):
        self.latest_data[5] = msg.data
        self.get_logger().info(f"Sensor 5 received data: {msg.data}")

    def timer_callback(self):
        # 過濾掉尚未收到或數值不合理的感應器數據
        available_keys = [k for k, v in self.latest_data.items() if v is not None and v > 0]
        if len(available_keys) < 3:
            self.get_logger().info("Not enough sensor data available (need at least 3), available: " + str(available_keys))
            return

        # 建立對應的感應器座標與測距數據
        sensor_positions_array = np.array([self.sensor_positions[k] for k in available_keys])
        measured_distances = np.array([self.latest_data[k] for k in available_keys], dtype=float)

        # 定義殘差函數（預測距離與測量距離的差值）
        def residuals(x, sensor_positions, measured_distances):
            return [np.linalg.norm(x - pos) - d for pos, d in zip(sensor_positions, measured_distances)]

        # 初始估計使用所有可用感應器座標的平均值
        initial_guess = np.mean(sensor_positions_array, axis=0)
        try:
            result = least_squares(residuals, initial_guess, args=(sensor_positions_array, measured_distances))
            estimated_position = result.x
        except Exception as e:
            self.get_logger().error(f"Nonlinear optimization failed: {e}")
            return

        # 檢查各感應器的殘差，若最大殘差超過預設閥值則記錄警告
        residual_values = residuals(estimated_position, sensor_positions_array, measured_distances)
        max_residual = np.max(np.abs(residual_values))
        threshold = 2.0  # 可根據實際環境調整
        if max_residual > threshold:
            self.get_logger().warn(f"High maximum residual: {max_residual:.2f}, potential outlier data.")

        # 建立 Point 訊息並發布
        point_msg = Point()
        point_msg.x = float(estimated_position[0])
        point_msg.y = float(estimated_position[1])
        point_msg.z = float(estimated_position[2])
        self.publisher_.publish(point_msg)
        self.get_logger().info(f"Published position: x={point_msg.x:.2f}, y={point_msg.y:.2f}, z={point_msg.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = UWBPositionEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
