#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

# 透過 message_filters 來同步多個感應器的資料
from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
from scipy.optimize import least_squares

class UWBPositionEstimator(Node):
    def __init__(self):
        super().__init__('position_nls')
        # 建立發布器，將計算結果發布到 /uwb_raw_position（格式為 Point）
        self.publisher_ = self.create_publisher(Point, '/uwb_raw_position', 10)

        # 使用 message_filters 同步四個感應器的數據
        self.sub0 = Subscriber(self, Float64, '/uwb_filtered_data_0')
        self.sub2 = Subscriber(self, Float64, '/uwb_filtered_data_2')
        self.sub3 = Subscriber(self, Float64, '/uwb_filtered_data_3')
        self.sub5 = Subscriber(self, Float64, '/uwb_filtered_data_5')
        # 這裡採用 ApproximateTimeSynchronizer 來允許小幅的時間差（slop=0.1秒）
        self.ts = ApproximateTimeSynchronizer([self.sub0, self.sub2, self.sub3, self.sub5],
                                              queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # 定義各感應器的全局座標 (單位可以依照實際情況調整)
        # (0,30,0)、(0,-30,0)、(-97,30,15)、(-97,-30,15)
        self.sensor_positions = np.array([
            [0.0,   30.0,  0.0],
            [0.0,  -30.0,  0.0],
            [-97.0, 30.0, 15.0],
            [-97.0, -30.0, 15.0]
        ], dtype=float)

    def callback(self, msg0, msg2, msg3, msg5):
        # 取得各感應器的距離測量值
        distances = [msg0.data, msg2.data, msg3.data, msg5.data]

        # 檢查數值是否合理 (例如不能是非正數)
        if any(d <= 0 for d in distances):
            self.get_logger().warn('收到非正的距離數值，略過此次計算')
            return

        measured_distances = np.array(distances, dtype=float)

        # 定義殘差函數：每個感應器的預測距離與測量距離之差
        def residuals(x, sensor_positions, measured_distances):
            res = []
            for pos, d in zip(sensor_positions, measured_distances):
                predicted = np.linalg.norm(x - pos)
                res.append(predicted - d)
            return res

        # 初始估計：取所有感應器座標的平均作為初始點
        initial_guess = np.mean(self.sensor_positions, axis=0)

        try:
            # 使用非線性最小二乘法求解（可調用 Levenberg-Marquardt 或其他方法）
            result = least_squares(residuals, initial_guess,
                                   args=(self.sensor_positions, measured_distances))
            estimated_position = result.x
        except Exception as e:
            self.get_logger().error('非線性優化計算失敗: %s' % str(e))
            return

        # 計算每個感應器的殘差，檢查是否有明顯異常值
        residual_values = residuals(estimated_position, self.sensor_positions, measured_distances)
        max_residual = np.max(np.abs(residual_values))
        # 設定一個殘差閥值 (此處設定為 2.0，可根據實際情況調整)
        threshold = 2.0
        if max_residual > threshold:
            idx_outlier = np.argmax(np.abs(residual_values))
            self.get_logger().warn(f'感應器 {idx_outlier} 殘差過高 ({residual_values[idx_outlier]:.2f})，嘗試捨棄該筆數據重新計算')

            # 移除疑似異常的感應器，再次運算
            sensor_positions_new = np.delete(self.sensor_positions, idx_outlier, axis=0)
            measured_distances_new = np.delete(measured_distances, idx_outlier)
            try:
                result_new = least_squares(residuals, estimated_position,
                                           args=(sensor_positions_new, measured_distances_new))
                estimated_position_new = result_new.x
                new_residuals = residuals(estimated_position_new, sensor_positions_new, measured_distances_new)
                if np.max(np.abs(new_residuals)) < max_residual:
                    self.get_logger().info('捨棄異常數據後解算結果改善')
                    estimated_position = estimated_position_new
            except Exception as e:
                self.get_logger().error('捨棄異常數據後重新計算失敗: %s' % str(e))
        
        # 構造 Point 訊息，並發布
        point_msg = Point()
        point_msg.x = float(estimated_position[0])
        point_msg.y = float(estimated_position[1])
        point_msg.z = float(estimated_position[2])
        self.publisher_.publish(point_msg)
        self.get_logger().info(f'發布定位座標: x={point_msg.x:.2f}, y={point_msg.y:.2f}, z={point_msg.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBPositionEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('節點關閉中...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
