import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.optimize import least_squares
import math

class Position(Node):
    def __init__(self):
        super().__init__('Position')

        # 建立 publisher：發佈估算後的位置
        self.publisher_ = self.create_publisher(Point, '/uwb/position', 10)

        # ✅ 改為訂閱一個 Float32MultiArray，接收全部 UWB 距離
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb/raw_data',
            self.uwb_list_callback,
            10
        )

        # 儲存目前最新的距離資料（索引 0~3）
        self.latest_distances = [None] * 4

        # 固定每個 anchor（UWB）的位置座標（單位：cm）
        self.sensor_positions = {
            0: np.array([0.0,  30.0,  0.0]),
            1: np.array([0.0, -30.0,  0.0]),
            2: np.array([-97.0,  30.0, 15.0]),
            3: np.array([-97.0, -30.0, 15.0])
        }

        # Timer 觸發估算（例如 10Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)

    def uwb_list_callback(self, msg: Float32MultiArray):
        # ✅ 儲存收到的距離清單（包含 NaN）
        self.latest_distances = list(msg.data)

    def timer_callback(self):
        # ✅ 檢查資料中有效（非 NaN）的 anchor
        available_keys = []
        for i, d in enumerate(self.latest_distances):
            if d is not None and not math.isnan(d) and d > 0:
                available_keys.append(i)

        if len(available_keys) < 3:
            # 少於 3 個 anchor 資料無法估算位置
            self.get_logger().info("有效 anchor 不足，略過這一輪")
            return

        # 建立 sensor 座標與對應距離 array
        sensor_positions_array = np.array([self.sensor_positions[k] for k in available_keys])
        measured_distances = np.array([self.latest_distances[k] for k in available_keys], dtype=float)

        # 定義殘差函數：目標是最小化位置與距離差
        def residuals(x, sensor_positions, measured_distances):
            return [np.linalg.norm(x - pos) - d for pos, d in zip(sensor_positions, measured_distances)]

        # 以 sensor 平均位置作為初始估計點
        initial_guess = np.mean(sensor_positions_array, axis=0)

        try:
            result = least_squares(residuals, initial_guess, args=(sensor_positions_array, measured_distances))
            estimated_position = result.x
        except Exception as e:
            self.get_logger().warn(f"最小平方法估算失敗: {e}")
            return

        # 建立 ROS 的 Point 訊息
        point_msg = Point()
        point_msg.x = float(estimated_position[1])        # 將 Y 變為 ROS 的 X
        point_msg.y = float(-estimated_position[0])       # 將 X 變為 ROS 的 Y（方向反轉）
        point_msg.z = float(estimated_position[2])        # Z 軸維持不變

        self.publisher_.publish(point_msg)
        self.get_logger().info(f"📍 發佈位置: {point_msg}")

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
