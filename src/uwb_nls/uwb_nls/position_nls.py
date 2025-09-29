import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.optimize import least_squares
from rclpy.qos import qos_profile_sensor_data

class Position(Node):
    def __init__(self):
        super().__init__('Position')
        self.publisher_ = self.create_publisher(Point, '/uwb/position', 10)

        self.sub_raw = self.create_subscription(
            Float32MultiArray,
            '/uwb/filter_data',
            self.raw_callback,
            qos_profile_sensor_data
        )

        self.latest_data = {0: None, 1: None, 2: None, 3: None}
        self.sensor_positions = {
            0: np.array([0.0, -36.0, 0.0]),
            1: np.array([0.0, 36.0, 0.0]),
            2: np.array([-119.0, -49.0, 13.0]),
            3: np.array([-119.0, 49.0, 13.0]),
        }

        self.timer = self.create_timer(0.1, self.timer_callback)

        # 強化邏輯變數
        self.last_position = None
        self.jump_counter = 0  # 追蹤連續跳變次數
        self.RESIDUAL_THRESHOLD = 50.0  # cm
        self.JUMP_THRESHOLD = 100.0     # cm
        self.MAX_JUMP_COUNT = 10        # 容許連跳次數上限

    def raw_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.latest_data[0] = msg.data[0]
            self.latest_data[1] = msg.data[1]
            self.latest_data[2] = msg.data[2]
            self.latest_data[3] = msg.data[3]

    def timer_callback(self):
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
            residual_norm = np.linalg.norm(result.fun)

            # 殘差過濾
            if residual_norm > self.RESIDUAL_THRESHOLD:
                self.get_logger().warn(
                    f"殘差過大 → {residual_norm:.2f} cm，捨棄該次定位結果。"
                )
                return

            # 跳變過濾 + 容錯機制
            if self.last_position is not None:
                jump = np.linalg.norm(est - self.last_position)
                if jump > self.JUMP_THRESHOLD:
                    self.jump_counter += 1
                    if self.jump_counter < self.MAX_JUMP_COUNT:
                        self.get_logger().warn(
                            f"位置跳變過大（{jump:.1f} cm），第 {self.jump_counter} 次跳過"
                        )
                        return
                    else:
                        self.get_logger().warn(
                            f"位置跳變連續超過 {self.MAX_JUMP_COUNT} 次，強制更新位置"
                        )
                        self.jump_counter = 0  # 重置計數器
                else:
                    self.jump_counter = 0  # 正常更新 → 重置計數器

            self.last_position = est  # 更新上一個位置

        except Exception as e:
            self.get_logger().warning(f'定位演算法失敗：{e}')
            return

        # 印出定位結果
        self.get_logger().info(
            f'Estimated position → x: {est[0]:.3f}, y: {est[1]:.3f}, z: {est[2]:.3f}'
        )

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

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# from rclpy.qos import qos_profile_sensor_data

# class Position(Node):
#     def __init__(self):
#         super().__init__('Position')
#         self.publisher_ = self.create_publisher(Point, '/uwb/position', 10)

#         self.sub_raw = self.create_subscription(
#             Float32MultiArray,
#             '/uwb/filter_data',
#             self.raw_callback,
#             qos_profile_sensor_data
#         )

#         self.latest_data = {0: None, 1: None, 2: None, 3: None}
#         self.sensor_positions = {
#             0: np.array([0.0, -26.0, 0.0]),
#             1: np.array([0.0, 26.0, 0.0]),
#             2: np.array([-102.0, -26.0, 15.0]),
#             3: np.array([-102.0, 26.0, 15.0]),
#         }

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         # 濾波與跳變偵測設定
#         self.last_position = None
#         self.jump_counter = 0
#         self.JUMP_THRESHOLD = 100.0     # cm
#         self.MAX_JUMP_COUNT = 10
#         self.z_person = 0.02            # 固定的人高度（z）

#     def raw_callback(self, msg: Float32MultiArray):
#         if len(msg.data) >= 4:
#             self.latest_data[0] = msg.data[0]
#             self.latest_data[1] = msg.data[1]
#             self.latest_data[2] = msg.data[2]
#             self.latest_data[3] = msg.data[3]

#     def timer_callback(self):
#         # 收集有效的感測器資料
#         keys = [k for k, v in self.latest_data.items() if v is not None and v > 0]
#         if len(keys) < 3:
#             return

#         positions = [self.sensor_positions[k] for k in keys]
#         distances = [self.latest_data[k] for k in keys]

#         plane_distances = []
#         xy_positions = []

#         # 將三維距離轉為平面距離
#         for dist, (x, y, z) in zip(distances, positions):
#             try:
#                 d_plane = np.sqrt(dist**2 - (z - self.z_person)**2)
#                 plane_distances.append(d_plane)
#                 xy_positions.append([x, y])
#             except ValueError:
#                 self.get_logger().warn(f"無效距離（z={z}）：{dist}")
#                 return

#         # 建立線性方程組 Ax = b
#         A = []
#         b = []
#         x0, y0 = xy_positions[0]
#         d0 = plane_distances[0]

#         for (x1, y1), d1 in zip(xy_positions[1:], plane_distances[1:]):
#             A.append([2 * (x1 - x0), 2 * (y1 - y0)])
#             b.append(d0**2 - d1**2 - x0**2 + x1**2 - y0**2 + y1**2)

#         A = np.array(A)
#         b = np.array(b)

#         if np.linalg.cond(A) > 1e12:
#             self.get_logger().warn('線性系統條件數過高，跳過該次定位')
#             return

#         try:
#             est_xy = np.linalg.lstsq(A, b, rcond=None)[0]
#         except Exception as e:
#             self.get_logger().warning(f'解方程失敗：{e}')
#             return

#         est = np.array([est_xy[0], est_xy[1], self.z_person])
#         jump = np.linalg.norm(est - self.last_position) if self.last_position is not None else 0

#         # 跳變濾波器
#         if self.last_position is not None and jump > self.JUMP_THRESHOLD:
#             self.jump_counter += 1
#             if self.jump_counter < self.MAX_JUMP_COUNT:
#                 self.get_logger().warn(f"跳變過大（{jump:.1f} cm），第 {self.jump_counter} 次跳過")
#                 return
#             else:
#                 self.get_logger().warn("連續跳變超限，強制更新")
#                 self.jump_counter = 0
#         else:
#             self.jump_counter = 0

#         self.last_position = est

#         # 印出定位結果
#         self.get_logger().info(f'2D位置 → x: {est[0]:.2f}, y: {est[1]:.2f}')

#         msg = Point()
#         msg.x = float(est[0])
#         msg.y = float(est[1])
#         msg.z = float(est[2])
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = Position()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
