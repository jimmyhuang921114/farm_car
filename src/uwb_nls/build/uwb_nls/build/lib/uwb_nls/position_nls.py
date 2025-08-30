import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32  # 改用 Float32
import numpy as np
from scipy.optimize import least_squares
from rclpy.qos import qos_profile_sensor_data  # 使用 sensor_data QoS

class UWBPositionNLS(Node):
    def __init__(self):
        super().__init__('uwb_position_nls')
        self.publisher_ = self.create_publisher(Point, '/uwb_raw_position_nls', 10)

        self.sub_0 = self.create_subscription(Float32, '/uwb_filtered_data_0', self.callback_0, qos_profile_sensor_data)
        self.sub_2 = self.create_subscription(Float32, '/uwb_filtered_data_2', self.callback_2, qos_profile_sensor_data)
        self.sub_3 = self.create_subscription(Float32, '/uwb_filtered_data_3', self.callback_3, qos_profile_sensor_data)
        self.sub_5 = self.create_subscription(Float32, '/uwb_filtered_data_5', self.callback_5, qos_profile_sensor_data)

        self.latest_data = {0: None, 2: None, 3: None, 5: None}

        # 調整感應器的固定座標以符合 ROS 座標系統 (X 向上, Y 向左)
        self.sensor_positions = {
            0: np.array([0.0,  30.0,  0.0]),
            2: np.array([0.0, -30.0,  0.0]),
            3: np.array([-97.0,  30.0, 15.0]),
            5: np.array([-97.0, -30.0, 15.0])
        }

        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_0(self, msg):
        self.latest_data[0] = msg.data

    def callback_2(self, msg):
        self.latest_data[2] = msg.data

    def callback_3(self, msg):
        self.latest_data[3] = msg.data

    def callback_5(self, msg):
        self.latest_data[5] = msg.data

    def timer_callback(self):
        available_keys = [k for k, v in self.latest_data.items() if v is not None and v > 0]
        if len(available_keys) < 3:
            return

        sensor_positions_array = np.array([self.sensor_positions[k] for k in available_keys])
        measured_distances = np.array([self.latest_data[k] for k in available_keys], dtype=float)

        def residuals(x, sensor_positions, measured_distances):
            return [np.linalg.norm(x - pos) - d for pos, d in zip(sensor_positions, measured_distances)]

        initial_guess = np.mean(sensor_positions_array, axis=0)
        try:
            result = least_squares(residuals, initial_guess, args=(sensor_positions_array, measured_distances))
            estimated_position = result.x
        except Exception as e:
            return

        point_msg = Point()
        point_msg.x = float(estimated_position[1])  # X 軸調整為 ROS 座標 (Y 變 X)
        point_msg.y = float(-estimated_position[0])  # Y 軸調整為 ROS 座標 (X 變 -Y)
        point_msg.z = float(estimated_position[2])
        self.publisher_.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UWBPositionNLS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
