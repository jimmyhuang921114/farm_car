import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
import numpy as np
from collections import deque
import sys

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_estimate=0.0, initial_estimate_error=1.0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate
        self.estimate_error = initial_estimate_error

    def update(self, measurement):
        kalman_gain = self.estimate_error / (self.estimate_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * self.estimate_error + self.process_variance
        return self.estimate

class MedianFilter:
    def __init__(self, window_size):
        self.window = deque(maxlen=window_size)

    def update(self, value):
        self.window.append(value)
        return np.median(self.window)

class UWBFilter(Node):
    def __init__(self, id, use_median_filter=True, window_size=5):
        super().__init__(f'uwb_filter_node_{id}')
        self.id = id
        self.use_median_filter = use_median_filter
        self.filter = MedianFilter(window_size) if use_median_filter else KalmanFilter(1.0, 4.0)
        self.subscription = self.create_subscription(Float32, f'uwb_raw_data_{id}', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32, f'uwb_filtered_data_{id}', 10)

    def listener_callback(self, msg):
        raw_distance = msg.data
        filtered_distance = self.filter.update(raw_distance)
        self.publish_filtered_distance(filtered_distance)

    def publish_filtered_distance(self, distance):
        msg = Float32()
        msg.data = round(distance, 2)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published filtered UWB data from ID {self.id}: Distance = {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    # 使用 rclpy 提供的參數解析方法
    node_args = rclpy.utilities.remove_ros_args(args)

    if len(node_args) < 2:
        print("請提供必要的參數，例如：ros2 run uwb filter <id1> <id2> ...")
        rclpy.shutdown()
        sys.exit(1)

    try:
        ids = [int(arg) for arg in node_args[1:]]
    except ValueError:
        print("無效的 ID，請提供數字類型的 ID，例如：ros2 run uwb filter 0 1 2")
        rclpy.shutdown()
        sys.exit(1)

    nodes = [UWBFilter(id) for id in ids]
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
