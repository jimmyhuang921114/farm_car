import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
import numpy as np
from collections import deque
import sys

class RANSACFilter:
    def __init__(self, window_size=5, iterations=100, threshold=0.5):
        """
        window_size: 觀察的資料點數量
        iterations: RANSAC 隨機抽樣次數
        threshold: 判定 inlier 的誤差閥值
        """
        self.window = deque(maxlen=window_size)
        self.iterations = iterations
        self.threshold = threshold

    def update(self, value):
        self.window.append(value)
        data = list(self.window)
        best_inliers = []
        # 進行多次隨機抽樣，找出擁有最多 inliers 的候選模型
        for _ in range(self.iterations):
            candidate = np.random.choice(data)
            inliers = [x for x in data if abs(x - candidate) <= self.threshold]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        # 若有 inliers 則取平均作為估計值，否則取全部數據的平均
        if best_inliers:
            estimate = np.mean(best_inliers)
        else:
            estimate = np.mean(data)
        return estimate

class UWBFilter(Node):
    def __init__(self, id, window_size=5, ransac_iterations=100, ransac_threshold=0.5):
        super().__init__(f'uwb_filter_node_{id}')
        self.id = id
        self.filter = RANSACFilter(window_size=window_size, iterations=ransac_iterations, threshold=ransac_threshold)
        self.subscription = self.create_subscription(
            Float32, f'uwb_raw_data_{id}', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32, f'uwb_filtered_data_{id}', 10)

    def listener_callback(self, msg):
        raw_distance = msg.data
        filtered_distance = self.filter.update(raw_distance)
        self.publish_filtered_distance(filtered_distance)

    def publish_filtered_distance(self, distance):
        msg = Float32()
        msg.data = round(distance, 2)
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published filtered UWB data from ID {self.id}: Distance = {msg.data}")

def main(args=None):
    rclpy.init(args=args)
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