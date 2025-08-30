# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from std_msgs.msg import Float32
# import numpy as np
# from collections import deque
# import sys

# class KalmanFilter:
#     def __init__(self, process_variance, measurement_variance, initial_estimate=0.0, initial_estimate_error=1.0):
#         self.process_variance = process_variance
#         self.measurement_variance = measurement_variance
#         self.estimate = initial_estimate
#         self.estimate_error = initial_estimate_error

#     def update(self, measurement):
#         kalman_gain = self.estimate_error / (self.estimate_error + self.measurement_variance)
#         self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
#         self.estimate_error = (1 - kalman_gain) * self.estimate_error + self.process_variance
#         return self.estimate

# class MedianFilter:
#     def __init__(self, window_size):
#         self.window = deque(maxlen=window_size)

#     def update(self, value):
#         self.window.append(value)
#         return np.median(self.window)

# class UWBFilter(Node):
#     def __init__(self, id, use_median_filter=True, window_size=5):
#         super().__init__(f'uwb_filter_node_{id}')
#         self.id = id
#         self.use_median_filter = use_median_filter
#         self.filter = MedianFilter(window_size) if use_median_filter else KalmanFilter(1.0, 4.0)
#         self.subscription = self.create_subscription(Float32, f'uwb_raw_data_{id}', self.listener_callback, 10)
#         self.publisher_ = self.create_publisher(Float32, f'uwb_filtered_data_{id}', 10)

#     def listener_callback(self, msg):
#         raw_distance = msg.data
#         filtered_distance = self.filter.update(raw_distance)
#         self.publish_filtered_distance(filtered_distance)

#     def publish_filtered_distance(self, distance):
#         msg = Float32()
#         msg.data = round(distance, 2)
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published filtered UWB data from ID {self.id}: Distance = {msg.data}")

# def main(args=None):
#     rclpy.init(args=args)

#     # 使用 rclpy 提供的參數解析方法
#     node_args = rclpy.utilities.remove_ros_args(args)

#     if len(node_args) < 2:
#         print("請提供必要的參數，例如：ros2 run uwb filter <id1> <id2> ...")
#         rclpy.shutdown()
#         sys.exit(1)

#     try:
#         ids = [int(arg) for arg in node_args[1:]]
#     except ValueError:
#         print("無效的 ID，請提供數字類型的 ID，例如：ros2 run uwb filter 0 1 2")
#         rclpy.shutdown()
#         sys.exit(1)

#     nodes = [UWBFilter(id) for id in ids]
#     executor = MultiThreadedExecutor()
#     for node in nodes:
#         executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         print("\nProgram interrupted by user.")
#     finally:
#         for node in nodes:
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main(sys.argv)


# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from std_msgs.msg import Float32, Float32MultiArray
# import numpy as np
# from collections import deque
# import sys

# class MedianFilter:
#     def __init__(self, window_size):
#         self.window = deque(maxlen=window_size)

#     def update(self, value):
#         self.window.append(value)
#         return np.median(self.window)

# class UWBAggregator(Node):
#     def __init__(self, ids, window_size=5):
#         super().__init__('uwb_aggregator')
#         self.ids = ids
#         # 为每个 ID 创建一个中值滤波器
#         self.filters = {id: MedianFilter(window_size) for id in ids}
#         # 用于存储每个 ID 的最新滤波结果
#         self.latest = {id: None for id in ids}

#         # 分别订阅 uwb_raw_data_<id>
#         for id in ids:
#             topic = f'uwb_raw_data_{id}'
#             self.create_subscription(
#                 Float32,
#                 topic,
#                 self._make_callback(id),
#                 10
#             )

#         # 发布聚合后的 Float32MultiArray
#         self.publisher_ = self.create_publisher(
#             Float32MultiArray,
#             '/uwb/raw_data',
#             10
#         )

#     def _make_callback(self, id):
#         # 为每个 ID 绑定不同的回调
#         def callback(msg: Float32):
#             raw = msg.data
#             filtered = self.filters[id].update(raw)
#             # 保留两位小数
#             self.latest[id] = round(filtered, 2)
#             self.publish_aggregated()
#         return callback

#     def publish_aggregated(self):
#         # 只把已经接收到过数据的 ID 排序后组成列表
#         data_list = [self.latest[id] for id in self.ids if self.latest[id] is not None]
#         if not data_list:
#             return  # 如果还没收到任何数据就不发布
#         arr = Float32MultiArray()
#         arr.data = data_list
#         self.publisher_.publish(arr)
#         self.get_logger().info(f'Published filtered list: {arr.data}')

# def main(args=None):
#     rclpy.init(args=args)
#     node_args = rclpy.utilities.remove_ros_args(args)

#     if len(node_args) < 2:
#         print("請提供必要的參數，例如：ros2 run uwb filter 0 2 3 5")
#         rclpy.shutdown()
#         sys.exit(1)

#     try:
#         ids = [int(arg) for arg in node_args[1:]]
#     except ValueError:
#         print("無效的 ID，請提供數字類型的 ID，例如：ros2 run uwb filter 0 1 2")
#         rclpy.shutdown()
#         sys.exit(1)

#     aggregator = UWBAggregator(ids)
#     executor = MultiThreadedExecutor()
#     executor.add_node(aggregator)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         print("\nProgram interrupted by user.")
#     finally:
#         aggregator.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main(sys.argv)


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import isnan, nan
from collections import deque
from statistics import median

# 和原本一樣的 ID 順序
IDS = [0, 2, 3, 5]

# 中值濾波的 window 大小
WINDOW_SIZE = 5

class NewDistanceFilter(Node):
    def __init__(self):
        super().__init__('NewDistanceFilter')
        self.pub = self.create_publisher(Float32MultiArray, '/new/uwb/filter_data', 10)
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/new/uwb/raw_data',
            self.callback,
            10
        )

        # 每個 ID 都分配一個定長 deque
        self.windows = [deque(maxlen=WINDOW_SIZE) for _ in IDS]
        # 儲存上一次輸出的中值，用來處理 window 全為 nan 的狀況
        self.last_median = [nan] * len(IDS)

    def callback(self, msg: Float32MultiArray):
        raw = msg.data
        if len(raw) != len(IDS):
            self.get_logger().warn(
                f'收到長度不符的 list (got {len(raw)}, expected {len(IDS)})'
            )
            return

        filtered_list = []
        for idx, x in enumerate(raw):
            # 只把合法值推進 deque
            if not isnan(x):
                self.windows[idx].append(x)

            # 從 window 裡挑出非 nan 的值來算中值
            valid_vals = [v for v in self.windows[idx] if not isnan(v)]
            if valid_vals:
                m = median(valid_vals)
            else:
                # 如果 window 完全沒有任何有效值，就回落到上一輪
                m = self.last_median[idx]

            self.last_median[idx] = m
            # 四捨五入兩位
            filtered_list.append(round(m, 2) if not isnan(m) else nan)

        # 發佈濾波後結果
        out_msg = Float32MultiArray()
        out_msg.data = filtered_list
        self.pub.publish(out_msg)
        self.get_logger().info(f'Filtered distances (median): {filtered_list}')

def main(args=None):
    rclpy.init(args=args)
    node = NewDistanceFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
