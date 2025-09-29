# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped, Point
# import matplotlib.pyplot as plt

# class VisualizationNode(Node):
#     def __init__(self):
#         super().__init__('visualization_node')

#         # 訂閱過濾後的位置
#         self.triangulation = self.create_subscription(
#             PointStamped, '/uwb_filtered_position', self.triangulation_callback, 10)

#         # 訂閱過濾前的位置
#         self.nls = self.create_subscription(
#             PointStamped, '/uwb_filtered_position_nls', self.nls_callback, 10)

#         # 感應器位置定義
#         self.sensor_positions = {
#             0: (26.0,   0.0),
#             2: (-26.0, 0.0),
#             3: (33.0, -102.0),
#             5: (-33.0, -102.0)
#         }

#         self.figure, self.ax = plt.subplots(figsize=(8, 8))
#         self.visualization_initialized = False

#         # 初始位置記錄
#         self.triangulation_position = None
#         self.nls_position = None

#     def triangulation_callback(self, msg):
#         """接收過濾前的位置"""
#         self.triangulation_position = (msg.point.x, msg.point.y)

#     def nls_callback(self, msg):
#         """接收過濾後的位置"""
#         self.nls_position = (msg.point.x, msg.point.y)
#         self.update_visualization()

#     def update_visualization(self):
#         if self.triangulation_position is None or self.nls_position is None:
#             # 如果任何一個數據未到達，等待
#             return

#         triangulation_x, triangulation_y = self.triangulation_position
#         nls_x, nls_y = self.nls_position

#         self.visualize(triangulation_x, triangulation_y, nls_x, nls_y)

#     # 視覺化方法
#     def visualize(self, triangulation_x, triangulation_y, nls_x, nls_y):
#         if not self.visualization_initialized:
#             # 設置繪圖的固定範圍和標籤
#             self.ax.set_xlim(-1000, 1000)
#             self.ax.set_ylim(-1000, 1000)
#             self.ax.set_title('UWB Positioning Visualization')
#             self.ax.set_xlabel('X (cm)')
#             self.ax.set_ylabel('Y (cm)')
#             self.ax.grid(True)
#             self.visualization_initialized = True

#         # 清除舊圖
#         self.ax.clear()

#         # 繪製感應器位置
#         for sensor_id, (sx, sy) in self.sensor_positions.items():
#             self.ax.scatter(sx, sy, color='blue', s=50, label=f'Sensor {sensor_id}')
#             self.ax.text(sx, sy, f'Sensor {sensor_id}\n({sx:.1f}, {sy:.1f})',
#                          fontsize=8, ha='center', va='bottom')

#         # 繪製原始和濾波後的位置
#         self.ax.scatter(triangulation_x, triangulation_y, color='orange', s=50, alpha=0.6, label='tria')
#         self.ax.scatter(nls_x, nls_y, color='red', s=50, label='nls')

#         # 添加位置標籤
#         self.ax.text(triangulation_x, triangulation_y, f'tria\n({triangulation_x:.2f}, {triangulation_y:.2f})',
#                      fontsize=9, ha='right', color='orange')
#         self.ax.text(nls_x, nls_y, f'nls\n({nls_x:.2f}, {nls_y:.2f})',
#                      fontsize=9, ha='left', color='red')

#         # 繪製由四個點所圍成的矩形
#         rectangle_points = [(320, 130), (-200, 130), (-200, -140), (320, -140), (320, 130)]
#         rect_x, rect_y = zip(*rectangle_points)
#         self.ax.plot(rect_x, rect_y, color='green', linewidth=2, label='Bounding Rectangle')

#         # 設置固定顯示範圍
#         self.ax.set_xlim(-1000, 1000)
#         self.ax.set_ylim(-1000, 1000)
#         self.ax.legend(loc='upper right')
#         plt.pause(0.1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = VisualizationNode()

#     try:
#         plt.ion()  # 開啟互動模式
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         plt.close()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped
# import matplotlib.pyplot as plt

# class FilteredComparisonNode(Node):
#     def __init__(self):
#         super().__init__('filtered_comparison_node')

#         # 訂閱兩組過濾後的位置
#         self.create_subscription(PointStamped, '/uwb/filter_position', self.group1_callback, 10)
#         self.create_subscription(PointStamped, '/new/uwb/filter_position', self.group2_callback, 10)

#         self.group1_position = None
#         self.group2_position = None

#         self.sensor_positions = {
#             0: (26.0,   0.0),
#             2: (-26.0, 0.0),
#             3: (33.0, -102.0),
#             5: (-33.0, -102.0)
#         }

#         self.figure, self.ax = plt.subplots(figsize=(8, 8))
#         self.visualization_initialized = False

#     def group1_callback(self, msg):
#         self.group1_position = (-msg.point.y, msg.point.x)
#         self.update_visualization()

#     def group2_callback(self, msg):
#         self.group2_position = (-msg.point.y, msg.point.x)
#         self.update_visualization()

#     def update_visualization(self):
#         if self.group1_position is None or self.group2_position is None:
#             return
#         self.visualize()

#     def visualize(self):
#         if not self.visualization_initialized:
#             self.ax.set_xlim(-1000, 1000)
#             self.ax.set_ylim(-1000, 1000)
#             self.ax.set_title('Filtered Position Comparison')
#             self.ax.set_xlabel('X (cm)')
#             self.ax.set_ylabel('Y (cm)')
#             self.ax.grid(True)
#             self.visualization_initialized = True

#         self.ax.clear()

#         # 畫感應器位置
#         for sid, (sx, sy) in self.sensor_positions.items():
#             self.ax.scatter(sx, sy, color='blue', s=50)
#             self.ax.text(sx, sy, f'Sensor {sid}', fontsize=8, ha='center', va='bottom')

#         # 畫 group 1
#         x1, y1 = self.group1_position
#         self.ax.scatter(x1, y1, color='red', s=50, label='Group 1')
#         self.ax.text(x1, y1, f'G1\n({x1:.2f}, {y1:.2f})', fontsize=9, ha='right', color='red')

#         # 畫 group 2
#         x2, y2 = self.group2_position
#         self.ax.scatter(x2, y2, color='blue', s=50, label='Group 2')
#         self.ax.text(x2, y2, f'G2\n({x2:.2f}, {y2:.2f})', fontsize=9, ha='left', color='blue')

#         # 畫邊界
#         rect = [(280, 130), (-250, 130), (-250, -130), (280, -130), (280, 130)]
#         rx, ry = zip(*rect)
#         self.ax.plot(rx, ry, color='black', linewidth=2, label='Area Boundary')

#         self.ax.set_xlim(-1000, 1000)
#         self.ax.set_ylim(-1000, 1000)
#         self.ax.legend(loc='upper right')
#         plt.pause(0.1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FilteredComparisonNode()
#     try:
#         plt.ion()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         plt.close()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #紀錄單一uwb即時位置
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped
# import matplotlib.pyplot as plt

# class UWBVisualizationNode(Node):
#     def __init__(self):
#         super().__init__('uwb_visualization_node')

#         self.create_subscription(PointStamped, '/uwb/filter_position', self.uwb_callback, 10)

#         self.uwb_position = None

#         self.sensor_positions = {
#             0: (26.0, 0.0),
#             2: (-26.0, 0.0),
#             3: (33.0, -102.0),
#             5: (-33.0, -102.0)
#         }

#         self.figure, self.ax = plt.subplots(figsize=(8, 8))
#         self.visualization_initialized = False

#     def uwb_callback(self, msg):
#         self.uwb_position = (-msg.point.y, msg.point.x)
#         self.visualize()

#     def visualize(self):
#         if self.uwb_position is None:
#             return

#         if not self.visualization_initialized:
#             self.ax.set_xlim(-1000, 1000)
#             self.ax.set_ylim(-1000, 1000)
#             self.ax.set_title('UWB Filtered Position')
#             self.ax.set_xlabel('X (cm)')
#             self.ax.set_ylabel('Y (cm)')
#             self.ax.grid(True)
#             self.visualization_initialized = True

#         self.ax.clear()

#         # 畫感應器位置
#         for sid, (sx, sy) in self.sensor_positions.items():
#             self.ax.scatter(sx, sy, color='blue', s=50)
#             self.ax.text(sx, sy, f'Sensor {sid}', fontsize=8, ha='center', va='bottom')

#         # 畫 UWB 位置
#         x, y = self.uwb_position
#         self.ax.scatter(x, y, color='red', s=50, label='UWB Position')
#         self.ax.text(x, y, f'({x:.2f}, {y:.2f})', fontsize=9, ha='right', color='red')

#         # 畫邊界
#         rect = [(280, 130), (-250, 130), (-250, -130), (280, -130), (280, 130)]
#         rx, ry = zip(*rect)
#         self.ax.plot(rx, ry, color='black', linewidth=2, label='Area Boundary')

#         self.ax.set_xlim(-1000, 1000)
#         self.ax.set_ylim(-1000, 1000)
#         self.ax.legend(loc='upper right')
#         plt.pause(0.1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = UWBVisualizationNode()
#     try:
#         plt.ion()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         plt.close()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
import threading
import csv
from datetime import datetime
import sys
import select

class UWBPathRecorderNode(Node):
    def __init__(self):
        super().__init__('uwb_path_recorder_node')

        self.create_subscription(PointStamped, '/uwb/filter_position', self.uwb_callback, 10)

        self.uwb_position = None
        self.recording = False
        self.path = []

        self.sensor_positions = {
            0: (26.0, 0.0),
            2: (-26.0, 0.0),
            3: (33.0, -102.0),
            5: (-33.0, -102.0)
        }

        self.figure, self.ax = plt.subplots(figsize=(8, 8))
        self.visualization_initialized = False

        # 啟動鍵盤監聽執行緒
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def keyboard_listener(self):
        print("[INFO] Press 'p' to start recording, 'q' to stop and save.")
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip()
                if key == 'p':
                    self.recording = True
                    self.path.clear()
                    print("[INFO] Start recording path...")
                elif key == 'q':
                    self.recording = False
                    print(f"[INFO] Stop recording. Total points: {len(self.path)}")
                    self.save_path()

    def uwb_callback(self, msg):
        self.uwb_position = (-msg.point.y, msg.point.x)
        if self.recording:
            self.path.append(self.uwb_position)
        self.visualize()

    def visualize(self):
        if self.uwb_position is None:
            return

        if not self.visualization_initialized:
            self.ax.set_xlim(-1000, 1000)
            self.ax.set_ylim(-1000, 1000)
            self.ax.set_title('UWB Position and Path')
            self.ax.set_xlabel('X (cm)')
            self.ax.set_ylabel('Y (cm)')
            self.ax.grid(True)
            self.visualization_initialized = True

        self.ax.clear()

        # 畫感應器
        for sid, (sx, sy) in self.sensor_positions.items():
            self.ax.scatter(sx, sy, color='blue', s=50)
            self.ax.text(sx, sy, f'Sensor {sid}', fontsize=8, ha='center', va='bottom')

        # 畫目前位置
        x, y = self.uwb_position
        self.ax.scatter(x, y, color='red', s=50, label='Current Position')
        self.ax.text(x, y, f'({x:.2f}, {y:.2f})', fontsize=9, ha='right', color='red')

        # 畫軌跡
        if self.path:
            px, py = zip(*self.path)
            self.ax.plot(px, py, linestyle='--', color='green', linewidth=2, label='Recorded Path')
            self.ax.scatter(px[0], py[0], color='black', marker='o', label='Start')
            self.ax.scatter(px[-1], py[-1], color='orange', marker='x', label='End')

        # 畫邊界
        rect = [(280, 130), (-250, 130), (-250, -130), (280, -130), (280, 130)]
        rx, ry = zip(*rect)
        self.ax.plot(rx, ry, color='black', linewidth=2, label='Area Boundary')

        self.ax.set_xlim(-1000, 1000)
        self.ax.set_ylim(-1000, 1000)
        self.ax.legend(loc='upper right')
        plt.pause(0.1)

    def save_path(self):
        if not self.path:
            print("[WARN] No path to save.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"recorded_path_{timestamp}.csv"
        png_filename = f"trajectory_{timestamp}.png"

        # 儲存 CSV
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["X (cm)", "Y (cm)"])
            writer.writerows(self.path)
        print(f"[INFO] Path saved to {csv_filename}")

        # 儲存 PNG
        px, py = zip(*self.path)
        plt.figure(figsize=(8, 6))
        plt.plot(px, py, color='black', linewidth=2, label='Trajectory')
        plt.scatter(px[0], py[0], color='green', label='Start')
        plt.scatter(px[-1], py[-1], color='red', label='End')
        plt.title("UWB Recorded Trajectory")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.savefig(png_filename)
        plt.close()
        print(f"[INFO] Trajectory image saved to {png_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = UWBPathRecorderNode()
    try:
        plt.ion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
