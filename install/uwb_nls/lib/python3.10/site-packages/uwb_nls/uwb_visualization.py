import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
import matplotlib.pyplot as plt

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        # 訂閱過濾後的位置
        self.triangulation = self.create_subscription(
            PointStamped, '/uwb_filtered_position', self.triangulation_callback, 10)

        # 訂閱過濾前的位置
        self.nls = self.create_subscription(
            PointStamped, '/uwb_filtered_position_nls', self.nls_callback, 10)

        # 感應器位置定義
        self.sensor_positions = {
            0: (0.0, 0.0),
            2: (28.5, -44.0),
            3: (-28.5, -44.0),
            5: (0.0, -100.0)
        }
        # self.sensor_positions = {
        #     0: (0.0, 30.0),
        #     2: (0.0, -30.0),
        #     3: (-97.0, 30.0),
        #     5: (-97.0, -30.0)
        # }

        self.figure, self.ax = plt.subplots(figsize=(8, 8))
        self.visualization_initialized = False

        # 初始位置記錄
        self.triangulation_position = None
        self.nls_position = None

    def triangulation_callback(self, msg):
        """接收過濾前的位置"""
        self.triangulation_position = (msg.point.x, msg.point.y)

    def nls_callback(self, msg):
        """接收過濾後的位置"""
        self.nls_position = (msg.point.x, msg.point.y)
        self.update_visualization()

    def update_visualization(self):
        if self.triangulation_position is None or self.nls_position is None:
            # 如果任何一個數據未到達，等待
            return

        triangulation_x, triangulation_y = self.triangulation_position
        nls_x, nls_y = self.nls_position

        self.visualize(triangulation_x, triangulation_y, nls_x, nls_y)

    # 視覺化方法
    def visualize(self, triangulation_x, triangulation_y, nls_x, nls_y):
        if not self.visualization_initialized:
            # 設置繪圖的固定範圍和標籤
            self.ax.set_xlim(-1000, 1000)
            self.ax.set_ylim(-1000, 1000)
            self.ax.set_title('UWB Positioning Visualization')
            self.ax.set_xlabel('X (cm)')
            self.ax.set_ylabel('Y (cm)')
            self.ax.grid(True)
            self.visualization_initialized = True

        # 清除舊圖
        self.ax.clear()

        # 繪製感應器位置
        for sensor_id, (sx, sy) in self.sensor_positions.items():
            self.ax.scatter(sx, sy, color='blue', label=f'Sensor {sensor_id}', s=50)
            self.ax.text(sx, sy, f'Sensor {sensor_id}\n({sx:.1f}, {sy:.1f})', fontsize=8, ha='center', va='bottom')

        # 繪製原始和濾波後的位置
        self.ax.scatter(triangulation_x, triangulation_y, color='orange', label='tria', s=50, alpha=0.6)
        self.ax.scatter(nls_x, nls_y, color='red', label='nls', s=50)

        # 添加標籤
        self.ax.text(triangulation_x, triangulation_y, f'tria\n({triangulation_x:.2f}, {triangulation_y:.2f})', fontsize=9, ha='right', color='orange')
        self.ax.text(nls_x, nls_y, f'nls\n({nls_x:.2f}, {nls_y:.2f})', fontsize=9, ha='left', color='red')

        # 設置固定顯示範圍
        self.ax.set_xlim(-1000, 1000)
        self.ax.set_ylim(-1000, 1000)
        self.ax.legend(loc='upper right')
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

    try:
        plt.ion()  # 開啟互動模式
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
