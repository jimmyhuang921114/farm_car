import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Rectangle
import numpy as np

class UltrasonicVisualizer(Node):
    def __init__(self):
        super().__init__('ultrasonic_visualizer')

        # 兩個訂閱主題：障礙物座標與感測器距離值
        self.subscription_obs = self.create_subscription(
            Float32MultiArray,
            '/us/obstacle',
            self.obs_callback,
            10)

        self.subscription_dist = self.create_subscription(
            Float32MultiArray,
            '/us/raw_data',
            self.dist_callback,
            10)

        self.obstacles = []  # [(x, y), ...]
        self.distances = [None, None, None, None]  # 4 個感測器距離（單位：公尺）

        # 感測器位置與角度設定
        self.sensors = [
            (0.33, -0.225, -45),
            (0.33, -0.075, -15),
            (0.33,  0.075,  15),
            (0.33,  0.225,  45)
        ]

        # Matplotlib 初始化
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-0.5, 2.5)
        self.ax.set_ylim(-1.2, 1.2)
        plt.ion()
        plt.show()

        # 定時更新畫面
        self.timer = self.create_timer(0.5, self.update_plot)

    def obs_callback(self, msg):
        self.obstacles = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        self.get_logger().info(f'收到障礙物點：{self.obstacles}')

    def dist_callback(self, msg):
        if len(msg.data) >= 4:
            self.distances = msg.data[:4]

    def draw_robot(self):
        car_length = 1.1
        car_width = 0.6
        car_front_x = 0.33
        car_start_x = car_front_x - car_length  # -0.77
        car_start_y = -car_width / 2            # -0.3

        car = Rectangle((car_start_x, car_start_y), car_length, car_width,
                        edgecolor='black', facecolor='none', linewidth=1.5)
        self.ax.add_patch(car)
        self.ax.text(car_front_x, car_width / 2 + 0.02, "Robot Front", ha='center', fontsize=9)

    def draw_sensor_fovs(self):
        for i, (x, y, angle) in enumerate(self.sensors):
            fov = Wedge((x, y), 2.0, angle - 30, angle + 30, alpha=0.15)
            self.ax.add_patch(fov)
            self.ax.plot(x, y, 'bo')

            # 加入感測器讀數（若存在）
            d = self.distances[i] if i < len(self.distances) else None
            if d is not None:
                self.ax.text(x - 0.05, y - 0.04, f"S{i+1}:{d:.2f}m", ha='right', fontsize=8, color='blue')
            else:
                self.ax.text(x - 0.05, y - 0.04, f"S{i+1}:---", ha='right', fontsize=8, color='gray')

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Ultrasonic Obstacle Visualization")
        self.ax.set_xlim(-0.5, 2.5)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        self.draw_robot()
        self.draw_sensor_fovs()

        if self.obstacles:
            for i, (x, y) in enumerate(self.obstacles):
                self.ax.scatter(x, y, s=80, color='red')
                self.ax.text(x + 0.05, y, f"({x:.2f}, {y:.2f})", fontsize=8, color='red', ha='left')

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
