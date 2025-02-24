#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
import math
import random

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        # Publisher
        self.uwb_pub = self.create_publisher(PointStamped, '/uwb_filtered_position', 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, '/obstacle/xy_list', 10)

        # 定時器 (0.1 秒 = 10 Hz)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 用於計算車子虛擬路徑的時間變數
        self.t = 0.0

        # 1) 固定的障礙物 (原先範例的 4 個)
        self.fixed_obstacles = [
            (50.0, 50.0),
            (100.0, 50.0),
            (50.0, 100.0),
            (100.0, 100.0),
        ]

        # 2) 隨機生成 20 個障礙物，集中在「前方」區域
        #    這裡假設「前方」是 y ∈ [0, 300], x ∈ [-100, 100]
        self.random_obstacles = []
        for _ in range(20):
            x = random.uniform(-100, 100)
            y = random.uniform(0, 300)
            self.random_obstacles.append((x, y))

    def timer_callback(self):
        # ---- (A) 發布 /uwb_filtered_position ----
        # 以半徑 100 cm 的圓形軌跡來模擬車子前中心位置
        uwb_msg = PointStamped()
        uwb_msg.header.stamp = self.get_clock().now().to_msg()

        radius = 100.0
        uwb_msg.point.x = radius * math.cos(self.t)
        uwb_msg.point.y = radius * math.sin(self.t)
        uwb_msg.point.z = 0.0

        self.uwb_pub.publish(uwb_msg)

        # ---- (B) 發布 /obstacle/xy_list ----
        # 固定障礙物 + 隨機障礙物 一起打包
        all_obstacles = self.fixed_obstacles + self.random_obstacles
        obs_msg = Float32MultiArray()
        data = []
        for (ox, oy) in all_obstacles:
            data.append(ox)
            data.append(oy)
        obs_msg.data = data

        self.obs_pub.publish(obs_msg)

        # 時間往前推，讓車子持續繞圈
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
