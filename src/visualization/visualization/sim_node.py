#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int32MultiArray
from sensor_msgs.msg import Image

import math
import random
import numpy as np
from cv_bridge import CvBridge

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        # Publishers
        self.uwb_pub = self.create_publisher(PointStamped, '/uwb_filtered_position', 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, '/obstacle/xy_list', 10)

        # Four image publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/color', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth', 10)
        self.tracking_pub = self.create_publisher(Image, '/tracking_image', 10)
        self.masked_pub = self.create_publisher(Image, '/masked_image', 10)

        # Two ultrasonic sensor publishers (8 readings each)
        self.ultra1_pub = self.create_publisher(Int16MultiArray, '/ultrasonic_data_1', 10)
        self.ultra2_pub = self.create_publisher(Int16MultiArray, '/ultrasonic_data_2', 10)
        
        # 新增：people_track publisher (格式為 Int32MultiArray)
        self.people_track_pub = self.create_publisher(Int32MultiArray, '/people_track', 10)

        # For converting between OpenCV images and ROS Image messages
        self.bridge = CvBridge()

        # Timer (0.1 seconds = 10 Hz)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Time variable for simulating a circular path
        self.t = 0.0

        # (1) Fixed obstacles
        self.fixed_obstacles = [
            (50.0, 50.0),
            (100.0, 50.0),
            (50.0, 100.0),
            (100.0, 100.0),
        ]

        # (2) 20 random obstacles in "front" region: y ∈ [0, 300], x ∈ [-100, 100]
        self.random_obstacles = []
        for _ in range(20):
            x = random.uniform(-100, 100)
            y = random.uniform(0, 300)
            self.random_obstacles.append((x, y))

    def timer_callback(self):
        # ---------------- (A) Publish /uwb_filtered_position ----------------
        uwb_msg = PointStamped()
        uwb_msg.header.stamp = self.get_clock().now().to_msg()

        # Circle radius 100 cm
        radius = 100.0
        uwb_msg.point.x = radius * math.cos(self.t)
        uwb_msg.point.y = radius * math.sin(self.t)
        uwb_msg.point.z = 0.0

        self.uwb_pub.publish(uwb_msg)

        # ---------------- (B) Publish /obstacle/xy_list ----------------
        all_obstacles = self.fixed_obstacles + self.random_obstacles
        obs_msg = Float32MultiArray()
        data = []
        for (ox, oy) in all_obstacles:
            data.append(ox)
            data.append(oy)
        obs_msg.data = data
        self.obs_pub.publish(obs_msg)

        # ---------------- (C) Publish 4 images (random) ----------------
        height, width = 240, 320

        # 1) RGB Image
        rgb_img = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding='bgr8')
        self.rgb_pub.publish(rgb_msg)

        # 2) Depth Image (mono8 for simplicity, random grayscale)
        depth_img = np.random.randint(0, 256, (height, width), dtype=np.uint8)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='mono8')
        self.depth_pub.publish(depth_msg)

        # 3) Tracking Image (RGB)
        tracking_img = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        tracking_msg = self.bridge.cv2_to_imgmsg(tracking_img, encoding='bgr8')
        self.tracking_pub.publish(tracking_msg)

        # 4) Masked Image (RGB)
        masked_img = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        masked_msg = self.bridge.cv2_to_imgmsg(masked_img, encoding='bgr8')
        self.masked_pub.publish(masked_msg)

        # ---------------- (D) Publish ultrasonic data (2 sets) ----------------
        u1_msg = Int16MultiArray()
        u2_msg = Int16MultiArray()
        u1_msg.data = [random.randint(0, 200) for _ in range(8)]
        u2_msg.data = [random.randint(0, 200) for _ in range(8)]
        self.ultra1_pub.publish(u1_msg)
        self.ultra2_pub.publish(u2_msg)

        # ---------------- (E) Publish simulated /people_track data ----------------
        # 模擬 3 個隨機的 track 點 (每兩個數值代表一個 (x, y) 座標，單位：cm)
        people_track_msg = Int32MultiArray()
        track_data = []
        num_tracks = 3
        for _ in range(num_tracks):
            x = random.randint(-200, 200)
            y = random.randint(-200, 200)
            track_data.extend([x, y])
        people_track_msg.data = track_data
        self.people_track_pub.publish(people_track_msg)

        # Increase time for circular motion
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
