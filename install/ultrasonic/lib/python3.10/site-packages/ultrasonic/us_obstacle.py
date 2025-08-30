import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import MarkerArray, Marker

class UltrasonicCircleIntersectionNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_circle_intersection_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/us/raw_data',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, '/us/obstacle', 10)

        # 感測器參數 (位置, 中心朝向角度 deg)
        self.sensors = [
            {'pos': np.array([0.33, -0.225]), 'angle_deg': -45},
            {'pos': np.array([0.33, -0.075]), 'angle_deg': -15},
            {'pos': np.array([0.33,  0.000]), 'angle_deg':   0},
            {'pos': np.array([0.33,  0.075]), 'angle_deg':  15},
            {'pos': np.array([0.33,  0.225]), 'angle_deg':  45},
        ]
        self.fov_half_angle = 30  # 半角度

    def listener_callback(self, msg):
        distances = msg.data[:5]
        obstacles = []
        used = [False] * 5

        for i in range(len(self.sensors) - 1):
            p1 = self.sensors[i]['pos']
            p2 = self.sensors[i + 1]['pos']
            r1 = distances[i]
            r2 = distances[i + 1]

            if r1 <= 0.01 or r2 <= 0.01:
                continue

            intersections = self.circle_intersections(p1, r1, p2, r2)
            if intersections is not None:
                for pt in intersections:
                    if self.in_sector(p1, self.sensors[i]['angle_deg'], pt) and \
                       self.in_sector(p2, self.sensors[i+1]['angle_deg'], pt):
                        obstacles.append(pt)
                        used[i] = True
                        used[i + 1] = True
                        self.get_logger().info(f"[交會] S{i} & S{i+1} → (x={pt[0]:.3f}, y={pt[1]:.3f})")
                        break

        for i in range(5):
            if not used[i] and distances[i] > 0.01:
                angle_rad = -np.radians(self.sensors[i]['angle_deg'])
                dir_vec = np.array([np.cos(angle_rad), np.sin(angle_rad)])
                point = self.sensors[i]['pos'] + dir_vec * distances[i]
                obstacles.append(point)
                self.get_logger().info(f"[單點] S{i} → (x={point[0]:.3f}, y={point[1]:.3f})")

        # 發送 obstacle 座標
        data = []
        for pt in obstacles:
            data.extend([pt[0], pt[1]])
        self.publisher_.publish(Float32MultiArray(data=data))

    def circle_intersections(self, c0, r0, c1, r1):
        d = np.linalg.norm(c1 - c0)
        if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
            return None  # no intersection

        a = (r0**2 - r1**2 + d**2) / (2 * d)
        h = np.sqrt(max(0.0, r0**2 - a**2))
        p2 = c0 + a * (c1 - c0) / d

        offset = h * np.array([-(c1[1] - c0[1]) / d, (c1[0] - c0[0]) / d])
        i1 = p2 + offset
        i2 = p2 - offset
        return [i1, i2]

    def in_sector(self, origin, angle_deg, point):
        vec = point - origin
        angle_to_point = np.degrees(np.arctan2(vec[1], vec[0]))
        delta = (angle_to_point - angle_deg + 180) % 360 - 180
        return abs(delta) <= self.fov_half_angle

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicCircleIntersectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
