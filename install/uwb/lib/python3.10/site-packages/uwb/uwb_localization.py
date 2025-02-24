import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import math
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')

        self.sensor_positions = {
            0: (0.0, 0.0, 0.0),
            2: (28.5, -44.0, 0.0),
            3: (-28.5, -44.0, 0.0),
            5: (0.0, -100.0, 0.0)
        }
        self.distances = {0: None, 2: None, 3: None, 5: None}

        self.subscribers = [
            self.create_subscription(Float32, f'uwb_filtered_data_{i}', self.create_callback(i), 10)
            for i in self.sensor_positions
        ]

        self.publisher = self.create_publisher(Point, '/uwb_raw_position', 10)
        self.timer = self.create_timer(0.1, self.calculate_position)

    def create_callback(self, sensor_id):
        def callback(msg):
            self.distances[sensor_id] = msg.data
        return callback

    def calculate_plane_distance(self, distance, z_sensor, z_person):
        try:
            return math.sqrt(distance**2 - (z_person - z_sensor)**2)
        except ValueError:
            self.get_logger().error(f'Invalid distance: distance={distance}, z_sensor={z_sensor}, z_person={z_person}')
            return None

    def calculate_position(self):
        if None in self.distances.values():
            self.get_logger().info('Waiting for all UWB data...')
            return

        positions = [self.sensor_positions[i] for i in self.sensor_positions]
        distances = [self.distances[i] for i in self.distances]
        z_person = 1.7

        plane_distances = []
        for distance, (x, y, z) in zip(distances, positions):
            plane_distance = self.calculate_plane_distance(distance, z, z_person)
            if plane_distance is None:
                return
            plane_distances.append(plane_distance)

        A = []
        b = []
        for (x0, y0, z0), d0_plane in zip(positions, plane_distances):
            for (x1, y1, z1), d1_plane in zip(positions[1:], plane_distances[1:]):
                A.append([2 * (x1 - x0), 2 * (y1 - y0)])
                b.append(d0_plane**2 - d1_plane**2 - x0**2 + x1**2 - y0**2 + y1**2)

        A = np.array(A)
        b = np.array(b)

        if np.linalg.cond(A) > 1e12:
            self.get_logger().error('Ill-conditioned matrix, skipping calculation.')
            return

        try:
            position = np.linalg.lstsq(A, b, rcond=None)[0]
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f'Failed to solve: {e}')
            return

        raw_position = Point()
        raw_position.x = position[0]
        raw_position.y = position[1]
        raw_position.z = z_person

        self.publisher.publish(raw_position)
        self.get_logger().info(f'Published raw position: x={raw_position.x:.2f}, y={raw_position.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
