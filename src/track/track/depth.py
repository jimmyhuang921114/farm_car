import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthNode(Node):
    def __init__(self):
        super().__init__('track_Depth')

        self.bridge = CvBridge()
        self.depth_image = None

        # cam parameter
        self.fx = 615.0
        self.fy = 615.0
        self.ppx = 320.0
        self.ppy = 240.0

        # sub the depth and posearray
        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Point, 'track/object', self.object_callback, 10)

        #pub by single topic (people and obstacle)
        self.object_pub = self.create_publisher(Point, '/track/object_3d', 10)

        self.get_logger().info("object 2d to 3d")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {str(e)}")

    def object_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().warn("No depth image received yet")
            return

        u = int(msg.y)
        v = int(msg.x)

        if 0 <= u < self.depth_image.shape[0] and 0 <= v < self.depth_image.shape[1]:
            x, y, z = self.convert_to_3d(u, v)

            if x is not None and y is not None and z is not None:
                new_point_msg = Point()
                new_point_msg.x = x
                new_point_msg.y = y
                new_point_msg.z = z

                self.object_pub.publish(new_point_msg)
                self.get_logger().info(f"[2D→3D] Published 3D point: ({x:.2f}, {y:.2f}, {z:.2f})")
            else:
                self.get_logger().warn("Valid depth not found at target location.")
        else:
            self.get_logger().warn("Target point outside depth image bounds.")


    def convert_to_3d(self, u, v, window_size=5):
        half_size = window_size // 2
        h, w = self.depth_image.shape
        valid_depths = []

        for dy in range(-half_size, half_size + 1):
            for dx in range(-half_size, half_size + 1):
                uu = u + dy
                vv = v + dx
                if 0 <= uu < h and 0 <= vv < w:
                    depth = self.depth_image[uu, vv]
                    if depth > 0:
                        valid_depths.append(depth)

        if valid_depths:
            avg_depth = np.mean(valid_depths) / 1000.0  
            x = (v - self.ppx) * avg_depth / self.fx
            y = (u - self.ppy) * avg_depth / self.fy
            z = avg_depth
            return x, y, z
        else:
            return None, None, None

def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
