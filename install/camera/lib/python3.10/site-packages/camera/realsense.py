import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')

        # Set up ROS 2 image publishers
        self.color_publisher = self.create_publisher(Image, '/camera/color', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)
        
        # OpenCV and ROS image conversion tool
        self.bridge = CvBridge()

        # Initialize RealSense camera
        self.pipeline = None
        self.config = rs.config()

        # Set image publishing frequency (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_frames)

        # Connection status tracking
        self.connected = False
        self.retry_count = 0
        self.max_retries = 5  # Maximum retry attempts

        # Attempt to connect to the camera
        self.connect_camera()

    def connect_camera(self):
        """ Attempt to connect to the RealSense camera """
        self.get_logger().info("Attempting to connect to RealSense camera...")
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()

            # Enable RGB and depth streams
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

            # Start RealSense pipeline
            self.pipeline.start(config)
            self.connected = True
            self.retry_count = 0  # Reset retry count
            self.get_logger().info("Successfully connected to RealSense camera")

        except Exception as e:
            self.connected = False
            self.get_logger().error(f"Failed to connect to RealSense: {str(e)}")
            self.retry_reconnect()

    def retry_reconnect(self):
        """ Retry connection if the camera is disconnected """
        if self.retry_count < self.max_retries:
            wait_time = 2 ** self.retry_count  # Exponential backoff strategy (2, 4, 8, 16, ...)
            self.get_logger().warn(f"Retrying connection in {wait_time} seconds... ({self.retry_count+1}/{self.max_retries})")
            time.sleep(wait_time)
            self.retry_count += 1
            self.connect_camera()
        else:
            self.get_logger().error("Maximum retry attempts reached. Please check the camera connection.")

    def publish_frames(self):
        """ Capture and publish images to ROS 2 topics """
        if not self.connected:
            self.get_logger().warn("RealSense camera is not connected. Unable to publish frames.")
            return

        try:
            # Capture frames
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                self.get_logger().warn("No valid frames received. Skipping this frame.")
                return

            # Convert to NumPy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert to ROS image messages
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')

            # Set timestamps
            now = self.get_clock().now().to_msg()
            color_msg.header.stamp = now
            depth_msg.header.stamp = now

            # Publish images
            self.color_publisher.publish(color_msg)
            self.depth_publisher.publish(depth_msg)

            self.get_logger().info("Successfully published images to /camera/color and /camera/depth")

        except Exception as e:
            self.get_logger().error(f"Error occurred while publishing frames: {str(e)}")
            self.connected = False
            self.retry_reconnect()

    def destroy(self):
        """ Release resources """
        if self.pipeline:
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping RealSense publisher node...")
    finally:
        node.destroy()


if __name__ == '__main__':
    main()