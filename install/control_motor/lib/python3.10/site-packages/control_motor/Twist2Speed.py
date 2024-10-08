import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetVelocityDual

class TwistToOKPublisherNode:
    def __init__(self):
        self.node = rclpy.create_node('twist_to_ok_publisher')

        self.subscription = self.node.create_subscription(
            Twist,
            'cmd_vel', 
            self.twist_callback,
            10)
        self.subscription 

        self.publisher = self.node.create_publisher(SetVelocityDual, '/motor_dual_speed', 10)

    def twist_callback(self, msg):
        self.node.get_logger().info("Received Twist message: Linear = %f, Angular = %f" % (msg.linear.x, msg.angular.z))

        speed_wish_right = int((msg.angular.z*160)/2 + msg.linear.x*1000)
        speed_wish_left = int(msg.linear.x*2000-speed_wish_right)

        print(speed_wish_right, speed_wish_left)

        speed_msg = SetVelocityDual()

        speed_msg.motorspeed1 = speed_wish_left 
        speed_msg.motorspeed2 = -speed_wish_right 

        self.publisher.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToOKPublisherNode()
    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
