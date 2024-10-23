import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math 
import json
from dynamixel_sdk_custom_interfaces.msg import SetVelocityDual

class TwistToOKPublisherNode:
    def __init__(self):
        self.node = rclpy.create_node('twist_to_publish')

        self.subscription = self.node.create_subscription(
            Twist,
            'cmd_vel', 
            self.twist_callback,
            1)

    
        self.publisher = self.node.create_publisher(SetVelocityDual, '/cmd_vel', 1)
    
    def twist_callback(self, msg):
        self.node.get_logger().info("Received Twist message: Linear = %f, Angular = %f" % (msg.linear.x, msg.angular.z))
        file_path='data.json'
        with open(file_path,'r',encoding='utf-8') as file:
            data=json.load(file)
        v=10
        W=0.5*math.pi
        L=data['wheel_base']
        r=data['wheel_radius']
        
        left_speed,right_speed=calculate_motor_speed(v,W,L,r)
        speed_wish_right = int((msg.angular.z*160)/2 + msg.linear.x*1000)
        speed_wish_left = int(msg.linear.x*2000-speed_wish_right)
        
        speed_msg.motorspeed1 = left_speed 
        speed_msg.motorspeed2 = right_speed 
        
        self.publisher.publish(speed_msg)
        
def calculate_motor_speed(v,W,L,r):
    left_speed=v-(L/2)*W
    right_speed=v+(L/2)*W
    
    
    left_speed=left_speed
    right_speed=right_speed
    
    
    W_L=left_speed/r
    W_R=right_speed/r
    
    print(f'left_motor:{left_speed:.2f}m/s')
    print(f'right_motor:{right_speed:.2f}m/s')
    return W_L,W_R

def main(args=None):
    rclpy.init(args=args)
    node = TwistToOKPublisherNode()
    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
