#!/usr/bin/env python3
# rcin_listener.py
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import Twist

def us_to_unit(us, us_min=1000.0, us_mid=1500.0, us_max=2000.0):
    if us <= us_mid:
        return max(-1.0, (us - us_mid) / (us_mid - us_min))
    return min( 1.0, (us - us_mid) / (us_max - us_mid))

def apply_deadband(x, db):
    return 0.0 if abs(x) < db else x

class RCInListener(Node):
    def __init__(self):
        super().__init__('rcin_listener')

        self.declare_parameter('ch_steer', 0)       # CH1 → index 0
        self.declare_parameter('ch_throttle', 2)    # CH3 → index 2
        self.declare_parameter('deadband', 0.05)    # -1..+1
        self.declare_parameter('max_linear', 1.0)   # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s

        self.ch_steer    = int(self.get_parameter('ch_steer').value)
        self.ch_throttle = int(self.get_parameter('ch_throttle').value)
        self.deadband    = float(self.get_parameter('deadband').value)
        self.max_linear  = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.sub = self.create_subscription(RCIn, '/mavros/rc/in', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/rc/cmd_vel', 10)

        self.get_logger().info('RCInListener ready (listening /mavros/rc/in → /rc/cmd_vel)')

    def cb(self, msg: RCIn):
        ch = msg.channels
        steer_us    = ch[self.ch_steer]    if len(ch) > self.ch_steer    else 1500
        throttle_us = ch[self.ch_throttle] if len(ch) > self.ch_throttle else 1500

        steer   = apply_deadband(us_to_unit(steer_us), self.deadband)     
        throttle= apply_deadband(us_to_unit(throttle_us), self.deadband)  

        # 對應到速度（給下游參考使用，可自行調尺度）
        twist = Twist()
        twist.linear.x  = throttle * self.max_linear
        twist.angular.z = steer    * self.max_angular
        self.pub.publish(twist)

        # 也印原始 μs（除錯）
        self.get_logger().debug(f"RC: ch{self.ch_steer+1}={steer_us}us  ch{self.ch_throttle+1}={throttle_us}us  "
                                f"→ vel=({twist.linear.x:.2f}, {twist.angular.z:.2f})")

def main():
    rclpy.init()
    node = RCInListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
