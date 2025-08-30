#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool

class GuidedVelBridge(Node):
    def __init__(self):
        super().__init__('guided_vel_bridge')

        # params
        self.declare_parameter('input_topic', '/cmd_vel')         # 你上游丟 Twist 的 topic
        self.declare_parameter('mode_name', 'GUIDED_NOGPS')       # GUIDED or GUIDED_NOGPS
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('rate_hz', 20.0)                   # 送 setpoint 的頻率
        self.declare_parameter('timeout', 0.5)                    # 沒新指令多久視為逾時

        self.input_topic = self.get_parameter('input_topic').value
        self.mode_name   = self.get_parameter('mode_name').value
        self.auto_arm    = bool(self.get_parameter('auto_arm').value)
        self.rate_hz     = float(self.get_parameter('rate_hz').value)
        self.timeout     = float(self.get_parameter('timeout').value)

        # io
        self.sub  = self.create_subscription(Twist, self.input_topic, self.cb_cmd, 10)
        self.pub  = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.srv_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.srv_arm  = self.create_client(CommandBool, '/mavros/cmd/arming')

        # state
        self.last_cmd = Twist()
        self.last_t   = 0.0
        self.activated = False

        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)
        self.get_logger().info(f'guided_vel_bridge listening on {self.input_topic}, target mode={self.mode_name}')

    def cb_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_t = time.time()
        if not self.activated:
            self.activate()

    def activate(self):
        # 切模式
        if not self.srv_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('set_mode service not ready yet')
            return
        reqm = SetMode.Request()
        reqm.base_mode = 0
        reqm.custom_mode = self.mode_name
        self.srv_mode.call_async(reqm)
        self.get_logger().info(f'set_mode -> {self.mode_name}')

        # ARM
        if self.auto_arm and self.srv_arm.wait_for_service(timeout_sec=2.0):
            reqa = CommandBool.Request()
            reqa.value = True
            self.srv_arm.call_async(reqa)
            self.get_logger().info('arming...')

        self.activated = True

    def loop(self):
        now = time.time()
        twist = Twist()
        if (now - self.last_t) <= self.timeout:
            twist = self.last_cmd   # 用最後一筆
        else:
            # 逾時 → 煞停
            twist.linear.x = twist.linear.y = twist.linear.z = 0.0
            twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = GuidedVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
