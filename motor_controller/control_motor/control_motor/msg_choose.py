import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlSwitcher(Node):
    def __init__(self):
        super().__init__('control_switcher')

        # 訂閱遙控器控制和路徑規劃控制的話題
        self.remote_subscriber = self.create_subscription(
            Twist,
            '/controller',
            self.remote_callback,
            10)

        self.nav_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_callback,
            10)

        # 發佈最終的速度命令到 /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 儲存最近的速度命令
        self.latest_remote_cmd = None
        self.latest_nav_cmd = None

        # 設定優先權標誌位
        self.remote_control_active = False

    def remote_callback(self, msg):
        # 接收到遙控器命令時，設置為遙控器模式
        self.remote_control_active = True
        self.latest_remote_cmd = msg
        self.publish_cmd()

    def nav_callback(self, msg):
        # 如果沒有遙控器命令，才使用路徑規劃的命令
        if not self.remote_control_active:
            self.latest_nav_cmd = msg
            self.publish_cmd()

    def publish_cmd(self):
        if self.remote_control_active and self.latest_remote_cmd:
            self.cmd_vel_publisher.publish(self.latest_remote_cmd)
        elif self.latest_nav_cmd:
            self.cmd_vel_publisher.publish(self.latest_nav_cmd)

    # def reset_remote_control(self):
    #     self.remote_control_active = False

def main(args=None):
    rclpy.init(args=args)
    control_switcher = ControlSwitcher()
    rclpy.spin(control_switcher)    
    control_switcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
