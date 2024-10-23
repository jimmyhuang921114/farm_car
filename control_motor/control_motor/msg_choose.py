import rclpy
import math
import json
from dynamixel_sdk import *
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk_custom_interfaces.msg import SetVelocityDual
from ament_index_python.packages import get_package_share_directory
import os

class ControlSwitcher(Node):
    def __init__(self):
        super().__init__('control_switcher')

        # 訂閱遙控器控制和路徑規劃控制的話題
        self.remote_subscriber = self.create_subscription(
            Twist,
            '/rc_receiver',
            self.remote_callback,
            2)

        self.nav_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_callback,
            2)

        # 發佈最終的速度命令到 /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(SetVelocityDual, '/cmd_vel', 2)

        # 設定一個計時器，每0.1秒觸發一次
        self.timer = self.create_timer(0.01, self.publish_cmd)

        # 儲存最近的速度命令
        self.latest_remote_cmd = None
        self.latest_nav_cmd = None
        # 設定優先權標誌位
        self.remote_control_active = True

    def remote_callback(self, msg):
        # 接收到遙控器命令時，設置為遙控器模式
        self.remote_control_active = True
        self.latest_remote_cmd = msg

    def nav_callback(self, msg):
        # 如果沒有遙控器命令，才使用路徑規劃的命令
        if not self.remote_control_active:
            self.latest_nav_cmd = msg
            
    def publish_cmd(self):
        # 創建 SetVelocityDual 訊息
        msg = SetVelocityDual()

        # 檢查遙控器是否活躍並且有命令可以發佈
        if self.remote_control_active and self.latest_remote_cmd:
            # 從最新的遙控器命令 (Twist) 提取速度
            v = self.latest_remote_cmd.linear.x  # 前進速度
            W = self.latest_remote_cmd.angular.z  # 角速度
        elif self.latest_nav_cmd:
            # 從最新的路徑規劃命令 (Twist) 提取速度
            v = self.latest_nav_cmd.linear.x  # 前進速度
            W = self.latest_nav_cmd.angular.z  # 角速度
        else:
            return  # 沒有可用的命令發佈
        
        # 使用 ROS 2 的方法動態獲取 data.json 的路徑
        package_share_directory = get_package_share_directory('control_motor')
        file_path = os.path.join(package_share_directory, 'config', 'data.json')
        
        # 從 config 檔案中讀取參數
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
        L = data['wheel_base']  # 輪距
        r = data['wheel_radius']  # 輪子半徑

        # 使用 transform 函數計算輪子的速度
        W_L, W_R = transform(v, W, L, r)

        # 設定 SetVelocityDual 訊息中的速度
        msg.left_motor_speed = int(W_L)
        msg.right_motor_speed = int(W_R)

        # 發佈 SetVelocityDual 訊息
        self.cmd_vel_publisher.publish(msg)

def transform(v, W, L, r):
    # 計算左右輪子的速度
    left_speed = v - (L / 2) * W
    right_speed = v + (L / 2) * W

    # 計算輪子的角速度
    W_L = int(left_speed / r)
    W_R = int(right_speed /r)

    print(f'left_motor: {W_L:.2f} m/s')
    print(f'right_motor: {W_R:.2f} m/s')

    return W_L, W_R

def main(args=None):
    rclpy.init(args=args)
    control_switcher = ControlSwitcher()
    
    try:
        rclpy.spin(control_switcher)  # 持續運行節點
    finally:
        control_switcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
