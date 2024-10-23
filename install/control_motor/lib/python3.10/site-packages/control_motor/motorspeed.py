import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetVelocityDual  # 使用正確的消息類型

class TwistMotorSpeed(Node):
    def __init__(self):
        # 使用 Node 的繼承方式創建 ROS2 節點
        super().__init__('twist_publish')

        # 創建一個 SetVelocityDual 消息的發佈器
        self.publisher = self.create_publisher(SetVelocityDual, 'motor_dual_speed', 10)

        # 設置計時器，每 1 秒調用一次 send_velocity 函數
        self.timer_period = 1.0  # 1秒
        self.timer = self.create_timer(self.timer_period, self.send_velocity)

    def send_velocity(self):
        # 創建一個 SetVelocityDual 消息
        velocity_msg = SetVelocityDual()

        # 設置左、右電機速度
        velocity_msg.motorspeed1 = 100 # 左電機速度
        velocity_msg.motorspeed2 = 0  # 右電機速度

        # 發佈 SetVelocityDual 消息
        self.publisher.publish(velocity_msg)

        # 日誌輸出
        self.get_logger().info(f"Publishing SetVelocityDual: left_velocity = {velocity_msg.motorspeed1}, right_velocity = {velocity_msg.motorspeed2s}")

def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)

    # 創建 TwistMotorSpeed 類的實例
    motor_speed_publisher = TwistMotorSpeed()

    # 讓節點持續運行
    rclpy.spin(motor_speed_publisher)

    # 銷毀節點
    motor_speed_publisher.destroy_node()

    # 關閉 ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
