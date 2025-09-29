#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading

def clamp(x, a, b):
    return a if x < a else b if x > b else x

class TwistToPWM(Node):
    def __init__(self):
        super().__init__('twist_to_pwm_serial')

        # ---- 參數（可用 ros2 param 設定或這裡修改）----
        self.declare_parameter('port', '/dev/arduino')       # 也可用 /dev/ttyUSB0
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.30)           # 輪距 (m)
        self.declare_parameter('max_linear', 0.8)            # 最大線速 (m/s) 對應 |PWM|=255
        self.declare_parameter('max_angular', 2.5)           # 最大角速 (rad/s) 對應 |PWM|=255
        self.declare_parameter('topic', '/cmd_vel')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.L = float(self.get_parameter('wheel_base').value)
        self.vmax = float(self.get_parameter('max_linear').value)
        self.wmax = float(self.get_parameter('max_angular').value)

        # ---- 串口 ----
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f'Opened serial: {self.ser.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')
            raise

        # ---- 訂閱 ----
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Twist, topic, self.cb_twist, 10)

        # 生命週期保護（關閉時停車）
        self._lock = threading.Lock()
        self._last_cmd = (0,0)

    def cb_twist(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # 差速：左右輪線速（簡化：輪半徑並入比例）
        v_l = v - 0.5 * self.L * w
        v_r = v + 0.5 * self.L * w

        # 正規化到 -1..+1
        # 這裡用 max(vmax, L*wmax/2) 確保角速時也不會超界
        scale = max(self.vmax, 0.5*self.L*self.wmax)
        nl = clamp(v_l / scale, -1.0, 1.0)
        nr = clamp(v_r / scale, -1.0, 1.0)

        # 映射至 -255..+255（直流馬達 duty）
        pl = int(round(nl * 255))
        pr = int(round(nr * 255))

        # Deadband（避免微抖）
        if abs(pl) < 5: pl = 0
        if abs(pr) < 5: pr = 0

        with self._lock:
            self._last_cmd = (pl, pr)

        # 串口傳輸：M,<L>,<R>\n
        try:
            cmd = f"M,{pl},{pr}\n".encode('ascii')
            self.ser.write(cmd)
        except Exception as e:
            self.get_logger().warn(f'Write failed: {e}')

    def destroy_node(self):
        # 停車
        try:
            self.ser.write(b"M,0,0\n")
        except:
            pass
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = TwistToPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
