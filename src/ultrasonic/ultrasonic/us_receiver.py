# 使用時若找不到板子,使用sudo apt purge brltty並重開機
# 確認串口 ls /dev/ttyUSB* /dev/ttyACM*
# 開啟對應權限sudo chmod 777 /dev/ttyUSB0
# 查看串口 ls /dev/serial/by-id/ -l

# 查看即時topic內容 ros2 topic echo /us_raw_data

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray
import struct

class UltrasonicReceiver(Node):
    def __init__(self):
        super().__init__('ultrasonic_receiver')

        self.declare_parameter('port', '/dev/ttyCH9344USB5')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.publisher_ = self.create_publisher(Float32MultiArray, '/us/raw_data', 10)
        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)

        self.buffer = bytearray()  # 累積讀取資料
        self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz

        self.get_logger().info(f'Listening on {port} at {baudrate} baud.')

    def read_serial_data(self):
        self.buffer += self.serial_port.read(100)  # 每次讀取較長的資料（防止截斷）

        while len(self.buffer) >= 21:
            # 嘗試尋找封包開頭
            idx = self.buffer.find(b'\x01\x03\x10')
            if idx == -1:
                self.buffer.clear()  # 找不到開頭，清除 buffer
                return

            if len(self.buffer) - idx < 21:
                # 封包長度不足，等待下次補齊
                break

            packet = self.buffer[idx:idx + 21]
            self.buffer = self.buffer[idx + 1:]  # 每次往後滑動一格（防止卡住）

            hex_data = packet.hex().upper()
            self.get_logger().info(f'Received: {hex_data}')

            if self.verify_crc(packet):
                sensor_values = self.parse_sensor_data(packet)
                if sensor_values:
                    msg = Float32MultiArray()
                    msg.data = sensor_values
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: {sensor_values}')
            else:
                self.get_logger().warn(f'CRC Error: {hex_data}')

    def parse_sensor_data(self, raw_data):
        """ 解析 8 個超聲波感測器數據，單位轉換為公尺 """
        try:
            sensor_values = []
            for i in range(3, 19, 2):  # 16 bytes sensor data
                value = int.from_bytes(raw_data[i:i+2], byteorder='big', signed=True)
                value = max(value, 0)
                sensor_values.append(value / 1000.0)  # mm ➜ m
            return sensor_values
        except Exception as e:
            self.get_logger().error(f'Parsing error: {e}')
            return None

    def verify_crc(self, data):
        if len(data) < 2:
            return False
        received_crc = data[-2:]
        calculated_crc = self.modbus_crc16(data[:-2])
        return received_crc == calculated_crc

    def modbus_crc16(self, data):
        """ 計算 Modbus CRC-16，回傳小端序 bytes """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
