# import sys
# import rclpy
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.node import Node
# import serial
# import threading
# from std_msgs.msg import Float32
# import time
# import traceback

# ID_TO_SERIAL_PORT = {
#     0: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02619997-if00-port0',
#     1: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_0111621A-if00-port0',
#     2: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199A8-if00-port0',
#     3: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199F2-if00-port0',
#     4: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF44C-if00-port0',
#     5: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF47D-if00-port0',
#     6: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF471-if00-port0'
# } 


# class UWBReceiver(Node):
#     def __init__(self, id, serial_port):
#         super().__init__(f'uwb_receiver_node_{id}')
#         self.id = id
#         self.serial_port = serial_port
#         self.publisher_ = self.create_publisher(Float32, f'uwb_raw_data_{id}', 10)
#         self.ser = None
#         self.reconnect_serial()

#         threading.Thread(target=self.read_from_serial, daemon=True).start()

#     def reconnect_serial(self):
#         while self.ser is None:
#             try:
#                 self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
#                 self.get_logger().info(f"Connected to serial port: {self.serial_port}")
#             except serial.SerialException as e:
#                 self.get_logger().error(f"Serial connection failed: {e}. Retrying in 5 seconds...")
#                 time.sleep(5)

#     def read_from_serial(self):
#         buffer = ""
#         while True:
#             try:
#                 if self.ser and self.ser.in_waiting > 0:
#                     chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
#                     buffer += chunk
#                     lines = buffer.split('\r\n')

#                     for line in lines[:-1]:
#                         try:
#                             distance_str = line.split(":")[1].strip().replace(' cm', '')
#                             distance = float(distance_str)
#                             if 0 <= distance <= 2000:  # 合理範圍
#                                 self.publish_distance(distance)
#                         except (ValueError, IndexError):
#                             self.get_logger().warning(f"Received invalid data: {line}")
#                             continue

#                     buffer = lines[-1]
#             except (serial.SerialException, OSError) as e:
#                 self.get_logger().error(f"Serial error: {e}. Attempting to reconnect...")
#                 self.ser = None
#                 self.reconnect_serial()
#             except Exception as e:
#                 self.get_logger().error(f"Unexpected error in read_from_serial: {e}")
#                 self.get_logger().error(traceback.format_exc())
#                 time.sleep(5)

#     def publish_distance(self, distance):
#         msg = Float32()
#         msg.data = round(distance, 2)
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published raw UWB data from ID {self.id}: Distance = {msg.data}")

# def main(args=None):
#     rclpy.init(args=args)

#     if len(sys.argv) < 2:
#         print("請指定要使用的 UWB ID，例如：ros2 run uwb_receiver 0")
#         sys.exit(1)

#     ids_to_use = [int(id) for id in sys.argv[1:] if id.isdigit()]
#     nodes = [UWBReceiver(id, ID_TO_SERIAL_PORT[id]) for id in ids_to_use]

#     executor = MultiThreadedExecutor()
#     for node in nodes:
#         executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         print("\nProgram interrupted by user.")
#     finally:
#         for node in nodes:
#             node.destroy_node()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import serial
import time
import traceback

# 只使用 UWB ID 2 的序列埠
SERIAL_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199A8-if00-port0'

def read_uwb_distance():
    ser = None
    while ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
            print(f"[INFO] 成功連接到序列埠: {SERIAL_PORT}")
        except serial.SerialException as e:
            print(f"[ERROR] 無法連接到序列埠: {e}，5 秒後重試...")
            time.sleep(5)

    buffer = ""
    while True:
        try:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += chunk
                lines = buffer.split('\r\n')

                for line in lines[:-1]:
                    try:
                        distance_str = line.split(":")[1].strip().replace(' cm', '')
                        distance = float(distance_str)
                        if 0 <= distance <= 2000:
                            print(f"[UWB 2] 距離: {distance:.2f} cm")
                    except (ValueError, IndexError):
                        print(f"[WARNING] 收到無效數據: {line}")
                        continue

                buffer = lines[-1]
        except (serial.SerialException, OSError) as e:
            print(f"[ERROR] Serial 錯誤: {e}，正在重新連接...")
            ser = None
            while ser is None:
                try:
                    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
                    print(f"[INFO] 重新連接成功")
                except serial.SerialException:
                    print("[ERROR] 重新連接失敗，5 秒後重試...")
                    time.sleep(5)
        except Exception as e:
            print(f"[UNEXPECTED ERROR] {e}")
            print(traceback.format_exc())
            time.sleep(5)

if __name__ == '__main__':
    read_uwb_distance()
