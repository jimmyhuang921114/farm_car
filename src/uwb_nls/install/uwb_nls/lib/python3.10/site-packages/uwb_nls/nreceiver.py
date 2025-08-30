# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import serial
# import time

# PACKET_SIZE = 64
# HEADER = b'\x67\x69\x70\x73'
# BAUDRATE = 57600
# FPS = 30
# TIMEOUT_SEC = 0.1  # 超過這段時間沒更新，設為 None

# # anchor ID 映射到 UWB ID
# ANCHOR_ID_MAP = {
#     0x1504: 0,
#     0x1404: 1,
#     0x1304: 2,
#     0x1204: 3,
# }

# PORTS = [
#     "/dev/ttyUSB0",
#     "/dev/ttyUSB1",
#     "/dev/ttyUSB2",
#     "/dev/ttyUSB3",
# ]

# class UWBSerialSyncNode(Node):
#     def __init__(self):
#         super().__init__('uwb_serial_sync_node')

#         self.serial_ports = {}
#         self.buffers = {}

#         # 儲存目前距離資訊與對應的「最後更新時間」
#         self.latest_distances = [None] * 4
#         self.last_update_time = [0.0] * 4

#         # 建立 publisher
#         self.publisher_ = self.create_publisher(Float32MultiArray, 'uwb_distances', 10)

#         # 嘗試開啟所有 serial port
#         for port in PORTS:
#             try:
#                 ser = serial.Serial(port, BAUDRATE, timeout=0.01)
#                 self.serial_ports[port] = ser
#                 self.buffers[port] = bytearray()
#                 self.get_logger().info(f"✅ 已連接 {port}")
#             except serial.SerialException as e:
#                 self.get_logger().error(f"❌ 無法開啟 {port}: {e}")

#         # 每 1/30 秒執行一次 callback（30Hz）
#         self.create_timer(1.0 / FPS, self.timer_callback)

#     def timer_callback(self):
#         current_time = time.time()

#         # 讀取所有 serial port
#         for port, ser in self.serial_ports.items():
#             try:
#                 self.buffers[port] += ser.read(ser.in_waiting)

#                 while len(self.buffers[port]) >= PACKET_SIZE:
#                     header_index = self.buffers[port].find(HEADER)

#                     if header_index == -1:
#                         self.buffers[port] = self.buffers[port][-32:]
#                         break

#                     if len(self.buffers[port]) - header_index < PACKET_SIZE:
#                         break

#                     packet = self.buffers[port][header_index:header_index + PACKET_SIZE]
#                     self.buffers[port] = self.buffers[port][header_index + PACKET_SIZE:]

#                     anchor_hex = packet[17:19].hex()
#                     anchor_id = int(anchor_hex, 16)
#                     distance_cm = int.from_bytes(packet[25:29], byteorder='little')

#                     if anchor_id in ANCHOR_ID_MAP:
#                         uwb_index = ANCHOR_ID_MAP[anchor_id]
#                         self.latest_distances[uwb_index] = distance_cm
#                         self.last_update_time[uwb_index] = current_time
#                         self.get_logger().info(
#                             f"[{port}] Anchor {anchor_hex} → UWB{uwb_index} | 距離: {distance_cm} cm"
#                         )
#                     else:
#                         self.get_logger().warn(f"❓ 未知 Anchor ID: {anchor_hex}")

#             except Exception as e:
#                 self.get_logger().error(f"[{port}] 發生錯誤: {e}")

#         # 更新過期的資料為 None
#         uwb_list = []
#         for i in range(4):
#             if current_time - self.last_update_time[i] > TIMEOUT_SEC:
#                 uwb_list.append(None)
#             else:
#                 uwb_list.append(self.latest_distances[i])

#         # 印出清單
#         self.get_logger().info(f"📊 同步 UWB 距離: {uwb_list}")

#         # 發布 ROS topic（None 會用 NaN 代替）
#         msg = Float32MultiArray()
#         msg.data = [float(d) if d is not None else float('nan') for d in uwb_list]
#         self.publisher_.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = UWBSerialSyncNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

PACKET_SIZE = 64
HEADER = b'\x67\x69\x70\x73'
BAUDRATE = 57600
FPS = 30
TIMEOUT_SEC = 0.1  # 資料超過 0.1 秒未更新就視為無效

ANCHOR_ID_MAP = {
    0x1204: 0,
    0x1304: 1,
    0x1404: 2,
    0x1504: 3,
}

PORTS = [
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyUSB2",
    "/dev/ttyUSB3",
]


class UWB(Node):
    def __init__(self):
        super().__init__('UWB')

        self.serial_ports = {}
        self.buffers = {}

        # 儲存最新距離 & 對應更新時間
        self.latest_distances = [None] * 4
        self.last_update_time = [0.0] * 4

        # ✅ 建立 publisher，發佈 uwb_list
        self.publisher_ = self.create_publisher(Float32MultiArray, '/uwb/raw_data', 10)

        # 初始化 serial ports
        for port in PORTS:
            try:
                ser = serial.Serial(port, BAUDRATE, timeout=0.01)
                self.serial_ports[port] = ser
                self.buffers[port] = bytearray()
                self.get_logger().info(f"✅ 已連接 {port}")
            except serial.SerialException as e:
                self.get_logger().error(f"❌ 無法開啟 {port}: {e}")

        # 設定 Timer：每 1/30 秒執行一次
        self.create_timer(1.0 / FPS, self.timer_callback)

    def timer_callback(self):
        current_time = time.time()

        # 讀取所有 serial 資料
        for port, ser in self.serial_ports.items():
            try:
                self.buffers[port] += ser.read(ser.in_waiting)

                while len(self.buffers[port]) >= PACKET_SIZE:
                    header_index = self.buffers[port].find(HEADER)

                    if header_index == -1:
                        self.buffers[port] = self.buffers[port][-32:]
                        break

                    if len(self.buffers[port]) - header_index < PACKET_SIZE:
                        break

                    packet = self.buffers[port][header_index:header_index + PACKET_SIZE]
                    self.buffers[port] = self.buffers[port][header_index + PACKET_SIZE:]

                    anchor_hex = packet[17:19].hex()
                    anchor_id = int(anchor_hex, 16)
                    distance_cm = int.from_bytes(packet[25:29], byteorder='little')

                    if anchor_id in ANCHOR_ID_MAP:
                        uwb_index = ANCHOR_ID_MAP[anchor_id]
                        self.latest_distances[uwb_index] = distance_cm
                        self.last_update_time[uwb_index] = current_time
                        self.get_logger().info(
                            f"[{port}] Anchor {anchor_hex} → UWB{uwb_index} | 距離: {distance_cm} cm"
                        )
                    else:
                        self.get_logger().warn(f"❓ 未知 Anchor ID: {anchor_hex}")

            except Exception as e:
                self.get_logger().error(f"[{port}] 發生錯誤: {e}")

        # 檢查 timeout，過期資料設為 None
        uwb_list = []
        for i in range(4):
            if current_time - self.last_update_time[i] > TIMEOUT_SEC:
                uwb_list.append(None)
            else:
                uwb_list.append(self.latest_distances[i])

        # ✅ 印出清單方便觀察
        self.get_logger().info(f"📊 UWB 距離列表: {uwb_list}")

        # ✅ 發佈資料（將 None 轉換成 NaN）
        msg = Float32MultiArray()
        msg.data = [float(d) if d is not None else float('nan') for d in uwb_list]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UWB()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
