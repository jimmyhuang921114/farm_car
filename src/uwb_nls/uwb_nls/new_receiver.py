# import serial
# from datetime import datetime

# PORT = "/dev/ttyUSB0"
# BAUDRATE = 57600

# def sync_and_parse_packets(port, max_packets=10):
#     with serial.Serial(port, BAUDRATE, timeout=1) as ser:
#         buffer = bytearray()
#         count = 0

#         while count < max_packets:
#             buffer += ser.read(64)  # 不一定剛好對齊封包

#             # 確保至少有 64 bytes 可以抓
#             while len(buffer) >= 64:
#                 # 嘗試尋找 header 的位置
#                 header_index = buffer.find(b'\x67\x69\x70\x73')

#                 if header_index == -1:
#                     # 若沒有找到，就移除前 32 bytes（避免 buffer 無限增長）
#                     buffer = buffer[-32:]
#                     break

#                 # 如果 header 後面沒有 64 bytes，就等下一輪再讀更多資料
#                 if len(buffer) - header_index < 64:
#                     break

#                 # 取出合法封包
#                 packet = buffer[header_index:header_index + 64]

#                 # 移除已處理的部分
#                 buffer = buffer[header_index + 64:]

#                 # ✅ 處理封包
#                 print(f"\n✅ 第 {count + 1} 筆合法封包 | {datetime.now()}")
#                 tag_id = packet[9:11].hex()
#                 anchor_id = packet[17:19].hex()
#                 tof_bytes = packet[25:29]
#                 distance_cm = int.from_bytes(tof_bytes, byteorder='little')

#                 print(f"📏 距離: {distance_cm} cm")
#                 print(f"📡 Tag ID（發送端）: {tag_id}")
#                 print(f"📡 Anchor ID（接收端）: {anchor_id}")
#                 print(f"🧬 Raw distance bytes: {tof_bytes.hex()}")

#                  count += 1
# `````````````````

# if __name__ == "__main__":
#     sync_and_parse_packets(PORT)`````````````````

import serial
import threading
from datetime import datetime

PORTS = [f"/dev/ttyUSB{i}" for i in range(4)]
BAUDRATE = 57600
PACKET_SIZE = 64
HEADER = b'\x67\x69\x70\x73'

def sync_and_parse_packets(port, max_packets=None):
    try:
        with serial.Serial(port, BAUDRATE, timeout=1) as ser:
            buffer = bytearray()
            count = 0

            while True:
                buffer += ser.read(PACKET_SIZE)

                while len(buffer) >= PACKET_SIZE:
                    header_index = buffer.find(HEADER)

                    if header_index == -1:
                        buffer = buffer[-32:]  # 避免 buffer 過大
                        break

                    if len(buffer) - header_index < PACKET_SIZE:
                        break

                    packet = buffer[header_index:header_index + PACKET_SIZE]
                    buffer = buffer[header_index + PACKET_SIZE:]

                    tag_id = packet[9:11].hex()
                    anchor_id = packet[17:19].hex()
                    tof_bytes = packet[25:29]
                    distance_cm = int.from_bytes(tof_bytes, byteorder='little')

                    count += 1
                    print(f"\n✅ [Port: {port}] 第 {count} 筆合法封包 | {datetime.now()}")
                    print(f"📏 距離: {distance_cm} cm")
                    print(f"📡 Tag ID（發送端）: {tag_id}")
                    print(f"📡 Anchor ID（接收端）: {anchor_id}")
                    print(f"🧬 Raw distance bytes: {tof_bytes.hex()}")

                    if max_packets and count >= max_packets:
                        return
    except serial.SerialException as e:
        print(f"[ERROR] 無法打開 {port}: {e}")
    except Exception as e:
        print(f"[ERROR] {port} 發生錯誤: {e}")

def main():
    threads = []
    for port in PORTS:
        t = threading.Thread(target=sync_and_parse_packets, args=(port,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

if __name__ == "__main__":
    main()
