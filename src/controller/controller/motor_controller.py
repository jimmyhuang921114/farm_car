#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 節點：/cmd_vel -> Serial "L,R"（百分比 -100~100）
- 請在「使用者可調整設定」區塊直接修改各項參數
- 以單一檔案運作，無需其他設定檔或參數伺服器
- 已含背景序列埠讀取執行緒（印出 [RX]），主迴圈定期送出 [TX]
- 具備 watchdog（超時自動送 0,0）、平滑化、死區、方向翻轉等實用功能
"""

import math
import time
import threading
import sys
import traceback

import serial
from serial import SerialException

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# =========================
# 使用者可調整設定（全部在此）
# =========================

# ---- Serial 連線 ----
SERIAL_PORT = "/dev/ttyCH341USB0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.2          # 讀取逾時
SERIAL_STARTUP_DELAY_S = 2.0     # 給下游控制器(如 Arduino)重置時間

# ---- 機構/運動學設定 ----
WHEEL_RADIUS_M = 0.08           # 驅動輪半徑 (m)
TRACK_WIDTH_M  = 0.80           # 左右履帶中心距 (m)
MAX_RPM        = 90.0           # 100% 時的輪子 rpm
OUTPUT_LIMIT   = 100            # 百分比輸出限定 [-OUTPUT_LIMIT, OUTPUT_LIMIT]

# ---- 訊號處理 ----
CMD_RATE_HZ          = 20.0     # 送出序列命令頻率
WATCHDOG_TIMEOUT_S   = 0.5      # 若超過此秒數未收到 /cmd_vel，自動送 0,0
SMOOTHING_ALPHA      = 0.3      # 平滑化係數 (0=關閉, 0.1~0.5 建議)
DEADZONE_PERCENT     = 0        # 小於此絕對值的百分比直接視為 0（避免微抖動）

# ---- 方向/接線校正 ----
SWAP_LEFT_RIGHT = False         # True 則交換左右輸出
LEFT_SIGN       = +1            # 左輪極性：+1 或 -1
RIGHT_SIGN      = +1            # 右輪極性：+1 或 -1

# ---- 輸出格式 ----
INTEGER_OUTPUT = True           # True: 四捨五入為整數；False: 保留一位小數
DECIMAL_PLACES = 1              # 非整數輸出時的小數位數

# =========================
# 內部工具函式
# =========================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def radps_to_rpm(radps: float) -> float:
    return radps * 60.0 / (2.0 * math.pi)

def rpm_to_percent(rpm: float, max_rpm: float) -> float:
    if max_rpm <= 0:
        return 0.0
    return (rpm / max_rpm) * 100.0

def apply_deadzone(pct: float, deadzone: float) -> float:
    return 0.0 if abs(pct) < deadzone else pct

def maybe_round(x: float) -> float:
    if INTEGER_OUTPUT:
        return float(int(round(x)))
    return round(x, DECIMAL_PLACES)

# =========================
# ROS2 Node
# =========================

class TrackedBaseSerialBridge(Node):
    def __init__(self):
        super().__init__("tracked_base_serial_bridge")

        # 運動學預計算
        self.omega_max = MAX_RPM * 2.0 * math.pi / 60.0       # 輪子角速度上限 (rad/s) @ 100%
        self.v_wheel_max = self.omega_max * WHEEL_RADIUS_M    # 輪面線速度上限 (m/s)

        # 狀態
        self._last_cmd_time = time.time()
        self._target_left_pct = 0.0
        self._target_right_pct = 0.0
        self._out_left_pct = 0.0
        self._out_right_pct = 0.0
        self._stop_sent = False

        # 建立序列埠
        self._ser = None
        self._serial_lock = threading.Lock()
        self._open_serial()

        # 背景 reader
        # if self._ser:
        #     self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        #     self._rx_thread.start()
        # else:
        #     self._rx_thread = None

        # ROS2 訂閱與定時器
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._timer = self.create_timer(1.0 / CMD_RATE_HZ, self._on_timer)

        # 啟動訊息
        self.get_logger().info(
            f"Node started. Serial={SERIAL_PORT}@{SERIAL_BAUD} | R={WHEEL_RADIUS_M} m, W={TRACK_WIDTH_M} m, "
            f"max={MAX_RPM} rpm -> v_wheel_max={self.v_wheel_max:.3f} m/s"
        )

    # ---------- Serial ----------
    def _open_serial(self):
        try:
            self._ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT_S)
            time.sleep(SERIAL_STARTUP_DELAY_S)
            self.get_logger().info("Serial port opened.")
        except Exception as e:
            self._ser = None
            self.get_logger().error(f"Open serial failed: {e}\n{traceback.format_exc()}")

    def _reader_loop(self):
        assert self._ser is not None
        while self._ser and self._ser.is_open:
            try:
                line = self._ser.readline().decode(errors="replace").strip()
                if line:
                    print(f"[RX] {line}")
            except Exception:
                # 通常是 timeout 或離線；稍等再繼續
                time.sleep(0.01)

    def _write_line(self, s: str):
        if not self._ser or not self._ser.is_open:
            return
        try:
            with self._serial_lock:
                self._ser.write((s + "\n").encode())
            print(f"[TX] {s}")
        except SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ---------- ROS callbacks ----------
    def _on_cmd_vel(self, msg: Twist):
        """
        將 /cmd_vel (vx, wz) 轉為左右履帶百分比
        """
        self._last_cmd_time = time.time()
        self._stop_sent = False

        vx = msg.linear.x          # m/s
        wz = -msg.angular.z         # rad/s

        # 差速運動學：輪角速度(rad/s)
        # 右輪 v_r = (v + ω*W/2)/R, 左輪 v_l = (v - ω*W/2)/R
        w_r = (vx + wz * (TRACK_WIDTH_M / 2.0)) / WHEEL_RADIUS_M
        w_l = (vx - wz * (TRACK_WIDTH_M / 2.0)) / WHEEL_RADIUS_M

        # 轉 rpm
        rpm_r = radps_to_rpm(w_r)
        rpm_l = radps_to_rpm(w_l)

        # 轉百分比，並套用方向校正、交換
        pct_r = rpm_to_percent(rpm_r, MAX_RPM)
        pct_l = rpm_to_percent(rpm_l, MAX_RPM)

        # 飽和
        pct_r = clamp(pct_r, -OUTPUT_LIMIT, OUTPUT_LIMIT)
        pct_l = clamp(pct_l, -OUTPUT_LIMIT, OUTPUT_LIMIT)

        # 死區
        pct_r = apply_deadzone(pct_r, DEADZONE_PERCENT)
        pct_l = apply_deadzone(pct_l, DEADZONE_PERCENT)

        # 方向校正與交換
        out_l = LEFT_SIGN * pct_l
        out_r = RIGHT_SIGN * pct_r
        if SWAP_LEFT_RIGHT:
            out_l, out_r = out_r, out_l

        # 設為目標，定時器內做平滑與送出
        self._target_left_pct = out_l
        self._target_right_pct = out_r

    def _on_timer(self):
        # 連線掉了嘗試重連
        if not self._ser or not self._ser.is_open:
            self._open_serial()
            if self._ser and self._ser.is_open and self._rx_thread is None:
                self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
                self._rx_thread.start()

        now = time.time()
        dt_since_cmd = now - self._last_cmd_time

        # Watchdog：太久沒指令就送 0,0
        if dt_since_cmd > WATCHDOG_TIMEOUT_S:
            target_l = 0.0
            target_r = 0.0
            if not self._stop_sent:
                self._stop_sent = True
                self.get_logger().warn("Watchdog timeout -> send 0,0")
        else:
            target_l = self._target_left_pct
            target_r = self._target_right_pct

        # 平滑化 (低通)
        alpha = float(SMOOTHING_ALPHA)
        if 0.0 < alpha < 1.0:
            self._out_left_pct  = (1 - alpha) * self._out_left_pct  + alpha * target_l
            self._out_right_pct = (1 - alpha) * self._out_right_pct + alpha * target_r
        else:
            self._out_left_pct  = target_l
            self._out_right_pct = target_r

        # 最終量化
        out_l = maybe_round(self._out_left_pct)
        out_r = maybe_round(self._out_right_pct)

        # 組字串並送出
        # 範例格式：10,-10
        cmd = f"{int(out_l)},{int(-out_r)}" if INTEGER_OUTPUT else f"{out_l:.{DECIMAL_PLACES}f},{out_r:.{DECIMAL_PLACES}f}"
        self._write_line(cmd)

    # ---------- 收尾 ----------
    def close(self):
        try:
            # 送一次停車
            self._write_line("0,0")
        except Exception:
            pass
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass


def main():
    rclpy.init(args=None)
    node = TrackedBaseSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}\n{traceback.format_exc()}")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
