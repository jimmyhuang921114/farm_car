# # #!/usr/bin/env python3
# # import time
# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import Twist
# # from mavros_msgs.srv import CommandLong, CommandBool
# # from mavros_msgs.msg import OverrideRCIn

# # def clamp(x, a, b): return a if x < a else b if x > b else x
# # def unit_to_us(x):  # -1..+1 → 1000..2000 us
# #     x = clamp(x, -1.0, 1.0)
# #     return int(1500 + 500*x)

# # # （若真的要直打 SERVO 腳才會用到；Brushed 模式建議用 RC Override）
# # SERVO_LEFT  = 9
# # SERVO_RIGHT = 11

# # MAV_CMD_DO_SET_SERVO         = 183
# # MAV_CMD_COMPONENT_ARM_DISARM = 400

# # class CmdVelToPWM(Node):
# #     def __init__(self):
# #         super().__init__('cmdvel_to_pwm_mavros')

# #         # 參數
# #         self.declare_parameter('mode', 'brushed')        # 'brushed' | 'override' | 'servo'
# #         self.declare_parameter('servo_left',  SERVO_LEFT)
# #         self.declare_parameter('servo_right', SERVO_RIGHT)
# #         self.declare_parameter('rc_steer_idx', 0)        # RC CH1 index=0（轉向）
# #         self.declare_parameter('rc_thrtl_idx', 2)        # RC CH3 index=2（油門）
# #         self.declare_parameter('turn_gain', 1.0)
# #         self.declare_parameter('max_linear', 1.0)
# #         self.declare_parameter('max_angular', 1.0)
# #         self.declare_parameter('timeout', 0.5)
# #         self.declare_parameter('input_topic', '/cmd_vel')
# #         self.declare_parameter('auto_arm_on_cmd', True)
# #         self.declare_parameter('auto_disarm_on_timeout', False)
# #         self.declare_parameter('log_hz', 5.0)            # 每秒印幾次除錯

# #         self.mode         = str(self.get_parameter('mode').value).lower()
# #         self.servo_left   = int(self.get_parameter('servo_left').value)
# #         self.servo_right  = int(self.get_parameter('servo_right').value)
# #         self.rc_steer_idx = int(self.get_parameter('rc_steer_idx').value)
# #         self.rc_thrtl_idx = int(self.get_parameter('rc_thrtl_idx').value)
# #         self.turn_gain    = float(self.get_parameter('turn_gain').value)
# #         self.max_linear   = float(self.get_parameter('max_linear').value)
# #         self.max_angular  = float(self.get_parameter('max_angular').value)
# #         self.timeout_s    = float(self.get_parameter('timeout').value)
# #         self.log_hz       = float(self.get_parameter('log_hz').value)
# #         input_topic       = str(self.get_parameter('input_topic').value)
# #         self.auto_arm_on_cmd        = bool(self.get_parameter('auto_arm_on_cmd').value)
# #         self.auto_disarm_on_timeout = bool(self.get_parameter('auto_disarm_on_timeout').value)

# #         # I/O
# #         self.sub = self.create_subscription(Twist, input_topic, self.cb_cmd, 10)
# #         self.cli_cmd_long = self.create_client(CommandLong, '/mavros/cmd/command')
# #         self.cli_arm_bool = self.create_client(CommandBool, '/mavros/cmd/arming')
# #         self.pub_override = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

# #         # 狀態
# #         self.last_cmd_time = 0.0
# #         self.last_linear   = 0.0
# #         self.last_angular  = 0.0
# #         self.armed         = False
# #         self.first_cmd_received = False
# #         self._last_log_t = 0.0

# #         self.timer = self.create_timer(1.0/30.0, self.loop)  # 30Hz
# #         self.get_logger().info(
# #             f'CmdVelToPWM ready, mode={self.mode}, input_topic={input_topic}, '
# #             f'servo L/R={self.servo_left}/{self.servo_right}, rc steer/thrtl idx={self.rc_steer_idx}/{self.rc_thrtl_idx}'
# #         )

# #     def cb_cmd(self, msg: Twist):
# #         now = time.time()
# #         self.last_cmd_time = now
# #         self.last_linear   = float(msg.linear.x)
# #         self.last_angular  = float(msg.angular.z)

# #         if self.auto_arm_on_cmd and not self.first_cmd_received:
# #             self.first_cmd_received = True
# #             self.get_logger().info('First /cmd_vel received → auto ARM...')
# #             self._arm(True)

# #     # 混控（差速）：把線速度/角速度轉成左右輪 -1..+1
# #     def mix_lr(self, lin, ang):
# #         lin = clamp(lin / self.max_linear, -1.0, 1.0)
# #         ang = clamp(ang / self.max_angular, -1.0, 1.0) * self.turn_gain
# #         left  = clamp(lin - ang, -1.0, 1.0)
# #         right = clamp(lin + ang, -1.0, 1.0)
# #         return left, right

# #     def loop(self):
# #         now = time.time()
# #         timed_out = (now - self.last_cmd_time) > self.timeout_s

# #         # 預設停止
# #         steer_u = thrtl_u = 1500
# #         steer_duty = thrtl_duty = 0.0

# #         if not timed_out:
# #             # 對於 brushed/override，我們用「方向/油門」而不是左右輪 μs
# #             steer = clamp(self.last_angular / self.max_angular, -1.0, 1.0) * self.turn_gain
# #             thrtl = clamp(self.last_linear  / self.max_linear,  -1.0, 1.0)
# #             steer_u = unit_to_us(steer)
# #             thrtl_u = unit_to_us(thrtl)
# #             steer_duty  = steer
# #             thrtl_duty  = thrtl
# #         else:
# #             if self.auto_disarm_on_timeout and self.armed:
# #                 self.get_logger().warn('cmd timeout → auto DISARM')
# #                 self._arm(False)

# #         if self.mode in ('brushed', 'override'):
# #             # RC Override：CH1=轉向、CH3=油門
# #             msg = OverrideRCIn()
# #             msg.channels = [65535] * 18
# #             msg.channels[self.rc_steer_idx] = int(steer_u)
# #             msg.channels[self.rc_thrtl_idx] = int(thrtl_u)
# #             self.pub_override.publish(msg)

# #             # 除錯列印（顯示 duty 與等效 μs）
# #             if self.log_hz > 0 and (now - self._last_log_t) >= (1.0 / self.log_hz):
# #                 self._last_log_t = now
# #                 self.get_logger().info(
# #                     f'[Brushed] steer={steer_duty:+.2f} ({(steer_duty*100):+.0f}%) -> {steer_u}us,  '
# #                     f'throttle={thrtl_duty:+.2f} ({(thrtl_duty*100):+.0f}%) -> {thrtl_u}us'
# #                 )

# #         elif self.mode == 'servo':
# #             # 這條路徑只適用真的要直接打 SERVO 腳位（建議用 PWM 組如 13/15）
# #             l, r = self.mix_lr(self.last_linear, self.last_angular) if not timed_out else (0.0, 0.0)
# #             left_u, right_u = unit_to_us(l), unit_to_us(r)
# #             self._do_set_servo(self.servo_left,  left_u)
# #             self._do_set_servo(self.servo_right, right_u)
# #             if self.log_hz > 0 and (now - self._last_log_t) >= (1.0 / self.log_hz):
# #                 self._last_log_t = now
# #                 self.get_logger().info(f'[Servo] L={l:+.2f}->{left_u}us, R={r:+.2f}->{right_u}us')

# #     # --- MAVROS helpers ---
# #     def _do_set_servo(self, servo_idx: int, pwm_us: int):
# #         if not self.cli_cmd_long.service_is_ready():
# #             return
# #         req = CommandLong.Request()
# #         req.broadcast = False
# #         req.command = MAV_CMD_DO_SET_SERVO
# #         req.confirmation = 0
# #         req.param1 = float(servo_idx)
# #         req.param2 = float(pwm_us)
# #         self.cli_cmd_long.call_async(req)

# #     def _arm(self, arm: bool) -> bool:
# #         if self.cli_arm_bool.service_is_ready():
# #             req = CommandBool.Request()
# #             req.value = arm
# #             self.cli_arm_bool.call_async(req)
# #             self.armed = arm
# #             self.get_logger().info(f'ARM via CommandBool → {arm}')
# #             return True

# #         if self.cli_cmd_long.service_is_ready():
# #             req = CommandLong.Request()
# #             req.broadcast = False
# #             req.command = MAV_CMD_COMPONENT_ARM_DISARM
# #             req.confirmation = 0
# #             req.param1 = 1.0 if arm else 0.0
# #             self.cli_cmd_long.call_async(req)
# #             self.armed = arm
# #             self.get_logger().info(f'ARM via CommandLong → {arm}')
# #             return True

# #         self.get_logger().warn('Arming service not ready yet.')
# #         return False

# # def main():
# #     rclpy.init()
# #     node = CmdVelToPWM()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()



# #!/usr/bin/env python3
# import time
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from mavros_msgs.srv import CommandLong, CommandBool, SetMode
# from mavros_msgs.msg import OverrideRCIn, State

# def clamp(x, a, b): return a if x < a else b if x > b else x
# def unit_to_us(x):
#     x = clamp(x, -1.0, 1.0)
#     return int(1500 + 500*x)

# class CmdVelToBrushOverride(Node):
#     def __init__(self):
#         super().__init__('cmdvel_to_brush_override')

#         # 可調參數
#         self.declare_parameter('rc_steer_idx', 0)   # CH1 index=0
#         self.declare_parameter('rc_thrtl_idx', 2)   # CH3 index=2
#         self.declare_parameter('max_linear',  1.0)
#         self.declare_parameter('max_angular', 1.0)
#         self.declare_parameter('turn_gain',   1.0)
#         self.declare_parameter('timeout',     0.5)  # s
#         self.declare_parameter('input_topic', '/cmd_vel')
#         self.declare_parameter('auto_arm_on_cmd', True)
#         self.declare_parameter('set_mode_on_cmd',  True)   # 第一次指令改模式
#         self.declare_parameter('mode_name', 'MANUAL')       # 或 'ACRO'
#         self.declare_parameter('log_hz', 5.0)

#         self.rc_steer_idx = int(self.get_parameter('rc_steer_idx').value)
#         self.rc_thrtl_idx = int(self.get_parameter('rc_thrtl_idx').value)
#         self.max_linear   = float(self.get_parameter('max_linear').value)
#         self.max_angular  = float(self.get_parameter('max_angular').value)
#         self.turn_gain    = float(self.get_parameter('turn_gain').value)
#         self.timeout_s    = float(self.get_parameter('timeout').value)
#         self.log_hz       = float(self.get_parameter('log_hz').value)
#         self.input_topic  = str(self.get_parameter('input_topic').value)
#         self.auto_arm     = bool(self.get_parameter('auto_arm_on_cmd').value)
#         self.set_mode     = bool(self.get_parameter('set_mode_on_cmd').value)
#         self.mode_name    = str(self.get_parameter('mode_name').value).upper()

#         # IO
#         self.sub_cmd  = self.create_subscription(Twist, self.input_topic, self.cb_cmd, 10)
#         self.sub_state= self.create_subscription(State, '/mavros/state', self.cb_state, 10)
#         self.pub_override = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
#         self.cli_arm   = self.create_client(CommandBool, '/mavros/cmd/arming')
#         self.cli_mode  = self.create_client(SetMode, '/mavros/set_mode')
#         self.cli_cmdlg = self.create_client(CommandLong, '/mavros/cmd/command')  # 備用

#         # 狀態
#         self.last_cmd_time = 0.0
#         self.lin = 0.0
#         self.ang = 0.0
#         self.armed = False
#         self.current_mode = 'UNKNOWN'
#         self.first_cmd_sent = False
#         self._last_log_t = 0.0

#         # 30 Hz 送 override
#         self.timer = self.create_timer(1.0/30.0, self.loop)

#         self.get_logger().info(
#             f'Brush Override ready: steer_idx={self.rc_steer_idx+1}, thrtl_idx={self.rc_thrtl_idx+1}, '
#             f'mode_on_cmd={self.mode_name}, input_topic={self.input_topic}'
#         )

#     # ========== Callbacks ==========
#     def cb_state(self, st: State):
#         self.armed = bool(st.armed)
#         self.current_mode = st.mode

#     def cb_cmd(self, msg: Twist):
#         now = time.time()
#         self.last_cmd_time = now
#         self.lin = float(msg.linear.x)
#         self.ang = float(msg.angular.z)

#         if not self.first_cmd_sent:
#             self.first_cmd_sent = True
#             if self.set_mode:
#                 self._set_mode(self.mode_name)
#             if self.auto_arm:
#                 self._arm(True)

#     # ========== Main loop ==========
#     def loop(self):
#         now = time.time()
#         timed_out = (now - self.last_cmd_time) > self.timeout_s

#         # 預設：全部通道 1500（中位），只有 CH1/CH3 會被我們改
#         msg = OverrideRCIn()
#         msg.channels = [1500] * 18

#         if not timed_out:
#             steer = clamp(self.ang / max(1e-6, self.max_angular), -1.0, 1.0) * self.turn_gain
#             thrtl = clamp(self.lin / max(1e-6, self.max_linear),  -1.0, 1.0)
#             steer_u = unit_to_us(steer)
#             thrtl_u = unit_to_us(thrtl)
#         else:
#             # 超時回中位
#             steer = thrtl = 0.0
#             steer_u = thrtl_u = 1500

#         # 只覆蓋 CH1 / CH3
#         msg.channels[self.rc_steer_idx] = int(steer_u)
#         msg.channels[self.rc_thrtl_idx] = int(thrtl_u)

#         # 發布覆蓋（**持續**）
#         self.pub_override.publish(msg)

#         # Log
#         if self.log_hz > 0 and (now - self._last_log_t) >= (1.0 / self.log_hz):
#             self._last_log_t = now
#             self.get_logger().info(
#                 f'[BrushOverride] mode={self.current_mode}  armed={self.armed}  timeout={timed_out} | '
#                 f'steer={steer:+.2f}->{steer_u}us,  throttle={thrtl:+.2f}->{thrtl_u}us'
#             )

#     # ========== MAVROS helpers ==========
#     def _arm(self, val: bool):
#         if self.cli_arm.service_is_ready():
#             req = CommandBool.Request(); req.value = val
#             self.cli_arm.call_async(req)
#             self.get_logger().info(f'ARM request: {val}')
#         else:
#             self.get_logger().warn('Arming service not ready.')

#     def _set_mode(self, mode_name: str):
#         if self.cli_mode.service_is_ready():
#             req = SetMode.Request()
#             req.base_mode = 0
#             req.custom_mode = mode_name
#             self.cli_mode.call_async(req)
#             self.get_logger().info(f'SET_MODE request: {mode_name}')
#         else:
#             self.get_logger().warn('SetMode service not ready.')

# def main():
#     rclpy.init()
#     node = CmdVelToBrushOverride()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode

def clamp(x, a, b): 
    return a if x < a else b if x > b else x

def unit_to_us(x):  # -1..+1 → 1000..2000 us（舵機脈寬等效值，Rover 會再轉成馬達輸出）
    x = clamp(x, -1.0, 1.0)
    return int(1500 + 500 * x)

class FixedBrushOverride(Node):
    def __init__(self):
        super().__init__('fixed_brush_override')

        # ===== 參數 =====
        self.declare_parameter('rc_steer_idx', 0)        # CH1 index=0（轉向）
        self.declare_parameter('rc_thrtl_idx', 2)        # CH3 index=2（油門）
        self.declare_parameter('steer', 0.0)             # -1..+1（左負右正）
        self.declare_parameter('throttle', 0.3)          # -1..+1（前正後負）
        self.declare_parameter('turn_gain', 1.0)         # 方向增益
        self.declare_parameter('duration', 0.0)          # 秒；0=無限直到 Ctrl-C
        self.declare_parameter('neutral_on_exit', True)  # 結束時送 1500/1500
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('set_mode', True)
        self.declare_parameter('mode_name', 'GUIDED')    # 你說只剩 GUIDED → 用 GUIDED；若可用無GPS建議 GUIDED_NOGPS
        self.declare_parameter('log_hz', 2.0)

        self.rc_steer_idx    = int(self.get_parameter('rc_steer_idx').value)
        self.rc_thrtl_idx    = int(self.get_parameter('rc_thrtl_idx').value)
        self.steer_cmd       = float(self.get_parameter('steer').value)
        self.thrtl_cmd       = float(self.get_parameter('throttle').value)
        self.turn_gain       = float(self.get_parameter('turn_gain').value)
        self.duration        = float(self.get_parameter('duration').value)
        self.neutral_on_exit = bool(self.get_parameter('neutral_on_exit').value)
        self.auto_arm        = bool(self.get_parameter('auto_arm').value)
        self.set_mode_flag   = bool(self.get_parameter('set_mode').value)
        self.mode_name       = str(self.get_parameter('mode_name').value).upper()
        self.log_hz          = float(self.get_parameter('log_hz').value)

        # ===== IO =====
        self.pub_override = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.sub_state    = self.create_subscription(State, '/mavros/state', self.cb_state, 10)
        self.cli_arm      = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode     = self.create_client(SetMode, '/mavros/set_mode')

        # 狀態
        self.armed = False
        self.current_mode = 'UNKNOWN'
        self.t0 = time.time()
        self._last_log_t = 0.0

        # 啟動時切模式/ARM（可選）
        if self.set_mode_flag:
            self._set_mode(self.mode_name)
        if self.auto_arm:
            self._arm(True)

        # 30 Hz 定時送指令
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        self.get_logger().info(
            f'FixedBrushOverride ready | steer={self.steer_cmd:+.2f}, throttle={self.thrtl_cmd:+.2f}, '
            f'duration={self.duration:.1f}s, mode_on_start={self.mode_name}, auto_arm={self.auto_arm}'
        )

    # ===== Callbacks =====
    def cb_state(self, st: State):
        self.armed = bool(st.armed)
        self.current_mode = st.mode if st.mode else 'UNKNOWN'

    # ===== Main loop =====
    def loop(self):
        now = time.time()
        elapsed = now - self.t0

        # 若設定 duration 且已到時 → 停止/解鎖並退出
        if self.duration > 0.0 and elapsed >= self.duration:
            self.get_logger().info('Duration reached.')
            if self.neutral_on_exit:
                self._send_override(1500, 1500)
                self.get_logger().info('Output → neutral (1500/1500)')
            if self.auto_arm and self.armed:
                self._arm(False)
            # 正常結束
            self.get_logger().info('Shutting down node...')
            rclpy.shutdown()
            return

        # 計算固定輸出（-1..+1 → μs）
        steer = clamp(self.steer_cmd * self.turn_gain, -1.0, 1.0)
        thrtl = clamp(self.thrtl_cmd, -1.0, 1.0)
        steer_u = unit_to_us(steer)
        thrtl_u = unit_to_us(thrtl)

        # 送出
        self._send_override(steer_u, thrtl_u)

        # 簡短 log
        if self.log_hz > 0 and (now - self._last_log_t) >= (1.0 / self.log_hz):
            self._last_log_t = now
            self.get_logger().info(
                f'mode={self.current_mode} armed={self.armed} | steer={steer:+.2f}->{steer_u}us, '
                f'throttle={thrtl:+.2f}->{thrtl_u}us'
            )

    # ===== Helpers =====
    def _send_override(self, steer_u: int, thrtl_u: int):
        msg = OverrideRCIn()
        # 65535 = 不覆蓋該通道；避免把其它通道鎖在 1500
        msg.channels = [65535] * 18
        msg.channels[self.rc_steer_idx] = int(steer_u)
        msg.channels[self.rc_thrtl_idx] = int(thrtl_u)
        self.pub_override.publish(msg)

    def _arm(self, val: bool):
        if not self.cli_arm.service_is_ready():
            self.cli_arm.wait_for_service(timeout_sec=2.0)
        if self.cli_arm.service_is_ready():
            req = CommandBool.Request()
            req.value = val
            self.cli_arm.call_async(req)
            self.get_logger().info(f'ARM request: {val}')
        else:
            self.get_logger().warn('Arming service not ready.')

    def _set_mode(self, mode_name: str):
        if not self.cli_mode.service_is_ready():
            self.cli_mode.wait_for_service(timeout_sec=2.0)
        if self.cli_mode.service_is_ready():
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = mode_name
            self.cli_mode.call_async(req)
            self.get_logger().info(f'SET_MODE request: {mode_name}')
        else:
            self.get_logger().warn('SetMode service not ready.')

def main():
    rclpy.init()
    node = FixedBrushOverride()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
