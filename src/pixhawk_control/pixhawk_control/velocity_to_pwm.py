#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandLong, CommandBool
from mavros_msgs.msg import OverrideRCIn

def clamp(x, a, b): return a if x < a else b if x > b else x
def unit_to_us(x):  # -1..+1 → 1000..2000 us
    x = clamp(x, -1.0, 1.0)
    return int(1500 + 500*x)

# （若真的要直打 SERVO 腳才會用到；Brushed 模式建議用 RC Override）
SERVO_LEFT  = 9
SERVO_RIGHT = 11

MAV_CMD_DO_SET_SERVO         = 183
MAV_CMD_COMPONENT_ARM_DISARM = 400

class CmdVelToPWM(Node):
    def __init__(self):
        super().__init__('cmdvel_to_pwm_mavros')

        # 參數
        self.declare_parameter('mode', 'brushed')        # 'brushed' | 'override' | 'servo'
        self.declare_parameter('servo_left',  SERVO_LEFT)
        self.declare_parameter('servo_right', SERVO_RIGHT)
        self.declare_parameter('rc_steer_idx', 0)        # RC CH1 index=0（轉向）
        self.declare_parameter('rc_thrtl_idx', 2)        # RC CH3 index=2（油門）
        self.declare_parameter('turn_gain', 1.0)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('auto_arm_on_cmd', True)
        self.declare_parameter('auto_disarm_on_timeout', False)
        self.declare_parameter('log_hz', 5.0)            # 每秒印幾次除錯

        self.mode         = str(self.get_parameter('mode').value).lower()
        self.servo_left   = int(self.get_parameter('servo_left').value)
        self.servo_right  = int(self.get_parameter('servo_right').value)
        self.rc_steer_idx = int(self.get_parameter('rc_steer_idx').value)
        self.rc_thrtl_idx = int(self.get_parameter('rc_thrtl_idx').value)
        self.turn_gain    = float(self.get_parameter('turn_gain').value)
        self.max_linear   = float(self.get_parameter('max_linear').value)
        self.max_angular  = float(self.get_parameter('max_angular').value)
        self.timeout_s    = float(self.get_parameter('timeout').value)
        self.log_hz       = float(self.get_parameter('log_hz').value)
        input_topic       = str(self.get_parameter('input_topic').value)
        self.auto_arm_on_cmd        = bool(self.get_parameter('auto_arm_on_cmd').value)
        self.auto_disarm_on_timeout = bool(self.get_parameter('auto_disarm_on_timeout').value)

        # I/O
        self.sub = self.create_subscription(Twist, input_topic, self.cb_cmd, 10)
        self.cli_cmd_long = self.create_client(CommandLong, '/mavros/cmd/command')
        self.cli_arm_bool = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.pub_override = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # 狀態
        self.last_cmd_time = 0.0
        self.last_linear   = 0.0
        self.last_angular  = 0.0
        self.armed         = False
        self.first_cmd_received = False
        self._last_log_t = 0.0

        self.timer = self.create_timer(1.0/30.0, self.loop)  # 30Hz
        self.get_logger().info(
            f'CmdVelToPWM ready, mode={self.mode}, input_topic={input_topic}, '
            f'servo L/R={self.servo_left}/{self.servo_right}, rc steer/thrtl idx={self.rc_steer_idx}/{self.rc_thrtl_idx}'
        )

    def cb_cmd(self, msg: Twist):
        now = time.time()
        self.last_cmd_time = now
        self.last_linear   = float(msg.linear.x)
        self.last_angular  = float(msg.angular.z)

        if self.auto_arm_on_cmd and not self.first_cmd_received:
            self.first_cmd_received = True
            self.get_logger().info('First /cmd_vel received → auto ARM...')
            self._arm(True)

    # 混控（差速）：把線速度/角速度轉成左右輪 -1..+1
    def mix_lr(self, lin, ang):
        lin = clamp(lin / self.max_linear, -1.0, 1.0)
        ang = clamp(ang / self.max_angular, -1.0, 1.0) * self.turn_gain
        left  = clamp(lin - ang, -1.0, 1.0)
        right = clamp(lin + ang, -1.0, 1.0)
        return left, right

    def loop(self):
        now = time.time()
        timed_out = (now - self.last_cmd_time) > self.timeout_s

        # 預設停止
        steer_u = thrtl_u = 1500
        steer_duty = thrtl_duty = 0.0

        if not timed_out:
            # 對於 brushed/override，我們用「方向/油門」而不是左右輪 μs
            steer = clamp(self.last_angular / self.max_angular, -1.0, 1.0) * self.turn_gain
            thrtl = clamp(self.last_linear  / self.max_linear,  -1.0, 1.0)
            steer_u = unit_to_us(steer)
            thrtl_u = unit_to_us(thrtl)
            steer_duty  = steer
            thrtl_duty  = thrtl
        else:
            if self.auto_disarm_on_timeout and self.armed:
                self.get_logger().warn('cmd timeout → auto DISARM')
                self._arm(False)

        if self.mode in ('brushed', 'override'):
            # RC Override：CH1=轉向、CH3=油門
            msg = OverrideRCIn()
            msg.channels = [65535] * 18
            msg.channels[self.rc_steer_idx] = int(steer_u)
            msg.channels[self.rc_thrtl_idx] = int(thrtl_u)
            self.pub_override.publish(msg)

            # 除錯列印（顯示 duty 與等效 μs）
            if self.log_hz > 0 and (now - self._last_log_t) >= (1.0 / self.log_hz):
                self._last_log_t = now
                self.get_logger().info(
                    f'[Brushed] steer={steer_duty:+.2f} ({(steer_duty*100):+.0f}%) -> {steer_u}us,  '
                    f'throttle={thrtl_duty:+.2f} ({(thrtl_duty*100):+.0f}%) -> {thrtl_u}us'
                )

        elif self.mode == 'servo':
            # 這條路徑只適用真的要直接打 SERVO 腳位（建議用 PWM 組如 13/15）
            l, r = self.mix_lr(self.last_linear, self.last_angular) if not timed_out else (0.0, 0.0)
            left_u, right_u = unit_to_us(l), unit_to_us(r)
            self._do_set_servo(self.servo_left,  left_u)
            self._do_set_servo(self.servo_right, right_u)
            if self.log_hz > 0 and (now - self._last_log_t) >= (1.0 / self.log_hz):
                self._last_log_t = now
                self.get_logger().info(f'[Servo] L={l:+.2f}->{left_u}us, R={r:+.2f}->{right_u}us')

    # --- MAVROS helpers ---
    def _do_set_servo(self, servo_idx: int, pwm_us: int):
        if not self.cli_cmd_long.service_is_ready():
            return
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(servo_idx)
        req.param2 = float(pwm_us)
        self.cli_cmd_long.call_async(req)

    def _arm(self, arm: bool) -> bool:
        if self.cli_arm_bool.service_is_ready():
            req = CommandBool.Request()
            req.value = arm
            self.cli_arm_bool.call_async(req)
            self.armed = arm
            self.get_logger().info(f'ARM via CommandBool → {arm}')
            return True

        if self.cli_cmd_long.service_is_ready():
            req = CommandLong.Request()
            req.broadcast = False
            req.command = MAV_CMD_COMPONENT_ARM_DISARM
            req.confirmation = 0
            req.param1 = 1.0 if arm else 0.0
            self.cli_cmd_long.call_async(req)
            self.armed = arm
            self.get_logger().info(f'ARM via CommandLong → {arm}')
            return True

        self.get_logger().warn('Arming service not ready yet.')
        return False

def main():
    rclpy.init()
    node = CmdVelToPWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
