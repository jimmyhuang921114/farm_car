#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, windup_limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_limit = windup_limit

        self._prev_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Clear the previous error and integral term (use when a new target arrives)."""
        self._prev_error = 0.0
        self._integral = 0.0

    def update(self, error: float, dt: float) -> float:
        """
        Compute the PID output given the current error and time step dt.
        If dt <= 0, returns 0 to avoid division by zero or unstable spikes.
        """
        if dt <= 0.0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        if self.windup_limit is not None:
            self._integral = max(min(self._integral, self.windup_limit), -self.windup_limit)
        i_term = self.ki * self._integral

        # Derivative term
        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative

        # Save current error for next derivative calculation
        self._prev_error = error

        return p_term + i_term + d_term


class DiffDrivePIDNode(Node):
    def __init__(self):
        super().__init__('diff_drive_pid_relative')

        # === Declare and read parameters ===
        # Linear velocity PID
        self.declare_parameter('kp_lin', 1.0)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.1)
        self.declare_parameter('windup_lin', 0.5)

        # Angular velocity PID
        self.declare_parameter('kp_ang', 0.2)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.1)
        self.declare_parameter('windup_ang', 1.0)

        # Maximum speed limits
        self.declare_parameter('max_lin', 0.15)   # m/s
        self.declare_parameter('max_ang', 0.6)   # rad/s

        # Read PID constants and speed limits
        kp_lin = self.get_parameter('kp_lin').get_parameter_value().double_value
        ki_lin = self.get_parameter('ki_lin').get_parameter_value().double_value
        kd_lin = self.get_parameter('kd_lin').get_parameter_value().double_value
        windup_lin = self.get_parameter('windup_lin').get_parameter_value().double_value

        kp_ang = self.get_parameter('kp_ang').get_parameter_value().double_value
        ki_ang = self.get_parameter('ki_ang').get_parameter_value().double_value
        kd_ang = self.get_parameter('kd_ang').get_parameter_value().double_value
        windup_ang = self.get_parameter('windup_ang').get_parameter_value().double_value

        self.max_lin = self.get_parameter('max_lin').get_parameter_value().double_value
        self.max_ang = self.get_parameter('max_ang').get_parameter_value().double_value

        # Angle threshold: if |yaw_error| > 60°, perform in-place rotation
        self.angle_threshold = math.pi / 2.5  # 60 degrees

        # Initialize two PID controllers
        self.pid_lin = PIDController(kp_lin, ki_lin, kd_lin, windup_lin)
        self.pid_ang = PIDController(kp_ang, ki_ang, kd_ang, windup_ang)

        # Target relative coordinates (x, y). When None, no target is active.
        self.target_x = None
        self.target_y = None

        # Use this to compute delta time for PID
        self._last_time = self.get_clock().now().nanoseconds * 1e-9

        # === Subscribers and Publishers ===
        # Subscribe to /target (relative coordinates as geometry_msgs/Point)
        self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            1
        )
        # Publisher for /motor/main
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        # Timer at 30 Hz for control loop
        timer_period = 1.0 / 30.0  # seconds
        self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('DiffDrive PID controller (relative mode) started.')

    def target_callback(self, msg: Point):
        """
        Callback when a new relative target arrives.
        Reset the PID controllers to avoid leftover integral/derivative terms.
        """
        self.target_x = msg.x
        self.target_y = msg.y
        self.pid_lin.reset()
        self.pid_ang.reset()
        # self.get_logger().info(f'Received new target (relative): x={self.target_x:.3f}, y={self.target_y:.3f}')

    def control_loop(self):
        """
        Runs at 30 Hz. Computes and publishes /cmd_vel according to:
         1. If no target: publish zero velocity and return.
         2. Compute distance and yaw_error to the target (robot assumed at (0,0), heading along +X).
         3. If distance <= 1.0 m: publish zero velocity (stop) and return.
         4. If |yaw_error| > 60°: set linear=0, use angular PID for in-place rotation.
         5. Otherwise: use linear PID on (distance–1.0) and angular PID on yaw_error.
         6. Clamp the resulting commands to max_lin/max_ang, publish Twist, and log them.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self._last_time
        self._last_time = now

        # 1. If no target, stop
        if self.target_x is None or self.target_y is None:
            zero_twist = Twist()
            zero_twist.linear.x = 0.0
            zero_twist.angular.z = 0.0
            self.cmd_pub.publish(zero_twist)
            self.get_logger().info('No target: Linear Vel=0.000, Angular Vel=0.000')
            return

        # 2. Compute distance and yaw_error
        dx = self.target_x
        dy = self.target_y
        distance = math.hypot(dx, dy)
        yaw_error = math.atan2(dy, dx)  # robot heading is along +X

        # 3. If within 1.0 m of the target, stop
        if distance <= 1.0:
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.cmd_pub.publish(stop_twist)
            self.get_logger().info(f'Within 1m: distance={distance:.3f}. Linear Vel=0.000, Angular Vel=0.000')
            return

        # 4. If yaw error > 60°, rotate in place
        if abs(yaw_error) > self.angle_threshold:
            ang_cmd = self.pid_ang.update(yaw_error, dt)
            ang_cmd = max(min(ang_cmd, self.max_ang), -self.max_ang)

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = ang_cmd
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'Rotating in place: Linear Vel=0.000, Angular Vel={ang_cmd:.3f}')
            return

        # 5. Normal mode: move forward while adjusting heading
        dist_error = distance - 1.0
        lin_cmd = self.pid_lin.update(dist_error, dt)
        ang_cmd = self.pid_ang.update(yaw_error, dt)

        # 6. Clamp speeds
        lin_cmd = max(min(lin_cmd, self.max_lin), -self.max_lin)
        ang_cmd = max(min(ang_cmd, self.max_ang), -self.max_ang)

        twist = Twist()
        twist.linear.x = lin_cmd
        twist.angular.z = ang_cmd
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Control: distance={distance:.3f}, dist_error={dist_error:.3f}, '
                               f'yaw_error={yaw_error:.3f} -> Linear Vel={lin_cmd:.3f}, Angular Vel={ang_cmd:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
