import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist, PoseStamped, Point
from nav_msgs.msg import Path
from math import atan2, sqrt, cos, sin, pi

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # 訂閱 UWB 目標位置 (PointStamped) - 注意: 接收到的座標單位為公分，需轉換為公尺
        self.subscription = self.create_subscription(
            PointStamped, '/uwb_filtered_position_nls', self.goal_callback, 10)
        
        # 發布機器人速度指令
        self.cmd_vel_publisher = self.create_publisher(Twist, '/motor/main', 10)

        # 發布 RViz 顯示的路徑
        self.path_publisher = self.create_publisher(Path, '/dwa_path', 10)

        # 機器人狀態
        self.current_position = PointStamped()
        self.current_theta = 0.0  # 此為車體內部角度，若為0則全域朝向為 Y 軸
        self.goal_position = None

        # 速度限制
        self.max_v = 0.3  # 最大線速度 (m/s)
        self.max_w = 1.2  # 最大角速度 (rad/s)
        self.acc_v = 0.2  # 最大線加速度 (m/s²)
        self.acc_w = 1.0  # 最大角加速度 (rad/s²)

        # 其他參數
        self.dt = 0.1  # 時間間隔 (s)
        self.goal_tolerance = 0.1  # 目標誤差範圍 (m)

        # 計時器，每 0.1s 計算一次 DWA
        self.timer = self.create_timer(self.dt, self.dwa_control)

    def goal_callback(self, msg):
        """ 目標位置回調函數 (接收 PointStamped)
            將原本以公分單位的座標轉換為公尺
        """
        converted_point = Point()
        converted_point.x = msg.point.x / 100.0
        converted_point.y = msg.point.y / 100.0
        converted_point.z = msg.point.z / 100.0  # 如有需要，轉換 z 軸資訊
        self.goal_position = converted_point
        self.get_logger().info(
            f'Received Goal (converted to meters): ({converted_point.x}, {converted_point.y})')

    def dwa_control(self):
        """ DWA 計算最佳速度並發布命令 """
        if self.goal_position is None:
            return  # 若無目標則不執行
        
        # 計算目標相對機器人的角度與距離（全域座標系）
        dx = self.goal_position.x - self.current_position.point.x
        dy = self.goal_position.y - self.current_position.point.y
        distance = sqrt(dx**2 + dy**2)
        target_angle = atan2(dy, dx)  # 全域角度

        # 若已到達目標，則停止機器人
        if distance < self.goal_tolerance:
            self.publish_cmd(0.0, 0.0)
            return

        # 計算可行的速度窗口
        v_window = np.linspace(0, self.max_v, 5)
        w_window = np.linspace(-self.max_w, self.max_w, 5)

        best_v, best_w = 0.0, 0.0
        best_score = -float('inf')
        best_path = []

        # 探索所有速度組合
        for v in v_window:
            for w in w_window:
                # 預測軌跡
                path = self.simulate_trajectory(v, w)
                score = self.evaluate_trajectory(path, target_angle, distance)

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
                    best_path = path
        
        # 發布最佳速度
        self.publish_cmd(best_v, best_w)
        
        # 發布路徑到 RViz
        self.publish_path(best_path)

    def simulate_trajectory(self, v, w):
        """ 模擬機器人未來的軌跡
            注意: 將車體內部角度 theta 轉換為全域角度 (theta + π/2)
                  使得當 theta=0 時，車子前進方向為全域 Y 軸
        """
        path = []
        x = self.current_position.point.x
        y = self.current_position.point.y
        theta = self.current_theta  # 車體內部角度

        for _ in range(10):  # 模擬 1s (10 次 0.1s)
            global_theta = theta + pi/2  # 全域角度
            x += v * cos(global_theta) * self.dt
            y += v * sin(global_theta) * self.dt
            theta += w * self.dt
            path.append((x, y, theta))
        
        return path

    def evaluate_trajectory(self, path, target_angle, distance):
        """ 評估軌跡得分 """
        if not path:
            return -float('inf')
        
        last_x, last_y, last_theta = path[-1]
        # 轉換模擬軌跡中最後的車體角度為全域角度
        global_last_theta = last_theta + pi/2

        # 1. 目標方向得分 (愈接近目標方向，得分越高)
        heading_score = -abs(target_angle - global_last_theta)

        # 2. 速度得分 (鼓勵較快速度)
        speed_score = last_x  # 此處可根據需求調整評分方式

        # 3. 安全性 (避障: 可加入雷射感測判斷)
        obstacle_score = 1.0  # 假設無障礙物，分數固定為 1

        # 總得分
        return heading_score * 1.0 + speed_score * 0.3 + obstacle_score * 1.0

    def publish_cmd(self, v, w):
        """ 發布機器人速度指令 """
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published cmd: v={v:.2f}, w={w:.2f}')

    def publish_path(self, path):
        """ 發布 DWA 預測的軌跡至 RViz """
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for x, y, theta in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
