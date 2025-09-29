#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2

class VisualizationCV2Node(Node):
    def __init__(self):
        super().__init__('visualization_cv2_node')
        # 訂閱濾波後的位置
        self.sub = self.create_subscription(
            PointStamped,
            '/uwb/filter_position',
            self.callback,
            10
        )
        # 視覺化畫布設定
        self.canvas_size = 600  # 畫布正方形大小 (pixels)
        self.center = self.canvas_size // 2
        self.range = 1000.0  # 可視化範圍: +-1000 單位
        self.scale = self.canvas_size / (2 * self.range)  # 單位距離對應像素大小

        # 四個感應器在車身座標中的固定位置 (X: 前方向, Y: 右方向)
        self.sensor_positions = {
            0: np.array([   0.0, -26.0]),
            1: np.array([   0.0,  26.0]),
            2: np.array([-102.0, -26.0]),
            3: np.array([-102.0,  26.0]),
        }

        # 方框在世界座標中的頂點
        self.box_points = [
            (130, -200),
            (130, 320),
            (-140, 320),
            (-140, -200)
        ]

        # 建立 CV2 視窗
        cv2.namedWindow('UWB Visualization', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('UWB Visualization', self.canvas_size, self.canvas_size)

    def world_to_pixel(self, x, y):
        # world (X forward, Y right) -> pixel coordinate
        px = int(self.center - y * self.scale)
        py = int(self.center - x * self.scale)
        return px, py

    def callback(self, msg: PointStamped):
        # 取得點位
        x = msg.point.x  # 前後 (X)
        y = msg.point.y  # 左右 (Y)

        # 清空畫布 (白背景)
        canvas = np.ones((self.canvas_size, self.canvas_size, 3), dtype=np.uint8) * 255

        # 繪製感測器位置 (藍)
        for idx, pos in self.sensor_positions.items():
            sx, sy = pos
            px, py = self.world_to_pixel(sx, sy)
            cv2.circle(canvas, (px, py), 5, (255, 0, 0), -1)
            cv2.putText(canvas, f'S{idx}', (px+5, py-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        # 繪製估算位置 (紅)
        px_p, py_p = self.world_to_pixel(x, y)
        cv2.circle(canvas, (px_p, py_p), 7, (0, 0, 255), -1)
        cv2.putText(canvas, 'P', (px_p+5, py_p-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 繪製方框 (綠)
        pts = []
        for wx, wy in self.box_points:
            px_b, py_b = self.world_to_pixel(wx, wy)
            pts.append((px_b, py_b))
        # 閉合方框
        pts_np = np.array(pts, np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts_np], isClosed=True, color=(0, 255, 0), thickness=2)

        # 繪製座標軸 (黑)
        axis_len = int(self.range * self.scale * 0.05)  # 軸長佔可視範圍 5%
        # X 軸 (向上)
        cv2.arrowedLine(
            canvas,
            (self.center, self.center),
            (self.center, self.center - axis_len),
            (0, 0, 0), 2, tipLength=0.1
        )
        cv2.putText(
            canvas,
            'X',
            (self.center + 5, self.center - axis_len - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2
        )
        # Y 軸 (向右)
        cv2.arrowedLine(
            canvas,
            (self.center, self.center),
            (self.center - axis_len, self.center),
            (0, 0, 0), 2, tipLength=0.1
        )
        cv2.putText(
            canvas,
            'Y',
            (self.center - axis_len + 5, self.center + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2
        )

        # 顯示畫面
        cv2.imshow('UWB Visualization', canvas)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationCV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
