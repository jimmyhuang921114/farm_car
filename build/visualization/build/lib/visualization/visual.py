#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QObject, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QImage, QPixmap, QFont, QPolygonF


# ===== Configurable Variables =====
# Visualization parameters
VIS_RANGE_CM = 1000         # Total visualization range in cm (view spans -VIS_RANGE_CM/2 to +VIS_RANGE_CM/2)
GRID_SPACING_CM = 100       # Grid spacing in cm

# Car parameters (dimensions in cm)
CAR_WIDTH = 57              # Car width
CAR_LENGTH = 100            # Car length
# 車子預設面向正 Y 方向，(0,0) 為車子「前中心」。

# Sensor positions relative to the car's front-center (in cm)
SENSOR_POSITIONS = {
    0: (0, 0),
    2: (28.5, -40),
    3: (-28.5, -40),
    5: (0, -100)
}

# ROS topics
TOPIC_RGB = '/camera/color'
TOPIC_DEPTH = '/camera/depth'
TOPIC_TRACKING = '/tracking_image'    # assumed topic name for tracking image
TOPIC_MASKED = '/masked_image'        # assumed topic name for masked image
TOPIC_UWB = '/uwb_filtered_position'
TOPIC_OBSTACLE = '/obstacle/xy_list'

# ===== ROS Signals =====
class RosSignals(QObject):
    new_localized_point = Signal(object)  # 由 /uwb_filtered_position 取得的定位點
    new_rgb_image = Signal(object)        # QImage for RGB
    new_depth_image = Signal(object)      # QImage for depth
    new_tracking_image = Signal(object)   # QImage for tracking
    new_masked_image = Signal(object)     # QImage for masked
    new_obstacles = Signal(object)        # Float32MultiArray for obstacles

# ===== ROS2 Node =====
class Ros2Node(Node):
    def __init__(self, signals):
        super().__init__('pyqt_ros_interface')
        self.signals = signals
        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(PointStamped, TOPIC_UWB, self.uwb_callback, 10)
        self.create_subscription(Image, TOPIC_RGB, self.rgb_callback, 10)
        self.create_subscription(Image, TOPIC_DEPTH, self.depth_callback, 10)
        self.create_subscription(Image, TOPIC_TRACKING, self.tracking_callback, 10)
        self.create_subscription(Image, TOPIC_MASKED, self.masked_callback, 10)
        self.create_subscription(Float32MultiArray, TOPIC_OBSTACLE, self.obstacle_callback, 10)

    def uwb_callback(self, msg):
        # 這裡不再把車子畫在該點，而是把這個當作「定位點」傳給前端
        self.signals.new_localized_point.emit(msg)

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            bytesPerLine = 3 * width
            q_img = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_BGR888)
            self.signals.new_rgb_image.emit(q_img)
        except Exception as e:
            self.get_logger().error("RGB image conversion error: " + str(e))

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Normalize depth for display purposes
            cv_norm = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_norm = cv_norm.astype('uint8')
            height, width = cv_norm.shape
            bytesPerLine = width
            q_img = QImage(cv_norm.data, width, height, bytesPerLine, QImage.Format_Grayscale8)
            self.signals.new_depth_image.emit(q_img)
        except Exception as e:
            self.get_logger().error("Depth image conversion error: " + str(e))

    def tracking_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            bytesPerLine = 3 * width
            q_img = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_BGR888)
            self.signals.new_tracking_image.emit(q_img)
        except Exception as e:
            self.get_logger().error("Tracking image conversion error: " + str(e))

    def masked_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            bytesPerLine = 3 * width
            q_img = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_BGR888)
            self.signals.new_masked_image.emit(q_img)
        except Exception as e:
            self.get_logger().error("Masked image conversion error: " + str(e))

    def obstacle_callback(self, msg):
        self.signals.new_obstacles.emit(msg)

# ===== Image Display Widget =====
class ImageDisplayWidget(QWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.title = title
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.title_label = QLabel(self.title)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        placeholder = QPixmap(160, 90)
        placeholder.fill(Qt.black)
        self.image_label.setPixmap(placeholder)
        layout.addWidget(self.title_label)
        layout.addWidget(self.image_label)
        self.setLayout(layout)

    def update_image(self, q_img):
        if q_img is not None:
            pix = QPixmap.fromImage(q_img)
            scaled_pix = pix.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pix)

    def resizeEvent(self, event):
        if self.image_label.pixmap() is not None:
            pix = self.image_label.pixmap()
            scaled_pix = pix.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pix)
        super().resizeEvent(event)

# ===== Coordinate View Widget =====
class CoordinateViewWidget(QWidget):
    """
    - 固定車子和感應器在原點(0,0)。
    - /uwb_filtered_position 當作「定位點」(localized_point)，畫在地圖上。
    - /obstacle/xy_list 照常在地圖上繪製。
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.localized_point = None  # /uwb_filtered_position 的 (x, y)
        self.obstacles = []          # [(x1,y1), (x2,y2), ...]
        self.setMinimumSize(400, 400)

    def update_localized_point(self, point_msg):
        # 只更新定位點，不動車子
        self.localized_point = (point_msg.point.x, point_msg.point.y)
        self.update()

    def update_obstacles(self, array_msg):
        data = array_msg.data
        self.obstacles.clear()
        if len(data) % 2 == 0:
            for i in range(0, len(data), 2):
                self.obstacles.append((data[i], data[i+1]))
        else:
            print("Obstacle data length is not even.")
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.TextAntialiasing, True)

        # 先清空背景
        painter.fillRect(self.rect(), Qt.white)

        w = self.width()
        h = self.height()
        scale_factor = min(w, h) / VIS_RANGE_CM
        painter.translate(w / 2, h / 2)
        painter.scale(scale_factor, -scale_factor)  # 翻轉 Y 軸，使正 Y 向上

        half_range = VIS_RANGE_CM / 2

        # (1) 繪製網格線
        pen_grid = QPen(QColor(200, 200, 200))
        pen_grid.setWidthF(0)
        painter.setPen(pen_grid)
        x = -half_range
        while x <= half_range:
            painter.drawLine(x, -half_range, x, half_range)
            x += GRID_SPACING_CM
        y = -half_range
        while y <= half_range:
            painter.drawLine(-half_range, y, half_range, y)
            y += GRID_SPACING_CM

        # (2) 繪製 X 與 Y 軸
        pen_axis = QPen(QColor(0, 0, 0))
        pen_axis.setWidthF(0)
        painter.setPen(pen_axis)
        painter.drawLine(0, -half_range, 0, half_range)
        painter.drawLine(-half_range, 0, half_range, 0)

        # (3) 繪製「固定」的車子
        # 車子中心假設在 (0,0)，車子面向 +Y
        car_polygon = [
            (-CAR_WIDTH / 2, 0),
            ( CAR_WIDTH / 2, 0),
            ( CAR_WIDTH / 2, -CAR_LENGTH),
            (-CAR_WIDTH / 2, -CAR_LENGTH)
        ]
        pen_car = QPen(QColor(0, 100, 200))
        pen_car.setWidthF(2 / scale_factor)
        painter.setPen(pen_car)
        poly = QPolygonF([QPointF(px, py) for px, py in car_polygon])
        painter.drawPolygon(poly)

        # (4) 繪製感應器位置（相對於車子 (0,0) 的固定 offset）
        pen_sensor = QPen(QColor(255, 0, 0))
        pen_sensor.setWidthF(4 / scale_factor)
        painter.setPen(pen_sensor)
        for sensor_id, offset in SENSOR_POSITIONS.items():
            sensor_x = offset[0]
            sensor_y = offset[1]
            painter.drawPoint(sensor_x, sensor_y)

            # 在螢幕座標下畫 ID
            painter.save()
            painter.resetTransform()
            painter.setPen(QColor(0, 0, 0))
            font = QFont("Arial", 8)
            painter.setFont(font)
            screen_x = sensor_x * scale_factor + w / 2
            screen_y = h / 2 - sensor_y * scale_factor
            painter.drawText(screen_x + 5, screen_y - 5, str(sensor_id))
            painter.restore()

        # (5) 繪製定位點 (localized_point)
        #     例如以一個小圈標示
        if self.localized_point is not None:
            pen_lp = QPen(QColor(0, 200, 0))
            pen_lp.setWidthF(2 / scale_factor)
            painter.setPen(pen_lp)
            px, py = self.localized_point
            # 畫一個半徑 10cm (實際可自行調整) 的小圓
            painter.drawEllipse(QPointF(px, py), 10, 10)

        # (6) 繪製障礙物
        pen_obstacle = QPen(QColor(255, 0, 255))
        pen_obstacle.setWidthF(6 / scale_factor)
        painter.setPen(pen_obstacle)
        for obs in self.obstacles:
            painter.drawPoint(obs[0], obs[1])

        # ----- 重置座標系後畫文字 -----
        painter.resetTransform()
        painter.setPen(QColor(0, 0, 0))
        font = QFont("Arial", 10)
        painter.setFont(font)
        painter.drawText(10, 20, "Coordinate View")

        # (7) 在下方/左方標示 X, Y 軸刻度
        painter.setFont(QFont("Arial", 8))
        for x_val in range(-int(half_range), int(half_range) + 1, GRID_SPACING_CM):
            screen_x = x_val * scale_factor + w / 2
            screen_y = h - 5
            painter.drawText(screen_x - 10, screen_y, f"{x_val}")

        for y_val in range(-int(half_range), int(half_range) + 1, GRID_SPACING_CM):
            screen_x = 5
            screen_y = h / 2 - y_val * scale_factor
            painter.drawText(screen_x, screen_y + 5, f"{y_val}")

# ===== Main Window =====
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 PySide6 Visualization")
        self.init_ui()
        self.init_ros()

    def init_ui(self):
        # 建立主視窗與版面配置
        central_widget = QWidget()
        main_layout = QHBoxLayout()

        # 左側：座標視窗（包含標題）
        self.coordinate_view = CoordinateViewWidget()
        coord_group = QGroupBox("Coordinate View")
        coord_layout = QVBoxLayout()
        coord_layout.addWidget(self.coordinate_view)
        coord_group.setLayout(coord_layout)
        main_layout.addWidget(coord_group, stretch=2)

        # 右側：四個 16:9 影像顯示視窗（由上至下）
        self.rgb_display = ImageDisplayWidget("RGB Image (/camera/color)")
        self.depth_display = ImageDisplayWidget("Depth Image (/camera/depth)")
        self.tracking_display = ImageDisplayWidget("Tracking Image (/tracking_topic)")
        self.masked_display = ImageDisplayWidget("Masked Image (/masked_topic)")
        image_layout = QVBoxLayout()
        image_layout.addWidget(self.rgb_display)
        image_layout.addWidget(self.depth_display)
        image_layout.addWidget(self.tracking_display)
        image_layout.addWidget(self.masked_display)
        image_group = QGroupBox("Image Displays")
        image_group.setLayout(image_layout)
        main_layout.addWidget(image_group, stretch=1)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def init_ros(self):
        rclpy.init(args=None)
        self.ros_signals = RosSignals()
        self.ros_node = Ros2Node(self.ros_signals)

        # 連接 ROS 訊號與對應的槽函式
        self.ros_signals.new_localized_point.connect(self.handle_new_localized_point)
        self.ros_signals.new_rgb_image.connect(self.handle_new_rgb_image)
        self.ros_signals.new_depth_image.connect(self.handle_new_depth_image)
        self.ros_signals.new_tracking_image.connect(self.handle_new_tracking_image)
        self.ros_signals.new_masked_image.connect(self.handle_new_masked_image)
        self.ros_signals.new_obstacles.connect(self.handle_new_obstacles)

        # 使用 QTimer 定時呼叫 rclpy.spin_once 以整合 ROS2 訊息處理
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 每 10ms 呼叫一次

    @Slot()
    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

    @Slot(object)
    def handle_new_localized_point(self, point_msg):
        self.coordinate_view.update_localized_point(point_msg)

    @Slot(object)
    def handle_new_rgb_image(self, q_img):
        self.rgb_display.update_image(q_img)

    @Slot(object)
    def handle_new_depth_image(self, q_img):
        self.depth_display.update_image(q_img)

    @Slot(object)
    def handle_new_tracking_image(self, q_img):
        self.tracking_display.update_image(q_img)

    @Slot(object)
    def handle_new_masked_image(self, q_img):
        self.masked_display.update_image(q_img)

    @Slot(object)
    def handle_new_obstacles(self, array_msg):
        self.coordinate_view.update_obstacles(array_msg)

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

# ===== Main =====
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
