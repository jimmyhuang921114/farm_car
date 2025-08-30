#!/usr/bin/env python3
import sys
import time
import math
import random

import cv2
import rclpy
from rclpy.node import Node
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox,
    QGridLayout, QPushButton, QSizePolicy, QSplitter
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QObject, QRect, QPointF
from PySide6.QtGui import (
    QPainter, QPen, QColor, QFont, QPolygonF, QPixmap, QCursor, QImage
)

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError

# ===== Constants =====
VIS_RANGE_CM = 1000
GRID_SPACING_CM = 100
CAR_WIDTH = 57
CAR_LENGTH = 100

# Depth display scale (adjust according to your sensor max range)
DEPTH_SCALE_ALPHA = 255.0 / 5000.0

# Timeout (in seconds) after which a topic is considered lost
TOPIC_TIMEOUT_SEC = 2.0

# Main car's sensor positions (example)
SENSOR_POSITIONS = {
    0: (0, 0),
    2: (28.5, -40),
    3: (-28.5, -40),
    5: (0, -100)
}

# ===== ROS Signals =====
class RosSignals(QObject):
    new_localized_point = Signal(object)      # /uwb_filtered_position
    new_obstacles = Signal(object)            # /obstacle/xy_list
    new_rgb_image = Signal(object)            # QImage from /camera/color
    new_depth_image = Signal(object)          # QImage from /camera/depth
    new_tracking_image = Signal(object)       # QImage from /tracking_image
    new_masked_image = Signal(object)         # QImage from /masked_image
    new_ultrasonic_data_1 = Signal(object)    # /ultrasonic_data_1
    new_ultrasonic_data_2 = Signal(object)    # /ultrasonic_data_2
    new_people_track = Signal(object)         # 新增：/people_track 的 track 結果

# ===== ROS2 Node =====
class Ros2Node(Node):
    def __init__(self, signals):
        super().__init__('pyqt_ros_interface')
        self.signals = signals
        self.bridge = CvBridge()

        self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        self.create_subscription(Float32MultiArray, '/obstacle/xy_list', self.obstacle_callback, 10)
        self.create_subscription(Image, '/camera/color', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Image, '/annotated_image', self.tracking_callback, 10)
        self.create_subscription(Image, '/masked_image', self.masked_callback, 10)
        self.create_subscription(Int16MultiArray, '/ultrasonic_data_1', self.ultrasonic_data_1_callback, 10)
        self.create_subscription(Int16MultiArray, '/ultrasonic_data_2', self.ultrasonic_data_2_callback, 10)
        # 新增 /people_track 訂閱，格式為 Int32MultiArray
        self.create_subscription(Int32MultiArray, '/people_track', self.people_track_callback, 10)

    def uwb_callback(self, msg):
        self.signals.new_localized_point.emit(msg)

    def obstacle_callback(self, msg):
        self.signals.new_obstacles.emit(msg)

    def rgb_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_img.shape
            bytes_line = 3 * w
            q_img = QImage(cv_img.data, w, h, bytes_line, QImage.Format_BGR888)
            self.signals.new_rgb_image.emit(q_img)
        except CvBridgeError as e:
            self.get_logger().error(f"rgb_callback error: {e}")

    def depth_callback(self, msg):
        try:
            cv16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv8 = cv2.convertScaleAbs(cv16, alpha=DEPTH_SCALE_ALPHA)
            h, w = cv8.shape
            bytes_line = w
            q_img = QImage(cv8.data, w, h, bytes_line, QImage.Format_Grayscale8)
            self.signals.new_depth_image.emit(q_img)
        except CvBridgeError as e:
            self.get_logger().error(f"depth_callback error: {e}")

    def tracking_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_img.shape
            bytes_line = 3 * w
            q_img = QImage(cv_img.data, w, h, bytes_line, QImage.Format_BGR888)
            self.signals.new_tracking_image.emit(q_img)
        except CvBridgeError as e:
            self.get_logger().error(f"tracking_callback error: {e}")

    def masked_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_img.shape
            bytes_line = 3 * w
            q_img = QImage(cv_img.data, w, h, bytes_line, QImage.Format_BGR888)
            self.signals.new_masked_image.emit(q_img)
        except CvBridgeError as e:
            self.get_logger().error(f"masked_callback error: {e}")

    def ultrasonic_data_1_callback(self, msg):
        self.signals.new_ultrasonic_data_1.emit(msg)

    def ultrasonic_data_2_callback(self, msg):
        self.signals.new_ultrasonic_data_2.emit(msg)
        
    # 新增 /people_track 回呼函數
    def people_track_callback(self, msg):
        self.signals.new_people_track.emit(msg)

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
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.title_label)
        layout.addWidget(self.image_label)
        self.setLayout(layout)

    def update_image(self, q_img):
        if q_img is not None:
            pix = QPixmap.fromImage(q_img)
            scaled_pix = pix.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pix)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_label.pixmap() is not None:
            pix = self.image_label.pixmap()
            scaled_pix = pix.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pix)

# ===== Coordinate View Widget =====
class CoordinateViewWidget(QWidget):
    """
    Draws:
      - A grid and axes.
      - A rectangular car.
      - A localized point and obstacles.
      - Two ultrasonic mini-car displays, now arranged side by side in the bottom-right corner.
      - 新增：people_track 的點 (以藍色圓圈顯示)
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.localized_point = None
        self.obstacles = []
        self.ultrasonic_data_1 = [0] * 8
        self.ultrasonic_data_2 = [0] * 8
        # 新增：存放 /people_track 的資料，格式為 [(x, y), ...]
        self.people_track = []
        self.setMinimumSize(400, 400)

    def update_localized_point(self, point_msg):
        self.localized_point = (point_msg.point.x, point_msg.point.y)
        self.update()

    def update_obstacles(self, array_msg):
        data = array_msg.data
        self.obstacles.clear()
        if len(data) % 2 == 0:
            for i in range(0, len(data), 2):
                self.obstacles.append((data[i], data[i+1]))
        self.update()

    def update_ultrasonic_data1(self, array_msg):
        self.ultrasonic_data_1 = [int(v) for v in array_msg.data]
        self.update()

    def update_ultrasonic_data2(self, array_msg):
        self.ultrasonic_data_2 = [int(v) for v in array_msg.data]
        self.update()

    # 新增：更新 people_track 的資料
    def update_people_track(self, array_msg):
        data = array_msg.data
        self.people_track.clear()
        if len(data) % 2 == 0:
            for i in range(0, len(data), 2):
                self.people_track.append((data[i], data[i+1]))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), Qt.white)

        w = self.width()
        h = self.height()
        scale_factor = min(w, h) / VIS_RANGE_CM

        # Set origin to center and flip y-axis
        painter.translate(w/2, h/2)
        painter.scale(scale_factor, -scale_factor)

        half_range = VIS_RANGE_CM / 2

        # Draw grid lines
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

        # Draw axes
        pen_axis = QPen(Qt.black)
        pen_axis.setWidthF(0)
        painter.setPen(pen_axis)
        painter.drawLine(0, -half_range, 0, half_range)
        painter.drawLine(-half_range, 0, half_range, 0)

        # Draw main car as a rectangle
        car_polygon = [
            (-CAR_WIDTH/2, 0),
            ( CAR_WIDTH/2, 0),
            ( CAR_WIDTH/2, -CAR_LENGTH),
            (-CAR_WIDTH/2, -CAR_LENGTH)
        ]
        pen_car = QPen(QColor(0, 100, 200))
        pen_car.setWidthF(3/scale_factor)
        painter.setPen(pen_car)
        painter.setBrush(Qt.NoBrush)
        poly = QPolygonF([QPointF(px, py) for px, py in car_polygon])
        painter.drawPolygon(poly)

        # Draw sensor positions
        pen_sensor = QPen(Qt.red)
        pen_sensor.setWidthF(5/scale_factor)
        painter.setPen(pen_sensor)
        for sensor_id, (sx, sy) in SENSOR_POSITIONS.items():
            painter.drawPoint(sx, sy)
            painter.save()
            painter.resetTransform()
            painter.setPen(Qt.black)
            font = QFont("Arial", 10)
            painter.setFont(font)
            screen_x = sx * scale_factor + w/2
            screen_y = h/2 - sy * scale_factor
            painter.drawText(screen_x+5, screen_y-5, str(sensor_id))
            painter.restore()

        # Draw localized point (UWB)
        if self.localized_point is not None:
            pen_lp = QPen(QColor(0, 200, 0))
            pen_lp.setWidthF(3/scale_factor)
            painter.setPen(pen_lp)
            px, py = self.localized_point
            painter.drawEllipse(QPointF(px, py), 10, 10)

        # Draw obstacles
        pen_obs = QPen(QColor(255, 0, 255))
        pen_obs.setWidthF(6/scale_factor)
        painter.setPen(pen_obs)
        for ox, oy in self.obstacles:
            painter.drawPoint(ox, oy)

        # 新增：繪製 /people_track 的點 (以藍色圓圈顯示)
        if self.people_track:
            pen_track = QPen(QColor(0, 0, 255))
            pen_track.setWidthF(3/scale_factor)
            painter.setPen(pen_track)
            for tx, ty in self.people_track:
                painter.drawEllipse(QPointF(tx, ty), 10, 10)

        # Reset transform for UI overlay
        painter.resetTransform()
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(10, 20, "Robot View")

        # Draw axis labels
        painter.setFont(QFont("Arial", 9))
        for x_val in range(-int(half_range), int(half_range)+1, GRID_SPACING_CM):
            screen_x = x_val * scale_factor + w/2
            screen_y = h - 5
            painter.drawText(screen_x-15, screen_y, f"{x_val}")
        for y_val in range(-int(half_range), int(half_range)+1, GRID_SPACING_CM):
            screen_x = 5
            screen_y = h/2 - y_val * scale_factor
            painter.drawText(screen_x, screen_y+10, f"{y_val}")

        # Draw two miniature ultrasonic displays (side by side at bottom-right)
        margin = 10
        gap = 10
        mini_size = 300  # enlarged region
        total_width = 2 * mini_size + gap
        x0 = w - total_width - margin
        y0 = h - mini_size - margin

        def draw_mini_car(rx, ry, sensor_data):
            painter.setPen(Qt.black)
            painter.setBrush(QColor(220, 220, 220))
            mini_car_w = 50  # increased size
            mini_car_h = 75
            halfW = mini_car_w / 2
            halfH = mini_car_h / 2
            cx = rx + mini_size/2
            cy = ry + mini_size/2
            car_rect = QRect(int(cx - halfW), int(cy - halfH),
                             mini_car_w, mini_car_h)
            painter.drawRect(car_rect)
            # Positions: left side: two sensors; front: two sensors; right: two sensors; back: two sensors.
            offset = 50  # increased offset for clarity
            top_bot_offset = 20
            positions = [
                (cx - halfW - offset, cy - halfH/2),  # left-top
                (cx - halfW - offset, cy + halfH/2),  # left-bottom

                (cx - halfW/2 - top_bot_offset, cy + halfH + offset),  # back-left
                (cx + halfW/2 + top_bot_offset, cy + halfH + offset),  # back-right

                (cx + halfW + offset, cy + halfH/2),  # right-bottom
                (cx + halfW + offset, cy - halfH/2),  # right-top
                                                
                (cx + halfW/2 + top_bot_offset, cy - halfH - offset),  # front-right    
                (cx - halfW/2 - top_bot_offset, cy - halfH - offset),  # front-left         
            ]
            painter.setFont(QFont("Arial", 14))
            for i, pos in enumerate(positions):
                val = sensor_data[i]
                rect = QRect(int(pos[0]-20), int(pos[1]-15), 40, 30)
                painter.drawText(rect, Qt.AlignCenter, str(val))

        # Draw the first ultrasonic display on the left of the two
        draw_mini_car(x0, y0, self.ultrasonic_data_1)
        # Draw the second ultrasonic display to its right
        draw_mini_car(x0 + mini_size + gap, y0, self.ultrasonic_data_2)

# ===== Control Panel Widget =====
class ControlPanelWidget(QWidget):
    """
    A separate control panel containing four buttons and a compact status light grid.
    """
    def __init__(self, status_labels, status_lights, parent=None):
        super().__init__(parent)
        self.status_labels = status_labels
        self.status_lights = status_lights
        self.init_ui()

    def init_ui(self):
        btn_start_rec = QPushButton("Start Rec")
        btn_stop_rec = QPushButton("Stop Rec")
        btn_start_raw = QPushButton("Start Raw")
        btn_stop_raw = QPushButton("Stop Raw")

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(btn_start_rec)
        btn_layout.addWidget(btn_stop_rec)
        btn_layout.addWidget(btn_start_raw)
        btn_layout.addWidget(btn_stop_raw)

        status_grid = QGridLayout()
        status_grid.setHorizontalSpacing(15)
        status_grid.setVerticalSpacing(5)
        topics = list(self.status_labels.keys())
        for i, key in enumerate(topics):
            row = i // 4
            col = i % 4
            label = self.status_labels[key]
            light = self.status_lights[key]
            label.setFont(QFont("Arial", 9))
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            status_grid.addWidget(label, row, col*2)
            status_grid.addWidget(light, row, col*2 + 1)

        main_layout = QVBoxLayout()
        main_layout.addLayout(btn_layout)
        main_layout.addLayout(status_grid)
        self.setLayout(main_layout)

# ===== Main Window =====
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Visualization")

        # Topic received flags
        self.uwb_received = False
        self.obstacle_received = False
        self.rgb_received = False
        self.depth_received = False
        self.tracking_received = False
        self.masked_received = False
        self.ultra1_received = False
        self.ultra2_received = False
        # 新增：people_track 收到的 flag
        self.people_track_received = False

        now = time.time()
        self.last_msg_time = {
            "UWB": now,
            "Obstacle": now,
            "Color": now,
            "Depth": now,
            "Tracking": now,
            "Masked": now,
            "Ultra1": now,
            "Ultra2": now,
            "PeopleTrack": now  # 新增
        }

        self.init_ui()
        self.init_ros()
        self.update_status_lights()

    def init_ui(self):
        # Left: Robot/Coordinate View
        self.coordinate_view = CoordinateViewWidget()
        left_group = QGroupBox("Robot View")
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.coordinate_view)
        left_group.setLayout(left_layout)

        # Right top: Camera Streams (2x2 image grid)
        self.rgb_display = ImageDisplayWidget("RGB Image (/camera/color)")
        self.depth_display = ImageDisplayWidget("Depth Image (/camera/depth)")
        self.tracking_display = ImageDisplayWidget("Tracking (/tracking_image)")
        self.masked_display = ImageDisplayWidget("Masked (/masked_image)")
        image_grid = QGridLayout()
        image_grid.addWidget(self.rgb_display, 0, 0)
        image_grid.addWidget(self.depth_display, 0, 1)
        image_grid.addWidget(self.tracking_display, 1, 0)
        image_grid.addWidget(self.masked_display, 1, 1)
        cam_widget = QWidget()
        cam_widget.setLayout(image_grid)
        cam_group = QGroupBox("Camera Streams")
        cam_layout = QVBoxLayout()
        cam_layout.addWidget(cam_widget)
        cam_group.setLayout(cam_layout)

        # Right bottom: Control Panel (buttons and status lights)
        self.status_labels = {
            "UWB": QLabel("UWB"),
            "Obstacle": QLabel("Obstacle"),
            "Color": QLabel("Color"),
            "Depth": QLabel("Depth"),
            "Tracking": QLabel("Track"),
            "Masked": QLabel("Masked"),
            "Ultra1": QLabel("U1"),
            "Ultra2": QLabel("U2"),
            "PeopleTrack": QLabel("PeopleTrack")  # 新增
        }
        self.status_lights = {
            "UWB": QLabel(),
            "Obstacle": QLabel(),
            "Color": QLabel(),
            "Depth": QLabel(),
            "Tracking": QLabel(),
            "Masked": QLabel(),
            "Ultra1": QLabel(),
            "Ultra2": QLabel(),
            "PeopleTrack": QLabel()  # 新增
        }
        control_panel = ControlPanelWidget(self.status_labels, self.status_lights)
        control_group = QGroupBox("Control Panel")
        control_layout = QVBoxLayout()
        control_layout.addWidget(control_panel)
        control_group.setLayout(control_layout)

        # Right side vertical splitter (top: camera streams, bottom: control panel)
        right_vsplitter = QSplitter(Qt.Vertical)
        right_vsplitter.addWidget(cam_group)
        right_vsplitter.addWidget(control_group)
        right_vsplitter.setStretchFactor(0, 3)
        right_vsplitter.setStretchFactor(1, 1)
        right_vsplitter.setSizes([500, 150])

        # Main horizontal splitter: left and right
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(left_group)
        main_splitter.addWidget(right_vsplitter)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([600, 600])
        self.setCentralWidget(main_splitter)

    def init_ros(self):
        rclpy.init(args=None)
        self.ros_signals = RosSignals()
        self.ros_node = Ros2Node(self.ros_signals)
        self.ros_signals.new_localized_point.connect(self.handle_localized_point)
        self.ros_signals.new_obstacles.connect(self.handle_obstacles)
        self.ros_signals.new_rgb_image.connect(self.handle_rgb_image)
        self.ros_signals.new_depth_image.connect(self.handle_depth_image)
        self.ros_signals.new_tracking_image.connect(self.handle_tracking_image)
        self.ros_signals.new_masked_image.connect(self.handle_masked_image)
        self.ros_signals.new_ultrasonic_data_1.connect(self.handle_ultra1)
        self.ros_signals.new_ultrasonic_data_2.connect(self.handle_ultra2)
        # 新增：people_track 訊號連接
        self.ros_signals.new_people_track.connect(self.handle_people_track)
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)  # 20 Hz

    @Slot()
    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)
        self.check_timeouts()

    def check_timeouts(self):
        now = time.time()
        if now - self.last_msg_time["UWB"] > TOPIC_TIMEOUT_SEC:
            self.uwb_received = False
        if now - self.last_msg_time["Obstacle"] > TOPIC_TIMEOUT_SEC:
            self.obstacle_received = False
        if now - self.last_msg_time["Color"] > TOPIC_TIMEOUT_SEC:
            self.rgb_received = False
        if now - self.last_msg_time["Depth"] > TOPIC_TIMEOUT_SEC:
            self.depth_received = False
        if now - self.last_msg_time["Tracking"] > TOPIC_TIMEOUT_SEC:
            self.tracking_received = False
        if now - self.last_msg_time["Masked"] > TOPIC_TIMEOUT_SEC:
            self.masked_received = False
        if now - self.last_msg_time["Ultra1"] > TOPIC_TIMEOUT_SEC:
            self.ultra1_received = False
        if now - self.last_msg_time["Ultra2"] > TOPIC_TIMEOUT_SEC:
            self.ultra2_received = False
        # 新增：people_track timeout 檢查
        if now - self.last_msg_time["PeopleTrack"] > TOPIC_TIMEOUT_SEC:
            self.people_track_received = False
        self.update_status_lights()

    def make_light_pixmap(self, color):
        size = 10
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setBrush(QColor(color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, size, size)
        painter.end()
        return pixmap

    def update_status_lights(self):
        def col(flag): return "green" if flag else "red"
        self.status_lights["UWB"].setPixmap(self.make_light_pixmap(col(self.uwb_received)))
        self.status_lights["Obstacle"].setPixmap(self.make_light_pixmap(col(self.obstacle_received)))
        self.status_lights["Color"].setPixmap(self.make_light_pixmap(col(self.rgb_received)))
        self.status_lights["Depth"].setPixmap(self.make_light_pixmap(col(self.depth_received)))
        self.status_lights["Tracking"].setPixmap(self.make_light_pixmap(col(self.tracking_received)))
        self.status_lights["Masked"].setPixmap(self.make_light_pixmap(col(self.masked_received)))
        self.status_lights["Ultra1"].setPixmap(self.make_light_pixmap(col(self.ultra1_received)))
        self.status_lights["Ultra2"].setPixmap(self.make_light_pixmap(col(self.ultra2_received)))
        # 新增：people_track 狀態指示
        self.status_lights["PeopleTrack"].setPixmap(self.make_light_pixmap(col(self.people_track_received)))

    @Slot(object)
    def handle_localized_point(self, msg):
        self.uwb_received = True
        self.last_msg_time["UWB"] = time.time()
        self.coordinate_view.update_localized_point(msg)
        self.update_status_lights()

    @Slot(object)
    def handle_obstacles(self, msg):
        self.obstacle_received = True
        self.last_msg_time["Obstacle"] = time.time()
        self.coordinate_view.update_obstacles(msg)
        self.update_status_lights()

    @Slot(object)
    def handle_rgb_image(self, q_img):
        self.rgb_received = True
        self.last_msg_time["Color"] = time.time()
        self.rgb_display.update_image(q_img)
        self.update_status_lights()

    @Slot(object)
    def handle_depth_image(self, q_img):
        self.depth_received = True
        self.last_msg_time["Depth"] = time.time()
        self.depth_display.update_image(q_img)
        self.update_status_lights()

    @Slot(object)
    def handle_tracking_image(self, q_img):
        self.tracking_received = True
        self.last_msg_time["Tracking"] = time.time()
        self.tracking_display.update_image(q_img)
        self.update_status_lights()

    @Slot(object)
    def handle_masked_image(self, q_img):
        self.masked_received = True
        self.last_msg_time["Masked"] = time.time()
        self.masked_display.update_image(q_img)
        self.update_status_lights()

    @Slot(object)
    def handle_ultra1(self, msg):
        self.ultra1_received = True
        self.last_msg_time["Ultra1"] = time.time()
        self.coordinate_view.update_ultrasonic_data1(msg)
        self.update_status_lights()

    @Slot(object)
    def handle_ultra2(self, msg):
        self.ultra2_received = True
        self.last_msg_time["Ultra2"] = time.time()
        self.coordinate_view.update_ultrasonic_data2(msg)
        self.update_status_lights()

    # 新增：處理 /people_track 的回呼
    @Slot(object)
    def handle_people_track(self, msg):
        self.people_track_received = True
        self.last_msg_time["PeopleTrack"] = time.time()
        self.coordinate_view.update_people_track(msg)
        self.update_status_lights()

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

# ===== main =====
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
