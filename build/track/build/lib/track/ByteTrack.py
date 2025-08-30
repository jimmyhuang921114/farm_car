#!/usr/bin/env python3
# bytetrack_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from datetime import datetime

from track.byte_tracker import BYTETracker
from track.basetrack import BaseTrack


class ByteTrackNode(Node):
    def __init__(self):
        super().__init__('track_ByteTrack')

        #=========Tracker 初始化參數=========
        self.tracker_args = type('', (), {})()
        self.tracker_args.track_thresh = 0.6
        self.tracker_args.match_thresh = 0.8
        self.tracker_args.track_buffer = 30
        self.tracker_args.aspect_ratio_thresh = 1.6
        self.tracker_args.min_box_area = 10
        self.tracker_args.mot20 = False

        #========== yolo 影像相關 ============
        self.bridge = CvBridge()
        self.color_image = None
        self.x1 = self.y1 = self.x2 = self.y2 = None
        self.has_detection = False
        self.fps_counter = 0
        self.last_fps_time = time.time()

        #============== 追蹤相關 ==============
        self.locked_target = None
        self.original_target_id  = None # 當初一開始鎖定的ID
        self.find_tracked_coords = []
        self.latest_tracked_coords = []
        self.lost_original_path = []  # 儲存原 ID 丟失時最後 5 點
        self.original_target_path = []  # 用來記錄原本鎖定 ID 的完整路徑



        #========== pubsub ============
        self.tracker = BYTETracker(self.tracker_args)
        self.cap = None
        self.video_path = None #'/workspace/src/track/track/cameracolorr.avi'
        self.create_subscription(String, '/track/yolo', self.yolo_detections_callback, 10)
        self.pub_annotated_image = self.create_publisher(Image, '/annotated_image', 10)
        self.publisher = self.create_publisher(Point, '/track/object', 10)
        self.timer_period = 0.1
        self.timer = None 

        # === 統計用變數 ===
        self.lost_count = 0
        self.auto_relock_count = 0
        self.manual_relock_count = 0

        if self.video_path:
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Cannot open video: {self.video_path}")
                exit(1)
            self.get_logger().info(f"Using video file as input: {self.video_path}")
        else:
            self.create_subscription(Image, '/camera/color', self.color_callback, 10)
            self.get_logger().info("Subscribed to /camera/color")


        self.get_logger().info("ByteTrack Tracking Node started.")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    def timer_callback(self):
        point_msg = Point()

        if self.locked_target is not None:
            x, y, w, h = self.locked_target.tlwh
            cx = x + w / 2
            cy = y + h / 2

            point_msg.x = float(cx)
            point_msg.y = float(cy)
            point_msg.z = 0.0 

            self.publisher.publish(point_msg)
            self.get_logger().info(f"Published locked target at ({cx:.2f}, {cy:.2f})")
        else:
            self.get_logger().info("No locked target to publish.")


    def yolo_detections_callback(self, msg):
        if self.cap:  # 從影片檔讀取
            ret, frame = self.cap.read()
            if not ret:
                rclpy.shutdown()
                return
        else:  # 從 RealSense 訂閱
            if self.color_image is None:
                return
            frame = self.color_image.copy()
        if frame is None:
            self.get_logger().warn("Frame is None. Skipping processing.")
            return

        self.fps_counter += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.get_logger().info(f"FPS: {self.fps_counter}")
            self.fps_counter = 0
            self.last_fps_time = now

        detections = []
        for det_str in msg.data.strip().split(';'):
            if not det_str:
                continue
            x1, y1, x2, y2, conf = map(float, det_str.split(','))
            detections.append([x1, y1, x2, y2, conf])

        dets = np.array(detections, dtype=np.float32)

        if dets.shape[0] == 0:
            self.has_detection = False
            self.latest_tracked_coords = []
            self.tracker.tracked_stracks.clear()
            self.tracker.lost_stracks.clear()
            self.tracker.removed_stracks.clear()

            if self.timer is not None:
                self.timer.cancel()   # <== 關掉 timer
                self.timer = None     # <== 重設

            cv2.putText(frame, "No Detection", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            self.publish_annotated_image(frame)
            cv2.imshow("ByteTrack", frame)
            key = cv2.waitKey(1) & 0xFF
            return

        img_h, img_w = frame.shape[:2]
        info_img = (img_h, img_w, 0)
        online_targets = self.tracker.update(dets, info_img, (img_h, img_w))
        found_locked_target = False
        offset = 30
        for target in online_targets:
            x, y, w, h = target.tlwh
            track_id = target.track_id
            ix, iy, iw, ih = int(x), int(y), int(w), int(h)
            cx = x + w / 2
            cy = y + h / 2
            # 偵測到人時，如果 timer 沒有開啟，就開啟
            if self.timer is None:
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
                self.get_logger().info("[INFO] Timer started because detection resumed.")


            if self.locked_target is None:
                cv2.rectangle(frame, (ix, iy), (ix + iw, iy + ih), (255, 0, 0), 2)
                cv2.putText(frame, f"ID: {track_id}", (ix, iy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                self.lost_original_path = []  # 成功 relock 後清空
                if self.locked_target.track_id == track_id:
                    self.x1, self.y1 = ix, iy
                    self.x2, self.y2 = ix + iw, iy + ih
                    self.original_target_path.append((cx, cy))
                    if len(self.original_target_path) > 20:
                        self.original_target_path.pop(0)

                    found_locked_target = True
                    cv2.rectangle(frame, (self.x1, self.y1), (self.x2, self.y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"Locked ID: {track_id}", (self.x1, self.y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    if self.locked_target.track_id not in [t.track_id for t in online_targets]:
                        if not self.lost_original_path and len(self.original_target_path) >= 5:
                            self.lost_original_path = self.original_target_path[-5:].copy()
                            self.get_logger().info(f"[INFO] Saved last 5 points before losing original ID: {self.lost_original_path}")
                        self.find_tracked_coords = []
                        self.get_logger().info("[INFO] Lost locked target, waiting for relock.")

            if self.locked_target is not None and self.locked_target.track_id != self.original_target_id:
                if track_id == self.locked_target.track_id:
                    self.find_tracked_coords.append((cx, cy))
                    if len(self.find_tracked_coords) > 20:
                        self.find_tracked_coords.pop(0)

                    for nx, ny in self.find_tracked_coords[-5:]:
                        cv2.circle(frame, (int(nx), int(ny)), 5, (255, 255, 255), -1)  # 白色點

        if self.locked_target and not found_locked_target:
            best_target = None
            min_dist = float('inf')
            center_x = (self.x1 + self.x2) / 2
            center_y = (self.y1 + self.y2) / 2
            cv2.rectangle(frame, (self.x1, self.y1), (self.x2, self.y2), (0, 255, 255), 2)
            # 畫出原本 ID 丟失前的最後5個點（藍色）
            for ox, oy in self.lost_original_path:
                cv2.circle(frame, (int(ox), int(oy)), 6, (255, 0, 0), -1)  # 藍色點

            for target in online_targets:
                x, y, w, h = target.tlwh
                cx = x + w / 2
                cy = y + h / 2
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 255), -1)

                # 目標中心點在原本框範圍內
                if self.x1 <= cx <= self.x2 and self.y1 <= cy <= self.y2:
                    dist = np.sqrt((cx - center_x)**2 + (cy - center_y)**2)
                    if dist < min_dist:
                        min_dist = dist
                        best_target = target


            if best_target is not None:
                self.locked_target = best_target
                x, y, w, h = best_target.tlwh
                self.x1, self.y1 = int(x), int(y)
                self.x2, self.y2 = int(x + w), int(y + h)
                self.auto_relock_count += 1
                self.get_logger().info(f"[INFO] Lost original target, relocked new ID {self.locked_target.track_id} in last bbox.")

                self.find_tracked_coords = []  # 清空舊的
                self.relocked_target_id = best_target.track_id  # 新增這個變數來追蹤新ID


            else:
                self.get_logger().info("[INFO] No suitable target found in last bbox, keeping unlock.")

        # --- Check if original ID is back ---
        if self.original_target_id is not None and online_targets:
            for target in online_targets:
                if target.track_id == self.original_target_id:
                    if self.locked_target is None or self.locked_target.track_id != self.original_target_id:
                        self.locked_target = target
                        x, y, w, h = target.tlwh
                        self.x1, self.y1 = int(x), int(y)
                        self.x2, self.y2 = int(x + w), int(y + h)
                        self.get_logger().info(f"[INFO] Original ID {self.original_target_id} recovered and relocked.")
                    break

        self.publish_annotated_image(frame)
        cv2.imshow("ByteTrack", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info(f"[STATS] Total lost: {self.lost_count}")
            self.get_logger().info(f"[STATS] Auto relocked: {self.auto_relock_count}")
            self.get_logger().info(f"[STATS] Manual relocked: {self.manual_relock_count}")
            rclpy.shutdown()
        elif key == ord('r'):
            self.reset_tracker()
        elif key == ord('l'):
            self.lock_nearest_target(online_targets, frame)

    def publish_annotated_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_annotated_image.publish(msg)

    def reset_tracker(self):
        self.tracker.tracked_stracks.clear()
        self.tracker.lost_stracks.clear()
        self.tracker.removed_stracks.clear()
        BaseTrack._count = 0
        self.locked_target = None
        self.get_logger().info("[INFO] Tracker reset -> UNLOCKED")

    def lock_nearest_target(self, online_targets, frame):

        # 已經鎖定的目標解除鎖定
        if self.locked_target is not None: 
            self.get_logger().info(f"[INFO] Unlocked target: ID={self.locked_target.track_id}")
            self.locked_target = None
            self.x1 = self.y1 = self.x2 = self.y2 = None
        # 鎖定最近的目標
        else:              
            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2
            min_dist = float('inf')
            closest_target = None
            for t in online_targets:
                tx, ty, tw, th = t.tlwh
                tx_center = tx + tw / 2
                ty_center = ty + th / 2
                dist = np.sqrt((tx_center - center_x)**2 + (ty_center - center_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_target = t

            if closest_target is not None:
                self.locked_target = closest_target
                self.original_target_id = closest_target.track_id
                ix, iy, iw, ih = map(int, closest_target.tlwh)
                self.x1, self.y1 = ix, iy
                self.x2, self.y2 = ix + iw, iy + ih
                self.manual_relock_count += 1
                self.get_logger().info(f"[INFO] Locked target: ID={closest_target.track_id}")

        

def main(args=None):
    rclpy.init(args=args)
    node = ByteTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
