#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import numpy as np

MODEL_PATH = "/workspace/src/track/track/best_new_l.engine"
# MODEL_PATH = "/workspace/src/track/track/best_old_l.engine"
IMAGE_PATH = "/workspace/src/track/track/frames/frame_0272.jpg"
OUTPUT_PATH = "/workspace/src/track/track/person_output5.jpg"

class YoloPersonDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_person_detect_node')

        self.model = YOLO(MODEL_PATH, task="detect")  # 使用 TensorRT .engine 模型

        image = cv2.imread(IMAGE_PATH)
        if image is None:
            self.get_logger().error(f"Cannot read image: {IMAGE_PATH}")
            rclpy.shutdown()
            return

        results = self.model(image, device=0)

        boxes = results[0].boxes
        if boxes is None:
            self.get_logger().info("No detections found.")
            rclpy.shutdown()
            return

        # 取得類別、座標、置信度
        cls_ids = boxes.cls.cpu().numpy().astype(int)   # 類別
        xyxys = boxes.xyxy.cpu().numpy()               # 邊界框
        confs = boxes.conf.cpu().numpy()               # 信心值

        person_found = False

        for cls_id, xyxy, conf in zip(cls_ids, xyxys, confs):
            if cls_id == 0:  # 0 是 YOLO COCO 預設的 "person" 類別
                x1, y1, x2, y2 = map(int, xyxy)
                label = f"person {conf:.2f}"
                print(f"person detected: ({x1}, {y1}), ({x2}, {y2}), conf={conf:.2f}")
                person_found = True

                # 畫框
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # cv2.putText(image, label, (x1+20, y1 + 50),
                #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(image, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if person_found:
            cv2.imwrite(OUTPUT_PATH, image)
            self.get_logger().info(f"Output image saved: {OUTPUT_PATH}")
        else:
            # 沒有偵測到人，在圖片上標示 "no detection"
            cv2.putText(image, "no detection", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            cv2.imwrite(OUTPUT_PATH, image)
            self.get_logger().info("No person detected. Output image saved with 'no detection' text.")


        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonDetectNode()

if __name__ == '__main__':
    main()
