#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
from collections import deque
from scipy.spatial import distance

# 定义发布主题
TOPIC_OBSTACLE = '/obstacle/xy_list'
TOPIC_MASKED = '/masked_image'
TOPIC_DEPTH_VIZ = '/depth_viz'

class ObstacleDetectPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_detect_publisher')
        
        # 初始化 CvBridge
        self.bridge = CvBridge()

        # 订阅RealSense图像
        self.color_sub = self.create_subscription(Image, '/camera/color', self.color_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth', self.depth_cb, 10)

        # 初始化发布者
        self.xy_pub = self.create_publisher(Float32MultiArray, TOPIC_OBSTACLE, 10)
        self.masked_pub = self.create_publisher(Image, TOPIC_MASKED, 10)
        self.depth_viz_pub = self.create_publisher(Image, TOPIC_DEPTH_VIZ, 10)

        # 加载YOLOv8模型
        self.model = YOLO("src/obstacle_detect_last/obstacle_detect_last/epoch110.pt")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.get_logger().info("YOLOv8模型加载成功")

        # 相机参数 (根据用户提供的数据)
        self.fx = 607.0333862304688
        self.fy = 606.1414794921875
        self.cx = 321.22332763671875
        self.cy = 239.4711456298828
        self.dist_coeffs = np.zeros(5)  # 畸变系数 [k1, k2, p1, p2, k3]
                # 在__init__中修改安装参数（保持米单位输入）
        self.camera_offset_z = 0.4  # 单位：米（代码中会自动转换为40厘米）
        self.camera_offset_x = 0.0   # 单位：米
        # 安装参数
        self.camera_offset_x = 0.0
        self.camera_offset_z = 0.4  # 假设相机安装位置前移40cm
        self.enable_mirror = True

        # 初始化变量
        self.color_img = None
        self.depth_img = None

    def color_cb(self, msg):
        """处理彩色图像回调"""
        self.color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        """处理深度图像回调"""
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.process_data()

    def process_data(self):
        """主处理流程"""
        if self.color_img is None or self.depth_img is None:
            return

        # 图像预处理
        color = cv2.resize(self.color_img, (640, 480))
        depth = cv2.resize(self.depth_img, (640, 480))

        # 发布深度可视化
        self.publish_depth_viz(depth)

        # YOLO分割推理
        try:
            results = self.model(color)
        except Exception as e:
            self.get_logger().error(f"模型推理失败: {str(e)}")
            return

        if not results or results[0].masks is None:
            return

        # 生成并发布遮罩图像
        masked_img = self.draw_masks(color, results)
        self.publish_masked_image(masked_img)

        # 计算障碍物坐标
        points = []
        for mask in results[0].masks.data:
            mask_np = mask.cpu().numpy().astype(np.uint8)
            mask_resized = cv2.resize(mask_np, (640, 480))
            points += self.calculate_coordinates(mask_resized, depth)

        # 发布坐标数据
        self.publish_obstacle_points(points)

    def draw_masks(self, img, results):
        """绘制分割遮罩"""
        masked_img = img.copy()
        for mask in results[0].masks.data:
            mask_np = mask.cpu().numpy().astype(np.uint8) * 255
            mask_resized = cv2.resize(mask_np, (640, 480))
            
            # 绘制半透明遮罩
            overlay = masked_img.copy()
            overlay[mask_resized > 0] = (255, 0, 0)  # 蓝色
            masked_img = cv2.addWeighted(overlay, 0.3, masked_img, 0.7, 0)
            
            # 绘制边界框
            contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(masked_img, contours, -1, (0,255,0), 2)
        
        return masked_img

    def calculate_coordinates(self, mask, depth):
        """精确坐标转换（厘米单位输出）"""
        h, w = mask.shape
        points = []
        
        # 获取有效像素坐标
        y_indices, x_indices = np.where(mask > 0)
        if len(y_indices) == 0:
            return points
        
        # 随机采样点（每个障碍物最多50个点）
        for i in np.random.choice(len(y_indices), min(50, len(y_indices))):
            x = x_indices[i]
            y = y_indices[i]
            
            raw_depth = depth[y, x]  # 原始深度值（毫米）
            
            # 深度值有效性检查（30cm~5m）
            if raw_depth < 300 or raw_depth > 5000:  # 300mm=30cm, 5000mm=5m
                continue
            
            try:
                # 毫米转厘米（直接除以10）
                depth_cm = raw_depth // 10  # 整数除法提高效率
                
                # 去畸变处理
                pts = np.array([[[x, y]]], dtype=np.float32)
                undistorted = cv2.undistortPoints(
                    pts,
                    cameraMatrix=np.array([
                        [self.fx, 0, self.cx],
                        [0, self.fy, self.cy],
                        [0, 0, 1]
                    ]),
                    distCoeffs=self.dist_coeffs
                )
                x_norm, y_norm = undistorted[0][0]
                
                # 3D坐标计算（厘米单位）
                Z_cm = depth_cm + int(self.camera_offset_z * 100)  # 米转厘米
                X_cm = x_norm * (Z_cm / 100.0) * 100  # 归一化坐标转厘米
                X_cm += self.camera_offset_x * 100  # 米转厘米
                

                    
                # 精度控制（保留1位小数）
                X_cm = round(X_cm, 1)
                Z_cm = round(Z_cm, 1)
                
                points.append([X_cm, Z_cm])
                
                # 调试日志
                self.get_logger().debug(
                    f"转换结果: "
                    f"原始深度={raw_depth}mm -> "
                    f"X={X_cm}cm Z={Z_cm}cm "
                    f"(offset_x={self.camera_offset_x}m offset_z={self.camera_offset_z}m)"
                )
                
            except Exception as e:
                self.get_logger().error(f"坐标转换异常: {str(e)}")
        
        return points



    def publish_obstacle_points(self, points):
        """安全发布障碍物坐标（增强数值检查）"""
        data = []
        valid_count = 0
        FLOAT32_MIN = -3.4028235e+38
        FLOAT32_MAX = 3.4028235e+38
        
        for point in points:
            # 数据维度验证
            if len(point) != 2:
                self.get_logger().warn(f"无效数据维度: {point}")
                continue
            
            try:
                # 转换为浮点数并验证数值范围
                x = float(round(point[0], 1))
                z = float(round(point[1], 1))
                
                # 数值有效性检查
                if not (FLOAT32_MIN < x < FLOAT32_MAX) or \
                not (FLOAT32_MIN < z < FLOAT32_MAX):
                    self.get_logger().warn(f"数值超出float32范围: ({x}, {z})")
                    continue
                    
                if np.isnan(x) or np.isnan(z) or \
                np.isinf(x) or np.isinf(z):
                    self.get_logger().warn(f"非正常数值: ({x}, {z})")
                    continue
                    
                data.extend([x, z])
                valid_count += 1
                
            except Exception as e:
                self.get_logger().error(f"数据转换异常: {str(e)}")

        # 数据长度对齐
        if len(data) % 2 != 0:
            self.get_logger().warn("数据长度为奇数，进行截断")
            data = data[:-1]
            valid_count -= 1

        # 创建并发布消息
        if valid_count > 0:
            msg = Float32MultiArray()
            msg.data = data
            self.xy_pub.publish(msg)
            self.get_logger().info(f"成功发布 {valid_count} 个有效数据点")
        else:
            self.get_logger().warn("无有效数据可发布")

    def publish_masked_image(self, img):
        """发布处理后的图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.masked_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"图像发布失败: {str(e)}")

    def publish_depth_viz(self, depth):
        """发布深度可视化图像"""
        try:
            # 转换为伪彩色
            depth_viz = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_viz = cv2.applyColorMap(depth_viz.astype(np.uint8), cv2.COLORMAP_JET)
            msg = self.bridge.cv2_to_imgmsg(depth_viz, "bgr8")
            self.depth_viz_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"深度可视化发布失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭中...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()