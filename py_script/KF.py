#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yolo_follow_node.py - 集成卡尔曼滤波版本

功能：
- 订阅 RGB / Depth / PX4 VehicleAttitude / VehicleLocalPosition
- YOLOv8 检测人体（或任意类别）
- 深度融合 -> 计算摄像头坐标系下目标 3D 点（相机为 RDF：x=right,y=down,z=forward）
- 坐标变换：Camera(RDF) -> Body(FRD) -> NED -> ENU (ROS2)
- **卡尔曼滤波**：联合估计位置和速度（6维状态）
- 发布目标在 ROS2 ENU 下的 PoseStamped 与 TwistStamped（供跟随节点）

作者/Author: 刘永学/Liu Yongxue
邮箱/Email: 805110687@qq.com

改进：集成卡尔曼滤波器进行位置-速度联合估计
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from collections import deque
from typing import Tuple, Optional

try:
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
except Exception:
    VehicleAttitude = None
    VehicleLocalPosition = None

from ultralytics import YOLO

# ============================================================================
# 卡尔曼滤波器实现（线性常速度模型）
# ============================================================================

class KalmanFilter3D:
    """
    6维卡尔曼滤波器：联合估计 3D 位置和速度
    状态向量: [x, y, z, vx, vy, vz]^T
    测量值: [x, y, z]^T（来自深度融合）
    
    运动模型（恒速）: x(k+1) = x(k) + v(k) * dt
    """
    
    def __init__(self, 
                 process_noise_pos: float = 0.01,      # Q - 位置过程噪声
                 process_noise_vel: float = 0.1,       # Q - 速度过程噪声
                 measurement_noise: float = 0.5):      # R - 测量噪声
        """
        process_noise_pos: 位置过程噪声方差（m^2）
        process_noise_vel: 速度过程噪声方差（(m/s)^2）
        measurement_noise: 测量噪声标准差（m）
        """
        self.dim_state = 6  # [x, y, z, vx, vy, vz]
        self.dim_meas = 3   # [x, y, z]
        
        # 初始状态估计
        self.x = np.zeros((self.dim_state, 1))  # 状态向量
        self.P = np.eye(self.dim_state) * 1.0   # 状态协方差矩阵
        
        # 过程噪声协方差 Q (6x6)
        self.Q = np.eye(self.dim_state)
        self.Q[:3, :3] *= process_noise_pos      # 位置部分
        self.Q[3:6, 3:6] *= process_noise_vel    # 速度部分
        
        # 测量噪声协方差 R (3x3)
        self.R = np.eye(self.dim_meas) * (measurement_noise ** 2)
        
        # 测量矩阵 H (3x6): 只测量位置，不测量速度
        self.H = np.zeros((self.dim_meas, self.dim_state))
        self.H[:3, :3] = np.eye(3)
        
        self.last_time = None
        self.initialized = False
    
    def predict(self, dt: float):
        """
        预测步骤（时间更新）
        
        状态转移矩阵 F (恒速模型):
        [x']   [1 0 0 dt 0  0 ] [x]
        [y'] = [0 1 0 0  dt 0 ] [y]
        [z']   [0 0 1 0  0  dt] [z]
        [vx']  [0 0 0 1  0  0 ] [vx]
        [vy']  [0 0 0 0  1  0 ] [vy]
        [vz']  [0 0 0 0  0  1 ] [vz]
        """
        if dt <= 0:
            return
        
        # 构造状态转移矩阵 F
        F = np.eye(self.dim_state)
        F[:3, 3:6] = dt * np.eye(3)  # 位置 = 位置 + 速度 * dt
        
        # 预测状态: x = F @ x
        self.x = F @ self.x
        
        # 预测协方差: P = F @ P @ F^T + Q
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, z: np.ndarray):
        """
        更新步骤（测量更新）
        
        Args:
            z: 测量值 [x, y, z]^T (3x1 或 (3,) 数组)
        """
        if z.ndim == 1:
            z = z.reshape((self.dim_meas, 1))
        
        # 测量残差
        y = z - (self.H @ self.x)  # (3x1)
        
        # 残差协方差
        S = self.H @ self.P @ self.H.T + self.R  # (3x3)
        
        # 卡尔曼增益
        K = self.P @ self.H.T @ np.linalg.inv(S)  # (6x3)
        
        # 更新状态估计
        self.x = self.x + K @ y  # (6x1)
        
        # 更新协方差估计
        I_KH = np.eye(self.dim_state) - K @ self.H
        self.P = I_KH @ self.P
        
        self.initialized = True
    
    def initialize(self, z: np.ndarray, v: Optional[np.ndarray] = None):
        """
        初始化滤波器状态
        
        Args:
            z: 初始位置测量 [x, y, z]
            v: 初始速度估计（可选）
        """
        if z.ndim == 1:
            z = z.reshape((3, 1))
        
        self.x[:3, 0] = z[:, 0]  # 位置
        
        if v is not None:
            self.x[3:6, 0] = v[:, 0] if v.ndim == 2 else v
        else:
            self.x[3:6, 0] = 0.0  # 初始速度为零
        
        self.P = np.eye(self.dim_state) * 0.1
        self.initialized = True
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取当前状态估计
        
        Returns:
            position: [x, y, z] (3,)
            velocity: [vx, vy, vz] (3,)
        """
        return self.x[:3, 0], self.x[3:6, 0]
    
    def get_covariance(self) -> np.ndarray:
        """获取状态协方差矩阵"""
        return self.P.copy()


# ============================================================================
# QoS 配置
# ============================================================================

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# ============================================================================
# 工具函数：四元数与向量旋转
# ============================================================================

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """q = [w,x,y,z]"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, q = [w,x,y,z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def rotate_vector_by_quaternion(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v (3,) by quaternion q (w,x,y,z)."""
    qv = np.concatenate(([0.0], v))
    return quat_mul(quat_mul(q, qv), quat_conjugate(q))[1:]

# ============================================================================
# 摄像头配置
# ============================================================================

CAMERA_CONFIGS = {
    "depth_cam_640x480": {
        "width": 640, "height": 480,
        "fx": 432.49603271484375, "fy": 432.49606132507324,
        "cx": 320.0, "cy": 240.0
    },
    "rgb_cam_1920x1080": {
        "width": 1920, "height": 1080,
        "fx": 1397.2235870361328, "fy": 1397.2235298156738,
        "cx": 960.0, "cy": 540.0,
        "dist_coeffs": [-0.132, 0.045, 0.0001, -0.0002, 0.0]
    }
}

# ============================================================================
# 坐标转换器
# ============================================================================

class CoordinateTransformer:
    """完成 Camera(RDF) -> Body(FRD) -> NED -> ENU 的坐标变换链"""
    
    def __init__(self,
                 cam_pose_in_body: Tuple[float, float, float, float, float, float],
                 camera_intrinsics: dict):
        """
        cam_pose_in_body: (x,y,z, roll, pitch, yaw)
        camera_intrinsics: dict with fx, fy, cx, cy
        """
        self.cam_pose_in_body = np.array(cam_pose_in_body)
        self.K = camera_intrinsics

        r, p, y = self.cam_pose_in_body[3:]
        self.R_cam_to_body = self._rpy_to_rotmat(r, p, y)
        self.t_cam_to_body = self.cam_pose_in_body[:3]

    @staticmethod
    def _rpy_to_rotmat(roll, pitch, yaw) -> np.ndarray:
        """ZYX order: yaw-pitch-roll"""
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz.dot(Ry).dot(Rx)

    def pixel_depth_to_camera_xyz(self, u: int, v: int, z: float) -> np.ndarray:
        """Pixel + depth -> camera 3D coordinates (RDF)"""
        fx = self.K["fx"]
        fy = self.K["fy"]
        cx = self.K["cx"]
        cy = self.K["cy"]
        
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        z_cam = z
        return np.array([x_cam, y_cam, z_cam])

    def camera_to_body_frd(self, p_cam: np.ndarray) -> np.ndarray:
        """Map camera RDF -> body FRD"""
        p_body_axes = self.R_cam_to_body.dot(p_cam) + self.t_cam_to_body
        x_cam, y_cam, z_cam = p_body_axes
        x_body = z_cam
        y_body = x_cam
        z_body = y_cam
        return np.array([x_body, y_body, z_body])

    def body_to_ned(self, v_body: np.ndarray, q_body_to_ned: np.ndarray) -> np.ndarray:
        """Rotate body vector to NED"""
        return rotate_vector_by_quaternion(q_body_to_ned, v_body)

    @staticmethod
    def ned_to_enu(p_ned: np.ndarray) -> np.ndarray:
        """NED [N, E, D] -> ENU [E, N, U]"""
        n, e, d = p_ned
        return np.array([e, n, -d])

# ============================================================================
# 主节点：YOLO + Depth Fusion + Kalman Filter
# ============================================================================

class YOLODepthFusion(Node):
    def __init__(self,
                 camera_intrinsics: dict,
                 cam_pose_in_body=(0.12, 0.03, 0.242, 0.0, 0.0, 0.0),
                 model_path='yolov8n.pt',
                 target_class_name='person'):
        super().__init__('yolo_depth_fusion_kalman')

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.target_class_name = target_class_name

        # 状态变量
        self.rgb_image = None
        self.depth_image = None
        self.latest_att_q = None
        self.latest_pos_ned = None

        # **卡尔曼滤波器** - 6维（位置+速度）
        self.kf = KalmanFilter3D(
            process_noise_pos=0.01,      # 位置过程噪声
            process_noise_vel=0.05,      # 速度过程噪声
            measurement_noise=0.3        # 测量噪声
        )
        
        self.last_kf_time = None
        self.last_measurement_time = None

        # 坐标转换器
        self.tf = CoordinateTransformer(cam_pose_in_body, camera_intrinsics)

        # 订阅
        self.rgb_sub = self.create_subscription(Image, '/rgb_camera', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_camera', self.depth_callback, 10)

        if VehicleAttitude is not None:
            self.att_sub = self.create_subscription(
                VehicleAttitude, '/fmu/out/vehicle_attitude',
                self.vehicle_attitude_callback, PX4_QOS)
        else:
            self.get_logger().warning("px4_msgs.VehicleAttitude not available")

        if VehicleLocalPosition is not None:
            self.pos_sub = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                self.pos_cb, PX4_QOS)
        else:
            self.get_logger().warning("px4_msgs.VehicleLocalPosition not available")

        # 发布
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target/pose_enu', 10)
        self.target_twist_pub = self.create_publisher(TwistStamped, '/target/twist_enu', 10)

        self._last_log = {}
        self.get_logger().info('✅ YOLO + Depth Fusion (Kalman Filter) 节点启动')

    def throttle_log(self, interval_sec: float, msg: str, 
                    level: str = "info", tag: str = "default"):
        """节流日志输出"""
        now = time.time()
        if tag not in self._last_log or (now - self._last_log[tag]) > interval_sec:
            if level == "info":
                self.get_logger().info(msg)
            elif level == "warn":
                self.get_logger().warning(msg)
            elif level == "error":
                self.get_logger().error(msg)
            self._last_log[tag] = now

    def rgb_callback(self, msg: Image):
        """RGB 图像回调 + YOLO 检测"""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB 转换失败: {e}")
            return

        if self.depth_image is None:
            self.throttle_log(2.0, "等待深度图像...", "warn", "no_depth")
            return

        depth_h, depth_w = self.depth_image.shape[:2]
        rgb_resized = cv2.resize(rgb, (depth_w, depth_h))

        # YOLO 检测
        results = self.model.predict(source=rgb_resized, conf=0.6, verbose=False)
        detections = results[0]

        img_vis = rgb_resized.copy()
        self.throttle_log(1.0, f"检测到 {len(detections.boxes)} 个目标", "info", "rgb_frame")

        current_time = self.get_clock().now().to_msg()

        for box in detections.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls]
            
            if label != self.target_class_name:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # 获取深度值
            if 0 <= cy < self.depth_image.shape[0] and 0 <= cx < self.depth_image.shape[1]:
                z = float(self.depth_image[cy, cx])
            else:
                z = np.nan

            if np.isnan(z) or z <= 0.01 or z > 50.0:
                self.throttle_log(1.0, f"无效深度值 ({cx},{cy}) -> {z}", "warn", "bad_depth")
                continue

            # -------- 坐标变换链 --------
            p_cam = self.tf.pixel_depth_to_camera_xyz(cx, cy, z)
            p_body = self.tf.camera_to_body_frd(p_cam)

            if self.latest_att_q is None:
                self.throttle_log(1.0, "等待姿态四元数...", "warn", "no_att")
                continue

            v_ned = self.tf.body_to_ned(p_body, self.latest_att_q)

            if self.latest_pos_ned is None:
                self.throttle_log(1.0, "等待位置信息...", "warn", "no_pos")
                continue

            p_target_ned = self.latest_pos_ned + v_ned
            p_target_enu = self.tf.ned_to_enu(p_target_ned)

            # -------- 卡尔曼滤波 --------
            z_measurement = p_target_enu  # ENU 坐标下的测量值

            # 预测步骤：计算时间差
            now_time = time.time()
            if self.last_measurement_time is None:
                self.kf.initialize(z_measurement)
                self.last_measurement_time = now_time
            else:
                dt = now_time - self.last_measurement_time
                self.kf.predict(dt)
                self.kf.update(z_measurement)
                self.last_measurement_time = now_time

            # 获取滤波后的位置和速度
            pos_filtered, vel_filtered = self.kf.get_state()

            # -------- 发布消息 --------
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "enu"
            pose_msg.pose.position.x = float(pos_filtered[0])
            pose_msg.pose.position.y = float(pos_filtered[1])
            pose_msg.pose.position.z = float(pos_filtered[2])
            pose_msg.pose.orientation.w = 1.0
            self.target_pose_pub.publish(pose_msg)

            twist_msg = TwistStamped()
            twist_msg.header.stamp = current_time
            twist_msg.header.frame_id = "enu"
            twist_msg.twist.linear.x = float(vel_filtered[0])
            twist_msg.twist.linear.y = float(vel_filtered[1])
            twist_msg.twist.linear.z = float(vel_filtered[2])
            self.target_twist_pub.publish(twist_msg)

            # 可视化
            cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(img_vis, f"{label} {conf:.2f} z={z:.2f}m",
                        (x1, max(0, y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
            cv2.circle(img_vis, (cx, cy), 4, (255, 0, 0), -1)

            # 日志
            self.throttle_log(
                1.0,
                f"目标 ENU (滤波): pos=[{pos_filtered[0]:.2f}, {pos_filtered[1]:.2f}, {pos_filtered[2]:.2f}] "
                f"vel=[{vel_filtered[0]:.2f}, {vel_filtered[1]:.2f}, {vel_filtered[2]:.2f}] m/s",
                "info", "target_info"
            )

        # 显示
        small = cv2.resize(img_vis, (img_vis.shape[1]//2, img_vis.shape[0]//2))
        cv2.imshow("YOLO+Depth+Kalman", small)
        cv2.waitKey(1)

    def depth_callback(self, msg: Image):
        """深度图像回调"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth 转换失败: {e}")
            return

        valid = self.depth_image[np.isfinite(self.depth_image)]
        if valid.size > 0:
            self.throttle_log(5.0, f"平均深度: {np.mean(valid):.3f} m", "info", "depth_info")

    def vehicle_attitude_callback(self, msg):
        """姿态四元数回调"""
        try:
            q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
            self.latest_att_q = q / np.linalg.norm(q)
        except Exception as e:
            self.get_logger().error(f"解析 VehicleAttitude 失败: {e}")

    def pos_cb(self, msg):
        """位置回调"""
        try:
            x = float(msg.x)  # north
            y = float(msg.y)  # east
            z = float(msg.z)  # down
            self.latest_pos_ned = np.array([x, y, z])
        except Exception as e:
            self.get_logger().error(f"解析 VehicleLocalPosition 失败: {e}")

# ============================================================================
# main
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    intr = CAMERA_CONFIGS["depth_cam_640x480"]

    node = YOLODepthFusion(
        camera_intrinsics=intr,
        cam_pose_in_body=(0.12, 0.03, 0.242, 0.0, 0.0, 0.0),
        model_path='yolov8n.pt',
        target_class_name='person'
    )

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
`**
