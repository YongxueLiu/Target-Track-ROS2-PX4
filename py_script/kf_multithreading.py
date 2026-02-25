#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进版本：使用线程 + 队列解耦 YOLO 推理和卡尔曼滤波（BUG修复版）
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
import threading
from queue import Queue
from typing import Tuple, Optional
from dataclasses import dataclass

try:
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
except Exception:
    VehicleAttitude = None
    VehicleLocalPosition = None

from ultralytics import YOLO

# ============================================================================
# 卡尔曼滤波器实现
# ============================================================================

class KalmanFilter3D:
    """6维卡尔曼滤波器：联合估计 3D 位置和速度"""
    
    def __init__(self, 
                 process_noise_pos: float = 0.01,
                 process_noise_vel: float = 0.1,
                 measurement_noise: float = 0.5):
        self.dim_state = 6
        self.dim_meas = 3
        
        self.x = np.zeros((self.dim_state, 1))
        self.P = np.eye(self.dim_state) * 1.0
        
        self.Q = np.eye(self.dim_state)
        self.Q[:3, :3] *= process_noise_pos
        self.Q[3:6, 3:6] *= process_noise_vel
        
        self.R = np.eye(self.dim_meas) * (measurement_noise ** 2)
        
        self.H = np.zeros((self.dim_meas, self.dim_state))
        self.H[:3, :3] = np.eye(3)
        
        self.initialized = False
    
    def predict(self, dt: float):
        """预测步骤"""
        if dt <= 0:
            return
        
        F = np.eye(self.dim_state)
        F[:3, 3:6] = dt * np.eye(3)
        
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, z: np.ndarray):
        """更新步骤"""
        if z.ndim == 1:
            z = z.reshape((self.dim_meas, 1))
        
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        I_KH = np.eye(self.dim_state) - K @ self.H
        self.P = I_KH @ self.P
        
        self.initialized = True
    
    def initialize(self, z: np.ndarray, v: Optional[np.ndarray] = None):
        """初始化滤波器"""
        if z.ndim == 1:
            z = z.reshape((3, 1))
        
        self.x[:3, 0] = z[:, 0]
        if v is not None:
            self.x[3:6, 0] = v[:, 0] if v.ndim == 2 else v
        else:
            self.x[3:6, 0] = 0.0
        
        self.P = np.eye(self.dim_state) * 0.1
        self.initialized = True
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取位置和速度估计"""
        return self.x[:3, 0], self.x[3:6, 0]


# ============================================================================
# 数据容器
# ============================================================================

@dataclass
class DetectionResult:
    """YOLO检测结果"""
    timestamp: float
    image: np.ndarray
    boxes: list
    depth_image: Optional[np.ndarray] = None


@dataclass
class FilteredState:
    """滤波后的状态"""
    timestamp: float
    position_enu: np.ndarray
    velocity_enu: np.ndarray
    measurement_enu: np.ndarray
    is_valid: bool


# ============================================================================
# YOLO推理线程
# ============================================================================

class YOLOInferenceThread(threading.Thread):
    """独立线程：YOLO推理"""
    
    def __init__(self, model_path: str, target_class_name: str, 
                 input_queue: Queue, output_queue: Queue, 
                 node_logger=None):
        super().__init__(daemon=True)
        self.model = YOLO(model_path)
        self.target_class_name = target_class_name
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.logger = node_logger
        self.running = True
        
    def run(self):
        while self.running:
            try:
                item = self.input_queue.get(timeout=1.0)
                if item is None:
                    break
                
                timestamp, rgb_image, depth_image = item
                
                results = self.model.predict(source=rgb_image, conf=0.5, verbose=False)
                detections = results[0]
                
                boxes = []
                for box in detections.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls]
                    
                    if label != self.target_class_name:
                        continue
                    
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    boxes.append((x1, y1, x2, y2, conf, label))
                
                result = DetectionResult(
                    timestamp=timestamp,
                    image=rgb_image,
                    boxes=boxes,
                    depth_image=depth_image
                )
                self.output_queue.put(result)
                
            except Exception as e:
                if self.logger:
                    self.logger.error(f"YOLO推理线程异常: {e}")
    
    def stop(self):
        self.running = False


# ============================================================================
# 卡尔曼滤波线程
# ============================================================================

class KalmanFilterThread(threading.Thread):
    """独立线程：卡尔曼滤波和坐标变换"""
    
    def __init__(self, input_queue: Queue, output_queue: Queue,
                 transformer, node_logger=None):
        super().__init__(daemon=True)
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.transformer = transformer
        self.logger = node_logger
        self.running = True
        
        self.latest_att_q = None
        self.latest_pos_ned = None
        self.kf = KalmanFilter3D(
            process_noise_pos=0.01,
            process_noise_vel=0.05,
            measurement_noise=0.3
        )
        self.last_measurement_time = None
        self.lock = threading.Lock()  # 线程锁，保护共享状态
    
    def update_vehicle_state(self, att_q: np.ndarray, pos_ned: np.ndarray):
        """由主线程调用，更新车辆状态"""
        with self.lock:
            if att_q is not None:
                self.latest_att_q = att_q.copy()
            if pos_ned is not None:
                self.latest_pos_ned = pos_ned.copy()
    
    def run(self):
        while self.running:
            try:
                result = self.input_queue.get(timeout=1.0)
                if result is None:
                    break
                
                detection_result: DetectionResult = result
                
                # 线程安全地获取车辆状态
                with self.lock:
                    att_q = self.latest_att_q
                    pos_ned = self.latest_pos_ned
                
                if att_q is None or pos_ned is None:
                    continue
                
                if not detection_result.boxes or detection_result.depth_image is None:
                    continue
                
                for x1, y1, x2, y2, conf, label in detection_result.boxes:
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    if (0 <= cy < detection_result.depth_image.shape[0] and 
                        0 <= cx < detection_result.depth_image.shape[1]):
                        z = float(detection_result.depth_image[cy, cx])
                    else:
                        continue
                    
                    if np.isnan(z) or z <= 0.01 or z > 50.0:
                        continue
                    
                    # 坐标变换
                    p_cam = self.transformer.pixel_depth_to_camera_xyz(cx, cy, z)
                    p_body = self.transformer.camera_to_body_frd(p_cam)
                    v_ned = self.transformer.body_to_ned(p_body, att_q)
                    p_target_ned = pos_ned + v_ned
                    p_target_enu = self.transformer.ned_to_enu(p_target_ned)
                    
                    # 卡尔曼滤波
                    now_time = time.time()
                    if self.last_measurement_time is None:
                        self.kf.initialize(p_target_enu)
                        self.last_measurement_time = now_time
                    else:
                        #dt = now_time - self.last_measurement_time
                        dt = detection_result.timestamp - self.last_measurement_time
                        self.kf.predict(dt)
                        self.kf.update(p_target_enu)
                        self.last_measurement_time = now_time
                    
                    pos_filtered, vel_filtered = self.kf.get_state()
                    
                    filtered_state = FilteredState(
                        timestamp=detection_result.timestamp,
                        position_enu=pos_filtered,
                        velocity_enu=vel_filtered,
                        measurement_enu=p_target_enu,
                        is_valid=True
                    )
                    self.output_queue.put(filtered_state)
                    
            except Exception as e:
                if self.logger:
                    self.logger.error(f"卡尔曼滤波线程异常: {e}")
    
    def stop(self):
        self.running = False


# ============================================================================
# 坐标转换器
# ============================================================================

class CoordinateTransformer:
    """完成坐标变换链"""
    
    def __init__(self, cam_pose_in_body: Tuple[float, float, float, float, float, float],
                 camera_intrinsics: dict):
        self.cam_pose_in_body = np.array(cam_pose_in_body)
        self.K = camera_intrinsics
        r, p, y = self.cam_pose_in_body[3:]
        self.R_cam_to_body = self._rpy_to_rotmat(r, p, y)
        self.t_cam_to_body = self.cam_pose_in_body[:3]

    @staticmethod
    def _rpy_to_rotmat(roll, pitch, yaw) -> np.ndarray:
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz.dot(Ry).dot(Rx)

    def pixel_depth_to_camera_xyz(self, u: int, v: int, z: float) -> np.ndarray:
        fx, fy = self.K["fx"], self.K["fy"]
        cx, cy = self.K["cx"], self.K["cy"]
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        return np.array([x_cam, y_cam, z])

    def camera_to_body_frd(self, p_cam: np.ndarray) -> np.ndarray:
        p_body_axes = self.R_cam_to_body.dot(p_cam) + self.t_cam_to_body
        x_cam, y_cam, z_cam = p_body_axes
        return np.array([z_cam, x_cam, y_cam])

    def body_to_ned(self, v_body: np.ndarray, q_body_to_ned: np.ndarray) -> np.ndarray:
        """四元数旋转"""
        qv = np.concatenate(([0.0], v_body))
        q_conj = np.array([q_body_to_ned[0], -q_body_to_ned[1], 
                          -q_body_to_ned[2], -q_body_to_ned[3]])
        
        def quat_mul(q1, q2):
            w1, x1, y1, z1 = q1
            w2, x2, y2, z2 = q2
            return np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ])
        
        return quat_mul(quat_mul(q_body_to_ned, qv), q_conj)[1:]

    @staticmethod
    def ned_to_enu(p_ned: np.ndarray) -> np.ndarray:
        n, e, d = p_ned
        return np.array([e, n, -d])


# ============================================================================
# QoS和配置
# ============================================================================

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

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
# 主节点
# ============================================================================

class YOLODepthFusionThreaded(Node):
    """改进版本：使用线程解耦处理流程"""
    
    def __init__(self,
                 camera_intrinsics: dict,
                 cam_pose_in_body=(0.12, 0.03, 0.242, 0.0, 0.0, 0.0),
                 model_path='yolov8n.pt',
                 target_class_name='person'):
        super().__init__('yolo_depth_fusion_threaded')

        self.bridge = CvBridge()
        self.target_class_name = target_class_name

        self.tf = CoordinateTransformer(cam_pose_in_body, camera_intrinsics)

        # 线程间通信队列
        self.yolo_input_queue = Queue(maxsize=5)
        self.kalman_input_queue = Queue(maxsize=10)
        self.output_queue = Queue(maxsize=10)

        # 启动处理线程
        self.yolo_thread = YOLOInferenceThread(
            model_path=model_path,
            target_class_name=target_class_name,
            input_queue=self.yolo_input_queue,
            output_queue=self.kalman_input_queue,
            node_logger=self.get_logger()
        )
        self.yolo_thread.start()

        self.kalman_thread = KalmanFilterThread(
            input_queue=self.kalman_input_queue,
            output_queue=self.output_queue,
            transformer=self.tf,
            node_logger=self.get_logger()
        )
        self.kalman_thread.start()

        # 状态
        self.latest_att_q = None
        self.latest_pos_ned = None
        self.depth_image = None

        # 订阅
        self.rgb_sub = self.create_subscription(Image, '/rgb_camera', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_camera', self.depth_callback, 10)

        if VehicleAttitude is not None:
            self.att_sub = self.create_subscription(
                VehicleAttitude, '/fmu/out/vehicle_attitude',
                self.vehicle_attitude_callback, PX4_QOS)

        if VehicleLocalPosition is not None:
            self.pos_sub = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                self.pos_cb, PX4_QOS)

        # 发布
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target/pose_enu', 10)
        self.target_twist_pub = self.create_publisher(TwistStamped, '/target/twist_enu', 10)

        # 发布定时器
        self.publish_timer = self.create_timer(0.01, self.publish_callback)

        self._last_log = {}
        self.get_logger().info(
            '✅ YOLO + Depth Fusion (Threaded with Kalman) 节点启动\n'
            '   - YOLO推理线程: 独立\n'
            '   - 卡尔曼滤波线程: 独立\n'
            '   - 发布线程: ROS2主线程 (100Hz)'
        )

    def throttle_log(self, interval_sec: float, msg: str, 
                    level: str = "info", tag: str = "default"):
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
        """RGB回调：快速解码和入队"""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB转换失败: {e}")
            return

        if self.depth_image is None:
            self.throttle_log(2.0, "等待深度图像...", "warn", "no_depth")
            return

        depth_h, depth_w = self.depth_image.shape[:2]
        rgb_resized = cv2.resize(rgb, (depth_w, depth_h))

        #timestamp = time.time()
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            self.yolo_input_queue.put_nowait((timestamp, rgb_resized, self.depth_image.copy()))
        except:
            self.throttle_log(1.0, "YOLO队列满，丢弃帧", "warn", "queue_full")

    def depth_callback(self, msg: Image):
        """深度图像回调"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth转换失败: {e}")

    def vehicle_attitude_callback(self, msg):
        """姿态四元数回调 - ✅ 修复版本"""
        try:
            q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
            self.latest_att_q = q / np.linalg.norm(q)
            
            # ✅ 修复：使用 None 检查，避免numpy数组条件判断歧义
            pos = self.latest_pos_ned if self.latest_pos_ned is not None else np.zeros(3)
            self.kalman_thread.update_vehicle_state(self.latest_att_q, pos)
            
        except Exception as e:
            self.get_logger().error(f"解析姿态失败: {e}")

    def pos_cb(self, msg):
        """位置回调 - ✅ 修复版本"""
        try:
            x, y, z = float(msg.x), float(msg.y), float(msg.z)
            self.latest_pos_ned = np.array([x, y, z])
            
            # ✅ 修复：使用 None 检查，避免numpy数组条件判断歧义
            att = self.latest_att_q if self.latest_att_q is not None else np.array([1, 0, 0, 0])
            self.kalman_thread.update_vehicle_state(att, self.latest_pos_ned)
            
        except Exception as e:
            self.get_logger().error(f"解析位置失败: {e}")

    def publish_callback(self):
        """发布定时器回调：消费输出队列"""
        try:
            filtered_state: FilteredState = self.output_queue.get_nowait()
        except:
            return

        now = self.get_clock().now().to_msg()

        # 发布PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "enu"
        pose_msg.pose.position.x = float(filtered_state.position_enu[0])
        pose_msg.pose.position.y = float(filtered_state.position_enu[1])
        pose_msg.pose.position.z = float(filtered_state.position_enu[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)

        # 发布TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now
        twist_msg.header.frame_id = "enu"
        twist_msg.twist.linear.x = float(filtered_state.velocity_enu[0])
        twist_msg.twist.linear.y = float(filtered_state.velocity_enu[1])
        twist_msg.twist.linear.z = float(filtered_state.velocity_enu[2])
        self.target_twist_pub.publish(twist_msg)

        self.throttle_log(
            1.0,
            f"目标 ENU (滤波): pos={filtered_state.position_enu} "
            f"vel={filtered_state.velocity_enu}",
            "info", "target_info"
        )

    def destroy_node(self):
        """优雅关闭"""
        self.get_logger().info("关闭处理线程...")
        self.yolo_thread.stop()
        self.kalman_thread.stop()
        self.yolo_thread.join(timeout=2.0)
        self.kalman_thread.join(timeout=2.0)
        super().destroy_node()


# ============================================================================
# main
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    intr = CAMERA_CONFIGS["depth_cam_640x480"]

    node = YOLODepthFusionThreaded(
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
