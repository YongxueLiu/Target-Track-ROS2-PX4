#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yolo_follow_node.py

功能：
- 订阅 RGB / Depth / PX4 VehicleAttitude / VehicleLocalPosition
- YOLOv8 检测人体（或任意类别）
- 深度融合 -> 计算摄像头坐标系下目标 3D 点（相机为 RDF：x=right,y=down,z=forward）
- 坐标变换：Camera(RDF) -> Body(FRD) -> NED -> ENU (ROS2)
- 发布目标在 ROS2 ENU 下的 PoseStamped 与 TwistStamped（供跟随节点）
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from collections import deque
from typing import Tuple, Optional

# 可能需要按实际系统调整消息导入（PX4 消息包名）
try:
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
except Exception:
    # 如果没有 px4_msgs 包，可用占位类或调整导入
    VehicleAttitude = None
    VehicleLocalPosition = None

# YOLO（ultralytics）
from ultralytics import YOLO

# -----------------------------
# QoS 配置
# -----------------------------
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# -----------------------------
# 工具函数：四元数与向量旋转
# -----------------------------
def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """q = [w,x,y,z]"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, q = [w,x,y,z]"""
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def rotate_vector_by_quaternion(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotate vector v (3,) by quaternion q (w,x,y,z).
    Returns rotated vector.
    """
    qv = np.concatenate(([0.0], v))
    return quat_mul(quat_mul(q, qv), quat_conjugate(q))[1:]


# -----------------------------
# 坐标系转换说明（内部实现）
# Camera coordinate: RDF (x_right, y_down, z_forward)
# Body coordinate:     FRD (x_forward, y_right, z_down)
# Mapping Camera(RDF) -> Body(FRD):
#   x_body = z_cam
#   y_body = x_cam
#   z_body = y_cam
#
# Body (FRD) rotated by vehicle attitude quaternion -> NED (当地导航系)
# Then NED -> ROS ENU mapping:
#   [north, east, down] -> [east, north, up] = [east, north, -down]
# -----------------------------


# -----------------------------
# Intrinsics 配置（示例：你给的两个内参）
# 你可以把这块改成读取 YAML/参数服务器
# -----------------------------
CAMERA_CONFIGS = {
    "depth_cam_640x480": {
        "width": 640, "height": 480,
        "fx": 432.49603271484375, "fy": 432.49606132507324,
        "cx": 320.0, "cy": 240.0
    },
    "rgb_cam_1920x1080": {
        "width": 1920, "height": 1080,
        "fx": 1397.2235870361328, "fy": 1397.2235298156738,
        "cx": 960.0, "cy": 540.0
    }
}


# -----------------------------
# Transformer 类：完成坐标变换链
# -----------------------------
class CoordinateTransformer:
    def __init__(self,
                 cam_pose_in_body: Tuple[float,float,float,float,float,float],
                 camera_intrinsics: dict):
        """
        cam_pose_in_body: (x,y,z, roll, pitch, yaw) 表示 camera_link 相对于 body(base_link) 的 pose
        camera_intrinsics: dict with fx, fy, cx, cy, width, height
        """
        self.cam_pose_in_body = np.array(cam_pose_in_body)  # tx,ty,tz, r,p,y (radians)
        self.K = camera_intrinsics

        # build rotation matrix from cam->body (if any rotation) using rpy
        r,p,y = self.cam_pose_in_body[3:]
        self.R_cam_to_body = self._rpy_to_rotmat(r,p,y)
        self.t_cam_to_body = self.cam_pose_in_body[:3]

    @staticmethod
    def _rpy_to_rotmat(roll, pitch, yaw) -> np.ndarray:
        """ZYX order yaw-pitch-roll -> returns 3x3 rotation matrix"""
        cr = np.cos(roll); sr = np.sin(roll)
        cp = np.cos(pitch); sp = np.sin(pitch)
        cy = np.cos(yaw); sy = np.sin(yaw)
        Rz = np.array([[cy, -sy, 0],[sy, cy, 0],[0,0,1]])
        Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        return Rz.dot(Ry).dot(Rx)

    def pixel_depth_to_camera_xyz(self, u:int, v:int, z:float) -> np.ndarray:
        """
        u,v: pixel coords (col,row) relative to image origin (0,0) top-left
        z: depth in meters along camera forward (camera z)
        Camera coord is RDF: x_right, y_down, z_forward
        Standard pinhole:
           x_cam = (u - cx) * z / fx
           y_cam = (v - cy) * z / fy
           z_cam = z
        """
        fx = self.K["fx"]; fy = self.K["fy"]; cx=self.K["cx"]; cy=self.K["cy"]
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        z_cam = z
        return np.array([x_cam, y_cam, z_cam])

    def camera_to_body_frd(self, p_cam: np.ndarray) -> np.ndarray:
        """
        Map camera RDF -> body FRD (no rotation except fixed mount)
        First apply camera rigid transform (if camera rotated/translated wrt body),
        then reorder axes to FRD conventions.
        """
        # apply camera mount rotation/translation to get vector in body coordinates (but still in cam-axis convention)
        # since p_cam is expressed in camera frame, R_cam_to_body rotates camera vector into body frame axes.
        p_body_axes = self.R_cam_to_body.dot(p_cam) + self.t_cam_to_body  # careful: this treats p_cam as point in cam frame -> point in body frame
        # But note: camera axis is RDF, body axis we want FRD. We want to reorder components:
        # camera: [x_right, y_down, z_forward]
        # body (FRD): [x_forward, y_right, z_down]  => mapping:
        x_cam, y_cam, z_cam = p_body_axes  # still in cam axis orientation after R
        x_body = z_cam
        y_body = x_cam
        z_body = y_cam
        return np.array([x_body, y_body, z_body])

    def body_to_ned(self, v_body: np.ndarray, q_body_to_ned: np.ndarray) -> np.ndarray:
        """
        Rotate vector in body (FRD) into NED using quaternion q (w,x,y,z)
        q_body_to_ned should rotate a vector from body frame to NED frame.
        """
        # rotate as vector (no translation)
        return rotate_vector_by_quaternion(q_body_to_ned, v_body)

    @staticmethod
    def ned_to_enu(p_ned: np.ndarray) -> np.ndarray:
        """Map point from NED [N, E, D] -> ENU [E, N, U]"""
        n, e, d = p_ned
        return np.array([e, n, -d])


# -----------------------------
# 主节点：YOLO + Depth + Transform + Publish
# -----------------------------
class YOLODepthFusion(Node):
    def __init__(self,
                 camera_intrinsics: dict,
                 cam_pose_in_body=(0.12, 0.03, 0.242, 0.0, 0.0, 0.0),
                 model_path='yolov8n.pt',
                 target_class_name='person'):
        super().__init__('yolo_depth_fusion_modular')

        # init components
        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.target_class_name = target_class_name

        # state
        self.rgb_image = None
        self.depth_image = None
        self.latest_att_q = None          # quaternion (w,x,y,z) body->ned
        self.latest_pos_ned = None        # body position in NED (north,east,down)
        self.prev_positions_enu = deque(maxlen=10)  # for velocity est
        self.prev_times = deque(maxlen=10)

        # transformer
        self.tf = CoordinateTransformer(cam_pose_in_body, camera_intrinsics)

        # subs
        self.rgb_sub = self.create_subscription(Image, '/rgb_camera', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_camera', self.depth_callback, 10)

        # PX4 topics (attitude + local position) with QoS
        if VehicleAttitude is not None:
            self.att_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, PX4_QOS)
        else:
            self.get_logger().warning("px4_msgs.VehicleAttitude not available in imports. Attitude input disabled.")

        if VehicleLocalPosition is not None:
            self.pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_cb, PX4_QOS)
        else:
            self.get_logger().warning("px4_msgs.VehicleLocalPosition not available in imports. Position input disabled.")

        # publishers for downstream follower (PoseStamped + TwistStamped)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target/pose_enu', 10)
        self.target_twist_pub = self.create_publisher(TwistStamped, '/target/twist_enu', 10)

        # throttled logging state
        self._last_log = {}

        self.get_logger().info('✅ YOLO + Depth Fusion 节点启动')

    def throttle_log(self, interval_sec: float, msg: str, level: str = "info", tag: str = "default"):
        now = time.time()
        if tag not in self._last_log or (now - self._last_log[tag]) > interval_sec:
            if level == "info":
                self.get_logger().info(msg)
            elif level == "warn":
                self.get_logger().warning(msg)
            elif level == "error":
                self.get_logger().error(msg)
            self._last_log[tag] = now

    # ---------------- image callbacks ----------------
    def rgb_callback(self, msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB 转换失败: {e}")
            return

        if self.depth_image is None:
            self.throttle_log(2.0, "等待深度图像以完成深度融合...", "warn", "no_depth")
            return

        # resize RGB 到 depth 大小以便像素对齐（如果你已经确保两者尺寸相同，可跳过）
        depth_h, depth_w = self.depth_image.shape[:2]
        rgb_resized = cv2.resize(rgb, (depth_w, depth_h))

        # YOLO 检测 - 使用 predict 返回列表，取第 0 帧
        results = self.model.predict(source=rgb_resized, conf=0.6, verbose=False)
        detections = results[0]  # ultralytics result

        img_vis = rgb_resized.copy()
        self.throttle_log(1.0, f"收到 RGB 帧，检测到 {len(detections.boxes)} 个目标", "info", "rgb_frame")

        for box in detections.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls]
            if label != self.target_class_name:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # extract depth z at centroid
            if 0 <= cy < self.depth_image.shape[0] and 0 <= cx < self.depth_image.shape[1]:
                z = float(self.depth_image[cy, cx])
            else:
                z = np.nan

            if np.isnan(z) or z <= 0.01 or z > 50.0:
                self.throttle_log(1.0, f"无效深度值 ({cx},{cy}) -> {z}", "warn", "bad_depth")
                continue

            # 1) pixel -> camera coords (RDF)
            p_cam = self.tf.pixel_depth_to_camera_xyz(cx, cy, z)  # [x_right, y_down, z_forward]

            # 2) camera -> body (FRD)
            p_body = self.tf.camera_to_body_frd(p_cam)  # [x_forward, y_right, z_down] in body frame coordinates (meters)

            # 3) rotate body vector into NED using latest attitude quaternion (body->ned)
            if self.latest_att_q is None:
                self.throttle_log(1.0, "等待 vehicle_attitude 四元数以计算目标在 NED 下的位置...", "warn", "no_att")
                continue

            # vector from body origin to target expressed in NED:
            v_ned = self.tf.body_to_ned(p_body, self.latest_att_q)  # this is offset vector in NED

            # 4) add vehicle body position in NED to get target absolute position in NED
            if self.latest_pos_ned is None:
                self.throttle_log(1.0, "等待 vehicle_local_position 以获得机体在 NED 的位置...", "warn", "no_pos")
                continue

            p_target_ned = self.latest_pos_ned + v_ned  # [north, east, down]
            p_target_enu = self.tf.ned_to_enu(p_target_ned)  # [east, north, up]

            # publish PoseStamped (ENU)
            now = self.get_clock().now().to_msg()
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = "enu"  # 约定下游使用 'enu'
            pose_msg.pose.position.x = float(p_target_enu[0])
            pose_msg.pose.position.y = float(p_target_enu[1])
            pose_msg.pose.position.z = float(p_target_enu[2])

            # orientation left identity (we only publish position); 可以改为指向目标的朝向
            pose_msg.pose.orientation.w = 1.0
            self.target_pose_pub.publish(pose_msg)

            # velocity estimation (简单差分)
            tnow = time.time()
            self.prev_positions_enu.append(np.array(p_target_enu))
            self.prev_times.append(tnow)
            vel = self.estimate_velocity_ols()

            twist_msg = TwistStamped()
            twist_msg.header.stamp = now
            twist_msg.header.frame_id = "enu"
            if vel is not None:
                twist_msg.twist.linear.x = float(vel[0])
                twist_msg.twist.linear.y = float(vel[1])
                twist_msg.twist.linear.z = float(vel[2])
            else:
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.linear.z = 0.0

            self.target_twist_pub.publish(twist_msg)

            # 可视化
            cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(img_vis, f"{label} {conf:.2f} z={z:.2f}m",
                        (x1, max(0, y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,255,0),2)
            cv2.circle(img_vis, (cx, cy), 4, (255,0,0), -1)

            # 额外日志
            #self.get_logger().info(f"目标 ENU pos: x={p_target_enu[0]:.3f}, y={p_target_enu[1]:.3f}, z={p_target_enu[2]:.3f}  vel: {vel if vel is not None else '[estimating]'}")
            self.throttle_log(1,f"目标 ENU pos: x={p_target_enu[0]:.3f}, y={p_target_enu[1]:.3f}, z={p_target_enu[2]:.3f}  vel: {vel if vel is not None else '[estimating]'}")
        # 显示窗口（调试）
        small = cv2.resize(img_vis, (img_vis.shape[1]//2, img_vis.shape[0]//2))
        cv2.imshow("YOLO+Depth", small)
        cv2.waitKey(1)

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth 转换失败: {e}")
            return

        # 打印平均深度，节流
        valid = self.depth_image[np.isfinite(self.depth_image)]
        if valid.size > 0:
            self.throttle_log(5.0, f"平均深度: {np.mean(valid):.3f} m", "info", "depth_info")
        else:
            self.throttle_log(5.0, "深度图无有效像素", "warn", "empty_depth")

    # ---------------- PX4 callbacks ----------------
    def vehicle_attitude_callback(self, msg):
        """
        期望消息包含 quaternion q: [w, x, y, z]
        并且该 quaternion 把 body 向量旋转到 NED（body -> ned）
        """
        try:
            q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])  # w,x,y,z
            self.latest_att_q = q / np.linalg.norm(q)
        except Exception as e:
            self.get_logger().error(f"解析 VehicleAttitude 失败: {e}")

    def pos_cb(self, msg):
        """
        期望 VehicleLocalPosition 中包含 position in NED: x (north), y (east), z (down)
        """
        try:
            # 这里字段名依实现而异，可能是 x, y, z 或 position[...]
            # 假设 msg.x, msg.y, msg.z
            # 如果你的消息字段不同，请修改这里。
            # 也有可能是 msg.x, msg.y, msg.z 是 body frame 的位置（相对于 home），请确认 PX4 消息定义
            x = float(msg.x)  # north
            y = float(msg.y)  # east
            z = float(msg.z)  # down
            self.latest_pos_ned = np.array([x, y, z])
        except Exception as e:
            self.get_logger().error(f"解析 VehicleLocalPosition 失败: {e}")

    # ---------------- velocity estimation ----------------
    def estimate_velocity(self) -> Optional[np.ndarray]:
        """
        基于 prev_positions_enu / prev_times 做差分，返回线速度（ENU）
        使用简单线性拟合 / 最小二乘可增加鲁棒性；当前使用最后两点差分
        """
        if len(self.prev_positions_enu) < 2:
            return None
        p1 = self.prev_positions_enu[-8]
        t1 = self.prev_times[-8]
        p2 = self.prev_positions_enu[-1]
        t2 = self.prev_times[-1]
        dt = t2 - t1
        if dt <= 1e-6:
            return None
        v = (p2 - p1) / dt
        return v
    

    def estimate_velocity_ols(self) -> Optional[np.ndarray]:
        """
        使用最小二乘法拟合匀速模型，估计 ENU 坐标系下的线速度。
        模型: p(t) = p(t0) + v * (t - t0)
        最小化: sum || p(ti) - [p(t0) + v*(ti - t0)] ||^2
        返回: v_enu (np.ndarray of shape (3,)) or None
        """
        if len(self.prev_positions_enu) < 2:
            return None

        # 转换为 numpy 数组以便计算
        positions = np.array(self.prev_positions_enu)  # shape: (N, 3)
        times = np.array(self.prev_times)              # shape: (N,)

        # 以第一个点为参考 (t0, p0)
        t0 = times[0]
        p0 = positions[0]

        dt = times - t0  # shape: (N,)
        dp = positions - p0  # shape: (N, 3)

        # 计算分母: sum(dt_i^2)
        denom = np.sum(dt * dt)
        if denom < 1e-9:
            return None  # 时间无变化，无法估计

        # 计算分子: sum(dt_i * dp_i) -> shape (3,)
        numer = np.sum(dt[:, None] * dp, axis=0)  # 等价于 for each dim: sum(dt_i * dp_i[dim])

        # 速度估计
        velocity = numer / denom  # shape: (3,)
        return velocity

# -----------------------------
# main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)

    # 选择内参（按实际 depth 相机选择）
    intr = CAMERA_CONFIGS["depth_cam_640x480"]

    node = YOLODepthFusion(camera_intrinsics=intr,
                           cam_pose_in_body=(0.12, 0.03, 0.242, 0.0, 0.0, 0.0),
                           model_path='yolov8n.pt',
                           target_class_name='person')

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
