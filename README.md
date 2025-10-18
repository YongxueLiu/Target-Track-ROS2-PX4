太好了 ✅
下面是为你的项目量身打造的 **GitHub 完整版 README.md**（中英双语 + 自动目录 + 徽章 + 数学公式 + 结构清晰），可直接放入仓库根目录使用。
我已使用 GitHub Markdown 语法（含公式、目录、徽章、代码块、表格）优化排版。

---

```markdown
# 🧠 YOLO Depth Fusion for PX4 ROS 2

[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20|%20Iron%20|%20Jazzy-blue)](https://docs.ros.org/)
[![PX4](https://img.shields.io/badge/PX4-1.13%2B-success)](https://px4.io/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-yellow)](https://github.com/ultralytics/ultralytics)
[![License: MIT](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-orange)](https://www.python.org/)

---

**EN:** A ROS 2 node integrating **YOLOv8 object detection**, **depth fusion**, and **PX4 attitude/position data** to estimate and publish the 3D position and velocity of detected targets (e.g., people) in the **ENU frame**, enabling UAV autonomous following and perception.  
**中文简介：**  
这是一个结合 **YOLOv8 + 深度图像 + PX4 位姿信息** 的 ROS 2 节点，可实时检测目标（如人体），估计目标在 **ENU 坐标系** 下的三维坐标与速度，并发布给无人机跟随或避障系统使用。

---

## 📚 Table of Contents 目录
- [🚀 Features 功能特性](#-features-功能特性)
- [🧩 System Architecture 系统结构](#-system-architecture-系统结构)
- [⚙️ Dependencies 依赖环境](#️-dependencies-依赖环境)
- [🧠 Node Overview 节点说明](#-node-overview-节点说明)
- [🧭 Coordinate Frames 坐标系说明](#-coordinate-frames-坐标系说明)
- [🔧 Configuration 配置](#-configuration-配置)
- [▶️ Run 运行节点](#️-run-运行节点)
- [🧮 Velocity Estimation 速度估计原理](#-velocity-estimation-速度估计原理)
- [📸 Visualization 可视化调试](#-visualization-可视化调试)
- [🧑‍💻 Author / 作者](#-author--作者)
- [📜 License 许可协议](#-license-许可协议)

---

## 🚀 Features 功能特性

✅ **YOLOv8 实时检测**（默认检测 `person`）  
✅ **深度融合定位**：提取深度图中目标三维坐标  
✅ **多坐标系转换**：  
- Camera (RDF) → Body (FRD) → NED → ENU  
✅ **PX4 集成**：订阅 `VehicleAttitude` 与 `VehicleLocalPosition`  
✅ **目标信息发布**：发布 `/target/pose_enu` 与 `/target/twist_enu`  
✅ **平滑速度估计**：基于最小二乘（OLS）拟合匀速模型  

---

## 🧩 System Architecture 系统结构

```

RGB + Depth
│
▼
[ YOLOv8 Detector ]
│
▼
[ Depth Fusion → 3D Point (Camera frame) ]
│
▼
[ Coordinate Transform ]
RDF → FRD → NED → ENU
│
▼
[ PX4 Attitude + Position Fusion ]
│
▼
[ Publish PoseStamped + TwistStamped ]

````

---

## ⚙️ Dependencies 依赖环境

| Dependency | Version | Notes |
|-------------|----------|-------|
| Python | ≥ 3.8 |  |
| ROS 2 | Humble / Iron / Jazzy | 已在 Humble 测试通过 |
| px4_msgs | ≥ 1.13 | PX4 ROS 2 消息接口 |
| ultralytics | ≥ 8.0 | YOLOv8 模型 |
| OpenCV | ≥ 4.5 | 图像处理 |
| numpy | ≥ 1.23 | 数值计算 |
| cv_bridge | ROS 2 自带 | 图像消息转换 |

安装 YOLOv8：
```bash
pip install ultralytics
````

---

## 🧠 Node Overview 节点说明

| Topic                                | Type                            | Description   |
| ------------------------------------ | ------------------------------- | ------------- |
| `/rgb_camera`                        | `sensor_msgs/Image`             | RGB 图像输入      |
| `/depth_camera`                      | `sensor_msgs/Image`             | 深度图像输入        |
| `/fmu/out/vehicle_attitude`          | `px4_msgs/VehicleAttitude`      | PX4 姿态（四元数）   |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | PX4 本地位置（NED） |
| `/target/pose_enu`                   | `geometry_msgs/PoseStamped`     | 目标位置（ENU）     |
| `/target/twist_enu`                  | `geometry_msgs/TwistStamped`    | 目标速度（ENU）     |

QoS 策略（与 PX4 FAST RTPS 通信兼容）：

```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

---

## 🧭 Coordinate Frames 坐标系说明

| Frame            | Convention                       | Axis Definition |
| ---------------- | -------------------------------- | --------------- |
| **Camera (RDF)** | x = Right, y = Down, z = Forward |                 |
| **Body (FRD)**   | x = Forward, y = Right, z = Down |                 |
| **NED (PX4)**    | North–East–Down                  |                 |
| **ENU (ROS 2)**  | East–North–Up                    |                 |

变换链：

```
Camera (RDF) → Body (FRD) → NED → ENU
```

---

## 🔧 Configuration 配置

在节点文件 `yolo_follow_node.py` 中修改：

```python
CAMERA_CONFIGS = {
    "depth_cam_640x480": {
        "width": 640, "height": 480,
        "fx": 432.496, "fy": 432.496,
        "cx": 320.0, "cy": 240.0
    }
}

node = YOLODepthFusion(
    camera_intrinsics=CAMERA_CONFIGS["depth_cam_640x480"],
    cam_pose_in_body=(0.12, 0.03, 0.242, 0.0, 0.0, 0.0),
    model_path='yolov8n.pt',
    target_class_name='person'
)
```

切换检测类别：

```python
target_class_name = 'bottle'  # 修改为任意 YOLO 支持的类别
```

---

## ▶️ Run 运行节点

1️⃣ 启动 PX4 SITL 或连接真实飞控
2️⃣ 确认摄像头话题存在 `/rgb_camera` 与 `/depth_camera`
3️⃣ 运行：

```bash
ros2 run your_package_name yolo_follow_node.py
```

或直接运行：

```bash
python3 yolo_follow_node.py
```

---

## 🧮 Velocity Estimation 速度估计原理

使用最小二乘法拟合匀速模型：

[
p(t) = p(t_0) + v (t - t_0)
]

最优速度估计公式：

[
v = \frac{\sum_i (t_i - t_0)(p_i - p_0)}{\sum_i (t_i - t_0)^2}
]

相比简单差分法，该方法在噪声环境中更平滑且抗抖动。

Python 实现：

```python
def estimate_velocity_ols(self) -> Optional[np.ndarray]:
    if len(self.prev_positions_enu) < 2:
        return None
    positions = np.array(self.prev_positions_enu)
    times = np.array(self.prev_times)
    t0, p0 = times[0], positions[0]
    dt, dp = times - t0, positions - p0
    denom = np.sum(dt * dt)
    if denom < 1e-9:
        return None
    numer = np.sum(dt[:, None] * dp, axis=0)
    return numer / denom
```

---

## 📸 Visualization 可视化调试

运行时自动弹出可视化窗口，显示：

* 目标检测框与标签
* 深度信息（米）
* 目标中心像素坐标

可在窗口中直观验证检测与深度融合效果。

---

## 🧑‍💻 Author / 作者

**Yongxue Law (lyx)**
✈️ UAV Systems & PX4/ROS 2 Integration
📍 Jiangsu, China
💬 Research Focus: Autonomous UAV Perception, Sensor Fusion, Navigation

---

## 📜 License 许可协议

This project is released under the [MIT License](LICENSE).
本项目基于 [MIT 许可协议](LICENSE) 开源。

---

```

---

