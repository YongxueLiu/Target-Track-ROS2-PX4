#!/bin/bash
# =========================================================
# PX4 + Gazebo + ROS2 无人机仿真环境自动化启动脚本
# 严格启动顺序：Gazebo → PX4 → ROS-GZ Bridge → MicroXRCEAgent → QGC
# 使用 gnome-terminal 在不同标签页启动
# =========================================================

set -e  # 遇到错误时退出

# === 配置路径（根据你的环境调整） ===
GZ_WORLD_PATH="$HOME/.simulation-gazebo/worlds"
PX4_PATH="$HOME/PX4-Autopilot"
PYTHON_ENV_PATH="$HOME/myenv/bin/activate"
QGC_PATH="$HOME/bin/QGroundControl-x86_64.AppImage"
ROS_DISTRO="rolling"  # 或 iron, jazzy

# 仿真参数
WORLD_NAME="actor"
PX4_MODEL="gz_x500_depth"
CAMERA_TOPICS="/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image"

# === 依赖检查 ===
echo "🔍 检查依赖..."
command -v gnome-terminal >/dev/null 2>&1 || { echo "错误: 需要安装 gnome-terminal"; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "错误: 需要安装 ROS2"; exit 1; }
[[ -d "$PX4_PATH" ]] || { echo "错误: PX4 路径不存在: $PX4_PATH"; exit 1; }
[[ -d "$GZ_WORLD_PATH" ]] || { echo "错误: Gazebo 路径不存在: $GZ_WORLD_PATH"; exit 1; }

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash

# === 颜色输出 ===
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# === 启动函数 ===
start_gazebo() {
    echo -e "${YELLOW}=== 1️⃣ 启动 Gazebo 仿真 ===${NC}"
    gnome-terminal --tab --title="🗺️ Gazebo ($WORLD_NAME)" -- bash -c "
        cd '$GZ_WORLD_PATH';
        echo '🚀 启动 Gazebo 世界: $WORLD_NAME';
        echo '等待 Gazebo 完全启动...';
        python3 simulation-gazebo --world $WORLD_NAME;
        echo 'Gazebo 已停止，按 Enter 关闭窗口...';
        read;
    " &
    GZ_PID=$!
    echo -e "${GREEN}Gazebo 启动中 (PID: $GZ_PID)${NC}"
    
    # 等待 Gazebo 启动
    echo -e "${BLUE}等待 Gazebo 启动完成 (10秒)...${NC}"
    sleep 10
}

start_px4() {
    echo -e "${YELLOW}=== 2️⃣ 启动 PX4 SITL ===${NC}"
    gnome-terminal --tab --title="🛫 PX4 SITL ($PX4_MODEL)" -- bash -c "
        cd '$PX4_PATH';
        echo '🛫 启动 PX4 SITL: $PX4_MODEL';
        echo '确保 Gazebo 已运行...';
        export PX4_SIM_MODEL=$PX4_MODEL;
        make px4_sitl $PX4_MODEL;
        echo 'PX4 已停止，按 Enter 关闭窗口...';
        read;
    " &
    PX4_PID=$!
    echo -e "${GREEN}PX4 启动中 (PID: $PX4_PID)${NC}"
    
    # 等待 PX4 连接 Gazebo
    echo -e "${BLUE}等待 PX4 连接 Gazebo (15秒)...${NC}"
    sleep 15
}

start_ros_bridge() {
    echo -e "${YELLOW}=== 3️⃣ 启动 ROS2-Gazebo 桥接 ===${NC}"
    gnome-terminal --tab --title="🔄 ROS-GZ Bridge" -- bash -c "
        source /opt/ros/$ROS_DISTRO/setup.bash;
        cd '$PX4_PATH';
        echo '🔄 启动 ROS2 <-> Gazebo 图像桥接';
        echo '桥接话题: /rgb_camera 和 /depth_camera';
        ros2 run ros_gz_bridge parameter_bridge $CAMERA_TOPICS;
        echo '桥接已停止，按 Enter 关闭窗口...';
        read;
    " &
    BRIDGE_PID=$!
    echo -e "${GREEN}ROS-GZ Bridge 启动中 (PID: $BRIDGE_PID)${NC}"
    
    # 验证桥接
    sleep 3
    if ros2 topic list 2>/dev/null | grep -q "/rgb_camera"; then
        echo -e "${GREEN}✓ ROS2 桥接成功！${NC}"
        echo -e "${BLUE}可测试: ros2 topic echo /rgb_camera --once${NC}"
    else
        echo -e "${RED}✗ 桥接可能失败，检查 Gazebo/PX4 状态${NC}"
    fi
    sleep 2
}

start_micro_agent() {
    echo -e "${YELLOW}=== 4️⃣ 启动 MicroXRCEAgent ===${NC}"
    gnome-terminal --tab --title="📡 MicroXRCEAgent" -- bash -c "
        cd '$PX4_PATH';
        echo '📡 启动 MicroXRCEAgent (UDP port 8888)';
        echo '等待 PX4 客户端连接...';
        MicroXRCEAgent udp4 -p 8888;
        echo 'Agent 已停止，按 Enter 关闭窗口...';
        read;
    " &
    AGENT_PID=$!
    echo -e "${GREEN}MicroXRCEAgent 启动中 (PID: $AGENT_PID)${NC}"
    sleep 5
}

start_qgroundcontrol() {
    if [[ -f "$QGC_PATH" ]]; then
        echo -e "${YELLOW}=== 5️⃣ 启动 QGroundControl ===${NC}"
        gnome-terminal --tab --title="🛰️ QGroundControl" -- bash -c "
            echo '🛰️ 启动 QGroundControl 地面站';
            echo '连接到 PX4 SITL (UDP 14550)...';
            '$QGC_PATH';
            echo 'QGC 已关闭';
        " &
        QGC_PID=$!
        echo -e "${GREEN}QGroundControl 启动中 (PID: $QGC_PID)${NC}"
    else
        echo -e "${RED}⚠️ QGroundControl 未找到: $QGC_PATH${NC}"
        echo -e "${YELLOW}请手动启动: $QGC_PATH${NC}"
    fi
    sleep 2
}

start_python_env() {
    if [[ -f "$PYTHON_ENV_PATH" ]]; then
        echo -e "${YELLOW}=== 6️⃣ 激活 Python 环境 ===${NC}"
        gnome-terminal --tab --title="🐍 Python Env" -- bash -c "
            source '$PYTHON_ENV_PATH';
            source /opt/ros/$ROS_DISTRO/setup.bash;
            echo '🐍 Python 环境已激活 (myenv)';
            echo '可运行 ROS2 节点或自定义脚本';
            echo '示例: ros2 run your_pkg your_node';
            echo '或: python3 your_script.py';
            exec bash;
        " &
        ENV_PID=$!
        echo -e "${GREEN}Python 环境窗口已打开 (PID: $ENV_PID)${NC}"
        echo -e "${YELLOW}在此窗口运行你的 VO/VIO 节点${NC}"
    else
        echo -e "${RED}⚠️ Python 环境未找到: $PYTHON_ENV_PATH${NC}"
    fi
}

# === 清理函数 ===
cleanup() {
    echo -e "\n${RED}=== 停止仿真环境 ===${NC}"
    echo "终止进程: GZ($GZ_PID), PX4($PX4_PID), Bridge($BRIDGE_PID), Agent($AGENT_PID)"
    kill $GZ_PID $PX4_PID $BRIDGE_PID $AGENT_PID $QGC_PID $ENV_PID 2>/dev/null || true
    pkill -f "QGroundControl\|MicroXRCEAgent\|ros_gz_bridge\|simulation-gazebo" 2>/dev/null || true
    echo -e "${GREEN}清理完成${NC}"
    exit 0
}

# 捕获 Ctrl+C
trap cleanup SIGINT SIGTERM

# === 主启动流程 ===
echo -e "${GREEN}🚁 PX4 + Gazebo + ROS2 仿真环境启动${NC}"
echo -e "${BLUE}ROS 版本: $ROS_DISTRO | 世界: $WORLD_NAME | 模型: $PX4_MODEL${NC}"
echo "================================================================"

# 1. 启动 Gazebo
start_gazebo

# 2. 启动 PX4
start_px4

# 3. 启动 ROS-GZ Bridge
start_ros_bridge

# 4. 启动 MicroXRCEAgent
start_micro_agent

# 5. 启动 QGroundControl
start_qgroundcontrol

# 6. 启动 Python 环境
#start_python_env

# === 验证和提示 ===
echo -e "\n${GREEN}✅ 所有组件启动完成！${NC}"
echo -e "${YELLOW}=== 验证检查 ===${NC}"
echo "1. 检查 ROS2 话题:"
echo "   ros2 topic list | grep -E '(rgb|depth|camera|fmu)'"
echo "2. 测试相机数据:"
echo "   ros2 topic hz /rgb_camera"
echo "   ros2 topic echo /depth_camera --once"
echo "3. 检查 PX4 连接:"
echo "   ros2 topic list | grep fmu"
echo "4. QGC 连接: UDP 14550 (默认)"
echo "5. Python 环境: 运行你的 VO/VIO 节点"

echo -e "\n${RED}停止脚本: Ctrl+C 或关闭所有终端${NC}"
echo -e "${YELLOW}日志查看: ~/.ros/log/ 最新的日志文件${NC}"

# 保持脚本运行，监控进程
echo -e "\n${BLUE}监控进程状态 (按 Ctrl+C 停止)...${NC}"
wait
