# MicroTwin-Arm 数字化孪生系统项目方案

## 1. 项目概述
**MicroTwin-Arm** 是一个结合了物理实体与数字仿真的微型机械臂系统。通过 ROS 2 Humble 环境下的 RViz2 可视化界面，实现对实物机械臂的实时控制与姿态同步。

## 2. 硬件系统架构

### 2.1 核心主控
*   **开发板**: ESP32 (建议使用 Fishbot DevKit 或标准 ESP32-WROOM)
*   **电源**: 12V 3A DC (总线舵机动力) + 5V (ESP32 逻辑)

### 2.2 舵机引脚与 ID 分配
| 关节名称 | 类型 | 控制 ID | ESP32 引脚 | 串口实例 | 作用 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Base** | PWM | N/A | IO 15 | LEDC PWM | 底部水平旋转轴 |
| **Joint 1** | 总线 (STS) | 15 | IO 17 | Serial2 | 机械臂大臂关节 |
| **Joint 2** | 总线 (STS) | 12 | IO 5 | Serial1 | 机械臂小臂关节 |
| **Joint 3** | 总线 (STS) | 14 | IO 14 | Serial0 | 机械臂末端旋转/夹爪 |

## 3. 软件系统架构 (ROS 2 Stack)

### 3.1 总体框架
```text
[ PC 端 ] (ROS 2 Humble)
    |-- joint_state_publisher_gui  # UI 界面控制滑动条
    |-- robot_state_publisher      # 解析 URDF 并发布 TF 坐标
    |-- RViz2                      # 3D 实时可视化
    |-- micro_ros_agent            # 通信网关 (Serial/UDP)
           ^
           | (USB Serial @ 115200)
           v
[ 下位机 ] (ESP32 + micro-ROS)
    |-- micro-ROS Node             # 订阅 /joint_states
    |-- Mapping Engine             # 弧度转舵机步数逻辑
    |-- Servo Driver               # 发送最终指令到 4 个 IO 口
```

## 4. 核心实施阶段

### 第一阶段：URDF 建模 (机器人描述)
*   **目标**: 编写 `arm.urdf` 文件，描述机械臂的物理连杆和关节。
*   **关键点**:
    *   设定关节限位 (`limit`): 防止 ROS 发出超过机械物理极限的指令。
    *   定义旋转轴 (`axis`): 确保 RViz 里的旋转方向与实物舵机一致。
    *   测量连杆长度: 准确的 `origin` 偏移量决定了仿真的真实性。

### 第二阶段：micro-ROS 固件开发 (ESP32)
*   **话题订阅**: 订阅 `sensor_msgs/msg/JointState`。
*   **映射逻辑**:
    *   **PWM 舵机**: `[-1.57, 1.57] rad` -> `[500, 2500] us`。
    *   **STS 舵机**: `[-3.14, 3.14] rad` -> `[0, 4095] step`。
*   **平滑处理**: 利用 STS 舵机的内置加速度 (`Acc`) 功能，使映射过程不生硬。

### 第三阶段：PC 端环境部署
*   **创建功能包**: `ros2 pkg create --build-type ament_cmake micro_twin_description`。
*   **编写 Launch**: 一键启动 `robot_state_publisher` 和 `rviz2`。

## 5. 项目亮点
1.  **独立 IO 控制**: 每个总线舵机拥有独立通道，极大提高了通信带宽和容错率。
2.  **数字化孪生**: 实现了“所见即所得”，RViz 里的模型动作会立即在实物上体现。
3.  **零点保护**: 开机自动引导至 2048/90 度安全位置，防止碰撞。

## 6. 未来扩展
1.  **MoveIt 2 集成**: 引入逆运动学（IK），点击空间坐标，机械臂自动规划路径。
2.  **视觉抓取**: 挂载摄像头，实现基于 OpenCV 的自动目标识别与抓取。
3.  **状态回传**: 读取 STS 舵机实际位置并反馈给 ROS 2，实现双向同步。

---
**制订日期**: 2026年4月11日
**状态**: 硬件已就绪，进入软件开发阶段。



# 使用的代码：

wget http://fishros.com/install -O fishros && . fishros


# 使用串口通讯协议运行microros-agent
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6

# UDPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6

# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6

# TCPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO tcp4 --port 8888 -v6

# CAN-FD micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO canfd --dev [YOUR CAN INTERFACE] -v6

