# MicroTwin-Arm 数字化孪生系统项目方案

## 1. 项目概述
**MicroTwin-Arm** 是一个结合了物理实体与数字仿真的微型机械臂系统。通过 ROS 2 Humble 环境下的 RViz2 可视化界面，实现对实物机械臂的实时控制与姿态同步。

## 2. 硬件系统架构

### 2.1 核心主控
*   **开发板**: ESP32 (建议使用 Fishbot DevKit 或标准 ESP32-WROOM)
*   **电源**: 12V DC (总线舵机动力) + 5V (ESP32 逻辑)
*   **通信波特率**:
    *   **micro-ROS**: 921600 (High Speed Serial)
    *   **STS 总线舵机**: 1M (1,000,000)

### 2.2 舵机引脚与 ID 分配
| 关节名称 | 类型 | 控制 ID | ESP32 引脚 (TX/RX) | 串口实例 | 作用 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Joint 1 (Base)** | PWM | N/A | IO 15 | LEDC PWM | 底部水平旋转轴 |
| **Joint 2** | 总线 (STS) | 15 | TX 14 / RX 17 | Serial2 | 机械臂大臂关节 |
| **Joint 3** | 总线 (STS) | 12 | TX 4 / RX 5 | Serial1 | 机械臂小臂关节 |
| **Joint 4** | 总线 (STS) | 14 | TX 14 / RX 17 | Serial2 | 机械臂末端旋转/夹爪 |

> **注意**: `Joint 2` 和 `Joint 4` 共用 `Serial2` 总线。

## 3. 软件系统架构 (ROS 2 Stack)

### 3.1 总体框架
```text
[ PC 端 ] (ROS 2 Humble)
    |-- joint_state_publisher_gui  # UI 界面控制滑动条
    |-- robot_state_publisher      # 解析 URDF 并发布 TF 坐标
    |-- RViz2                      # 3D 实时可视化
    |-- micro_ros_agent            # 通信网关 (Serial @ 921600)
           ^
           | (USB Serial @ 921600)
           v
[ 下位机 ] (ESP32 + micro-ROS)
    |-- micro-ROS Node             # 订阅 /joint_states
    |-- Mapping Engine             # 弧度转舵机步数逻辑
    |-- Servo Driver               # 发送最终指令到舵机总线与 PWM 口
```

## 4. 核心实施阶段

### 第一阶段：URDF 建模 (机器人描述)
*   **功能包**: `ros2_ws/src/fishbot_description`
*   **模型文件**: `urdf/arm.urdf`
*   **关键点**:
    *   设定关节限位 (`limit`): 防止 ROS 发出超过机械物理极限的指令。
    *   定义旋转轴 (`axis`): 确保 RViz 里的旋转方向与实物舵机一致。
    *   网格文件: 使用 `meshes/` 目录下的 STL 文件。

### 第二阶段：micro-ROS 固件开发 (ESP32)
*   **框架**: PlatformIO + `micro_ros_platformio`
*   **源码**: `arm/src/main.cpp`
*   **核心功能**:
    *   **订阅话题**: `sensor_msgs/msg/JointState`。
    *   **映射逻辑**:
        *   **PWM 舵机**: `[-1.57, 1.57] rad` -> `[500, 2500] us` (反向)。
        *   **STS 舵机**: `[-3.14, 3.14] rad` -> `[0, 4095] step` (部分关节反向)。
    *   **运动平滑**: 启用 STS 舵机的 `WritePosEx` 加速度控制（Acc=30）。

### 第三阶段：运行与调试
*   **启动仿真**: 
    ```bash
    cd ros2_ws
    colcon build
    source install/setup.bash
    ros2 launch fishbot_description display_rviz2.launch.py
    ```
*   **启动 micro-ROS Agent**:
    ```bash
    # 使用 921600 高速波特率
    docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 921600 -v6
    ```

## 5. 项目进展
1.  **URDF 模型**: 已完成 4 轴机械臂描述，各关节限位与方向已校准。
2.  **底层驱动**: ESP32 固件已支持 micro-ROS 串口通信，实现 PWM 与 STS 混合控制。
3.  **平滑性优化**: 引入了加速度控制，解决了舵机跳变导致的机械抖动。

---
**更新日期**: 2026年4月15日
**当前状态**: 软硬件闭环测试完成，系统运行稳定。
