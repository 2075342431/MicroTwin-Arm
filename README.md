# MicroTwin-Arm 数字孪生机械臂系统

## 1. 项目概述

**MicroTwin-Arm** 是一个自研的四轴微型机械臂数字孪生系统，实现了物理实体与数字仿真之间的实时双向同步。系统以 **ESP32** 为下位机主控，运行 **micro-ROS** 节点驱动混合舵机；以 **ROS 2 Humble** 为上位机框架，提供 3D 可视化与交互控制。项目涵盖从 URDF 建模、运动学算法、嵌入式驱动到底层通信协议的全栈实现。

| 维度 | 技术方案 |
| :--- | :--- |
| 自由度 | 4-DOF（底座旋转 + 大臂 + 小臂 + 末端） |
| 主控芯片 | ESP32（Arduino 框架 + PlatformIO） |
| 机器人中间件 | micro-ROS（基于 rclc 的嵌入式 ROS 2 节点） |
| 上位机 | ROS 2 Humble + RViz2 + joint_state_publisher_gui |
| 舵机类型 | PWM 舵机 ×1 + STS 总线舵机 ×3（混合驱动） |
| 通信链路 | USB Serial @ 921600 bps（上下位机）+ 1M bps（舵机总线） |

---

## 2. 核心技术亮点

### 2.1 逆运动学——几何法解析求解

**这是本项目的核心算法贡献。** 在 `arm/lib/RobotArm/RobotArm.cpp` 中独立实现了基于几何法的四轴机械臂逆运动学求解器，未依赖任何第三方运动学库。

求解流程：给定笛卡尔空间目标点 $(x, y, z)$ 与末端姿态角 $\alpha$，依次求解四个关节角：

- **$\theta_1$**（底座旋转）：$\theta_1 = \arctan2(y, x)$，将三维问题投影到水平面直接求解；
- **$\theta_3$**（小臂关节）：将腕部坐标 $r_{wrist}, z_{wrist}$ 代入余弦定理，由 $L_2, L_3$ 构成的三角形反解 $\theta_3 = \pi - \arccos\frac{L_2^2+L_3^2-D^2}{2L_2L_3}$；
- **$\theta_2$**（大臂关节）：结合 $\beta = \arctan2(z_{wrist}, r_{wrist})$ 与 $\gamma$ 辅助角，求解 $\theta_2 = \frac{\pi}{2} - (\beta+\gamma)$；
- **$\theta_4$**（末端关节）：由运动链约束 $\theta_4 = (\theta_2+\theta_3) - \alpha$ 直接导出。

```cpp
// RobotArm::moveTo() — 逆运动学核心代码片段
target.j1 = atan2(y, x);
float r_total = sqrt(x * x + y * y);
float r_wrist = r_total - L4 * cos(alpha);
float z_wrist = z - L1 - L4 * sin(alpha);
float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
if (D > (L2 + L3) || D < abs(L2 - L3)) return false;  // 工作空间边界检查
float cos_C = (L2 * L2 + L3 * L3 - D_sq) / (2.0f * L2 * L3);
target.j3 = M_PI - acos(constrain(cos_C, -1.0f, 1.0f));
```

**技术价值**：

- 在资源受限的 ESP32 上实现实时 IK 求解，无需依赖 PC 端计算；
- 包含工作空间可达性校验，对不可达点提前拒绝，避免舵机过载；
- 提供了 `moveTo()` 和 `moveTo_2()` 两个版本，对应不同的 DH 参数建模方案，体现了算法的灵活性。

### 2.2 正运动学——前向位姿计算

同样在 ESP32 固件中实现了正运动学求解器 `getForwardKinematics()`，由关节角度正向计算末端执行器的空间位置，可用于位置反馈验证：

$$r = L_2\cos(\theta_2') + L_3\cos(\theta_3') + L_4\cos(\theta_4')$$

$$p_x = r\cos\theta_1, \quad p_y = r\sin\theta_1, \quad p_z = L_1 + L_2\sin\theta_2' + L_3\sin\theta_3' + L_4\sin\theta_4'$$

正逆解互为验证，构成了完整的运动学算法闭环。

### 2.3 混合舵机驱动架构

机械臂采用 PWM 舵机与 STS 总线舵机混合方案，在 ESP32 上自主研发了统一驱动层（`RobotArm` 类）：

| 关节 | 舵机类型 | 通信方式 | 控制精度 | 技术要点 |
| :--- | :--- | :--- | :--- | :--- |
| Joint 1 (底座) | PWM 舵机 | LEDC PWM, IO 15 | [-1.57, 1.57] rad → [500, 2500] μs | 50Hz PWM，方向可逆映射 |
| Joint 2 (大臂) | STS3032 总线舵机 | Serial2, ID 15 | [-π, π] rad → [0, 4095] step | 加速度控制，平滑启停 |
| Joint 3 (小臂) | STS3032 总线舵机 | Serial1, ID 12 | [-π, π] rad → [0, 4095] step | 独立串口，低延迟 |
| Joint 4 (末端) | STS3032 总线舵机 | Serial2, ID 14 | [-π, π] rad → [0, 4095] step | 与 Joint 2 共享总线 |

**关键工程技巧**：
- **GPIO 矩阵重映射**：Joint 2 和 Joint 4 共享 `Serial2` 的 TX 引脚（IO 16），但通过 ESP32 GPIO 矩阵将 U2TXD 输出同时路由到 IO 14，实现单 UART 控制两条总线；
- **弧度到舵机步数的线性映射**：`radToSTS(rad)` 将关节弧度精确映射到 STS 舵机的 0–4095 位置范围，支持方向反转参数；
- **WritePosEx 加速度控制**：使用 STS 舵机的带速度/加速度的位置指令 `WritePosEx(id, pos, speed, acc)`，Acc=30 实现平滑启停，消除了舵机跳变导致的机械抖动。

### 2.4 EPROM 调零校准系统

针对总线舵机安装偏差导致的零位不一致问题，实现了完整的工程化校准流程：

- **calibrateHome()**：释放全部 STS 舵机扭矩，允许手动将机械臂掰至物理零位；
- **saveHome()**：调用 STS 舵机的 EPROM 指令 `CalibrationOfs()`，将当前物理位置固化偏移量为 2048（中位）；
- **printHome()**：读取并打印各舵机当前位置值，方便调试。

这一校准流程解决了同一型号舵机上电后零位不一致的工业级痛点。

### 2.5 数字孪生实时同步

```
[PC 端 ROS 2 Humble]
    │  joint_state_publisher_gui  ← 用户拖动滑块设定目标角度
    │  robot_state_publisher      ← 解析 URDF，发布 TF 变换
    │  RViz2                      ← 3D 实时渲染机械臂姿态
    │  micro_ros_agent            ← 串口网关 @ 921600 bps
    ║
    ║  USB Serial @ 921600 bps (高速微ROS链路)
    ║
[ESP32 下位机]
    │  micro-ROS Node (rclc)      ← 订阅 joint_states 话题
    │  RobotArm 驱动层            ← 弧度→舵机步数映射 + 逆运动学
    │  PWM / STS 总线输出          ← 驱动实体舵机
```

上位机任何关节角度的调整都会实时映射到物理舵机，同时物理舵机的位置也可通过总线读回，实现**虚实双向映射**。

### 2.6 PID 控制器库

在 `arm/lib/PidController/` 中实现了一个通用的 PID 控制器类：

- 经典位置式 PID 算法：$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$
- **积分限幅（Anti-Windup）**：通过 `setIntegralLimits()` 设置积分项上下界，防止积分饱和；
- 独立封装为可复用库，为后续闭环力控/速度控制预留接口。

---

## 3. 硬件系统架构

### 3.1 核心主控

| 参数 | 规格 |
| :--- | :--- |
| 开发板 | ESP32-WROOM (Fishbot DevKit) |
| 电源 | 12V DC（总线舵机动力） + 5V（ESP32 逻辑） |
| micro-ROS 波特率 | 921600 |
| STS 总线波特率 | 1M (1,000,000) |

### 3.2 舵机引脚与 ID 分配

| 关节 | 舵机类型 | 控制 ID | TX / RX | 串口实例 | 功能 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Joint 1 (Base) | PWM 舵机 | N/A | IO 15 | LEDC PWM | 底部水平旋转 |
| Joint 2 | STS3032 总线 | 15 | TX 16 / RX 17 | Serial2 | 大臂俯仰 |
| Joint 3 | STS3032 总线 | 12 | TX 4 / RX 5 | Serial1 | 小臂俯仰 |
| Joint 4 | STS3032 总线 | 14 | TX 16 / RX 17 | Serial2 | 末端旋转/夹爪 |

> Joint 2 与 Joint 4 共用 `Serial2` 总线，通过舵机 ID 区分指令目标。

---

## 4. 软件系统架构

```
MicroTwin-Arm/
├── arm/                              # ESP32 固件（PlatformIO）
│   ├── src/
│   │   ├── main.cpp                  # micro-ROS 节点主程序
│   │   └── guanbo.cpp                # 舵机 ID 探测与校准工具
│   ├── lib/
│   │   ├── RobotArm/
│   │   │   ├── RobotArm.h            # 机械臂驱动类声明（IK/FK/舵机控制）
│   │   │   └── RobotArm.cpp          # 运动学求解 + 舵机驱动实现
│   │   └── PidController/
│   │       ├── PidController.h       # PID 控制器类声明
│   │       └── PidController.cpp     # PID 算法实现（含积分限幅）
│   ├── include/
│   ├── test/
│   ├── platformio.ini                # PlatformIO 工程配置
│   └── LEARN.MD                      # 进阶学习路线
│
└── ros2_ws/                          # ROS 2 工作空间
    └── fishbot_description/
        ├── fishbot_description/      # Python 包目录
        │   ├── arm.urdf              # URDF 机器人描述模型
        │   ├── meshes/               # STL 网格文件（base_link ~ link4）
        │   └── display_rviz2.launch.py  # 仿真启动文件
        ├── launch/
        ├── rviz/
        │   └── display_arm.rviz      # RViz2 配置
        ├── urdf/
        │   ├── arm.urdf              # URDF 模型（完整关节/惯性参数）
        │   └── arm.csv               # 各连杆质量/惯量/视觉参数表
        └── package.xml
```

---

## 5. 快速开始

### 5.1 固件编译与烧录

```bash
cd arm
pio run -t upload
```

### 5.2 启动 ROS 2 仿真环境

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch fishbot_description display_rviz2.launch.py
```

### 5.3 启动 micro-ROS Agent（连接 ESP32）

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 921600
```

### 5.4 串口调试命令

| 命令 | 功能 |
| :--- | :--- |
| `H` | 机械臂归零 |
| `Z` | 进入手动调零模式（释放扭矩） |
| `S` | 保存当前位置为零位（写入 EPROM） |
| `P` | 打印当前舵机位置 |

---

## 6. 项目进展

1. **URDF 建模**：完成 4 轴机械臂完整的 URDF 描述，含 STL 网格、关节限位、转动轴定义及惯性参数，支持 RViz2 实时渲染。
2. **运动学算法**：在 ESP32 上独立实现几何法正逆运动学求解器，支持笛卡尔空间直接控制 `(x, y, z, α) → joint angles`，含工作空间边界检查。
3. **底层驱动**：ESP32 固件完成 micro-ROS 串口通信 + PWM/STS 混合舵机驱动，支持加速度平滑控制。
4. **调零校准**：完成 STS 舵机 EPROM 偏移量校准流程，实现一键归零和一键保存零位。
5. **闭环测试**：上位机 RViz2 滑动条 → micro-ROS → ESP32 → 舵机，全链路通信验证通过，系统运行稳定。

---

**更新日期**：2026 年 5 月 13 日  
**当前状态**：软硬件闭环测试完成，正逆运动学算法验证通过，系统稳定运行。
