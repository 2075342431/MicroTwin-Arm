#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <SCServo.h> 
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include "rom/gpio.h"
#include <math.h>

// --- 1. 硬件引脚 ---
const int LED_PIN       = 2;
const int PWM_SERVO_PIN = 15; // joint1
const int BUS_IO_17     = 17; // joint2 (ID 15)
const int BUS_IO_5      = 5;  // joint3 (ID 12)
const int BUS_IO_14     = 14; // joint4 (ID 14)

struct Point3D {
    float x, y, z;
};

struct JointAngles {
    float j1; // PWM 舵机
    float j2; // 总线舵机 ID 15
    float j3; // 总线舵机 ID 12
    float j4; // 总线舵机 ID 14
};

// 机械臂长度参数 (mm)
const float L1 = 24.0f; 
const float L2 = 62.0f; 
const float L3 = 40.0f; 
const float L4 = 20.0f; 

SMS_STS sts_bus2, sts_bus1;
Servo base_servo;

rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 预分配内存
double pos_data[6];
rosidl_runtime_c__String name_data[6];
char name_buffers[6][20];

// --- 2. 转换逻辑 (包含反向处理) ---
inline uint16_t radToSTS(float rad, bool reverse = false) {
    if (reverse) rad = -rad; // 反向处理
    return (uint16_t)((rad / 3.14159265f) * 2048.0f + 2048.0f);
}

inline uint16_t radToPWM(float rad, bool reverse = true) {
    if (reverse) rad = -rad; // 反向处理
    return (uint16_t)((rad / 1.570796f) * 1000.0f + 1500.0f);
}

// --- 3. 正向运动学 (FK) ---
// 归零状态下全部竖直向上，即 angles.j = 0 时，仰角为 PI/2
Point3D forwardKinematics(JointAngles angles) {
    Point3D p;
    // 计算连杆2、3、4相对于水平面的绝对仰角
    float t2 = (M_PI / 2.0f) - angles.j2;
    float t3 = t2 - angles.j3;
    float t4 = t3 - angles.j4;

    // 计算末端执行器在基坐标系XY平面上的投影半径
    float r = L2 * cos(t2) + L3 * cos(t3) + L4 * cos(t4);

    p.x = r * cos(angles.j1);
    p.y = r * sin(angles.j1);
    p.z = L1 + L2 * sin(t2) + L3 * sin(t3) + L4 * sin(t4);
    return p;
}

// --- 4. 逆向运动学 (IK) ---
// alpha: 夹爪俯仰角 (0: 水平向前, -PI/2: 垂直向下)
bool moveTo(float x, float y, float z, float alpha = 0.0f) {    
    JointAngles target;
    
    // 1. 求底座角 j1
    target.j1 = atan2(y, x);
    
    // 2. 求水平投影半径
    float r_total = sqrt(x * x + y * y);
    
    // 3. 求手腕中心 (Joint 4) 的坐标
    float r_wrist = r_total - L4 * cos(alpha);
    float z_wrist = z - L1 - L4 * sin(alpha);
    
    // 4. 求 Joint 2 到 Joint 4 的直线距离 D
    float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float D = sqrt(D_sq);
    
    // 检查目标是否在臂展可达范围内
    if (D > (L2 + L3) || D < abs(L2 - L3)) {
        return false;
    }
    
    // 5. 使用余弦定理求内部角 C，进而求 j3 (Elbow Up 模式)
    float cos_C = (L2 * L2 + L3 * L3 - D_sq) / (2.0f * L2 * L3);
    float C = acos(constrain(cos_C, -1.0f, 1.0f));
    target.j3 = (M_PI - C); 
    
    // 6. 求 j2
    float beta = atan2(z_wrist, r_wrist); 
    float cos_gamma = (L2 * L2 + D_sq - L3 * L3) / (2.0f * L2 * D);
    float gamma = acos(constrain(cos_gamma, -1.0f, 1.0f));
    
    float t2 = beta + gamma; 
    target.j2 = (M_PI / 2.0f) - t2;
    
    // 7. 求 j4
    float t3 = t2 - target.j3;
    target.j4 = t3 - alpha;

    // --- 驱动舵机 ---
    base_servo.writeMicroseconds(radToPWM(target.j1, true));
    sts_bus2.WritePosEx(15, radToSTS(target.j2, false), 1000, 30);
    sts_bus1.WritePosEx(12, radToSTS(target.j3, false), 1000, 30);
    sts_bus2.WritePosEx(14, radToSTS(target.j4, true),  1000, 30);
    
    return true;
}

// --- 5. 订阅回调 ---
void subscription_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    if (msg->position.size >= 4) {
        // 映射到实物舵机
        base_servo.writeMicroseconds(radToPWM(msg->position.data[0], true));
        sts_bus2.WritePosEx(15, radToSTS(msg->position.data[1], false), 1000, 30);
        sts_bus1.WritePosEx(12, radToSTS(msg->position.data[2], false), 1000, 30);
        sts_bus2.WritePosEx(14, radToSTS(msg->position.data[3], true),  1000, 30);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUS_IO_14, OUTPUT);
    pinMode(BUS_IO_17, OUTPUT);

    // micro-ROS 高速串口
    Serial.begin(921600);
    set_microros_serial_transports(Serial);

    // 总线硬件串口映射
    Serial2.begin(1000000, SERIAL_8N1, 16, BUS_IO_17); 
    gpio_matrix_out(BUS_IO_14, U2TXD_OUT_IDX, false, false);
    sts_bus2.pSerial = &Serial2;

    Serial1.begin(1000000, SERIAL_8N1, 4, BUS_IO_5); 
    sts_bus1.pSerial = &Serial1;

    // PWM 舵机
    ESP32PWM::allocateTimer(0);
    base_servo.setPeriodHertz(50);
    base_servo.attach(PWM_SERVO_PIN, 500, 2500);

    // 锁定并初始化位置
    delay(500);
    sts_bus2.EnableTorque(15, 1);
    sts_bus1.EnableTorque(12, 1);
    sts_bus2.EnableTorque(14, 1);
    
    // micro-ROS 内存预分配
    msg.position.data = pos_data;
    msg.position.capacity = 6;
    msg.name.data = name_data;
    msg.name.capacity = 6;
    for(int i=0; i<6; i++) {
        msg.name.data[i].data = name_buffers[i];
        msg.name.data[i].capacity = 20;
    }

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_twin_arm", "", &support);

    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states");
    
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
    static unsigned long last_time = 0;
    static bool toggle = false;

    if (millis() - last_time > 5000)
    {
        if (toggle)
        {
            // 移动到点 A：前方 10cm, 高度 5cm, 夹爪水平 (alpha=0)
            moveTo(100.0, 0.0, 50.0, 0.0);
        }
        else
        {
            // 移动到点 B：左前方, 高度 8cm, 夹爪向下俯冲 (alpha=-0.5)
            moveTo(80.0, 40.0, 80.0, -0.5);
        }
        toggle = !toggle;
        last_time = millis();
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}