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
const int PWM_SERVO_PIN = 15;
const int BUS_IO_17     = 17;
const int BUS_IO_5      = 5;
const int BUS_IO_14     = 14;

struct Point3D { float x, y, z; };
struct JointAngles { float j1, j2, j3, j4; };

// 机械臂长度参数 (mm) - 请确保这些值的准确性
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

double pos_data[6];
rosidl_runtime_c__String name_data[6];
char name_buffers[6][20];

// --- 转换逻辑 ---
inline uint16_t radToSTS(float rad, bool reverse = false) {
    if (reverse) rad = -rad; 
    return (uint16_t)((rad / M_PI) * 2048.0f + 2048.0f);
}

inline uint16_t radToPWM(float rad, bool reverse = true) {
    if (reverse) rad = -rad; 
    return (uint16_t)((rad / 1.570796f) * 1000.0f + 1500.0f);
}

// --- 逆向运动学 (IK) ---
bool moveTo(float x, float y, float z, float alpha = 0.0f) {    
    JointAngles target;
    target.j1 = atan2(y, x);
    float r_total = sqrt(x * x + y * y);
    float r_wrist = r_total - L4 * cos(alpha);
    float z_wrist = z - L1 - L4 * sin(alpha);
    float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float D = sqrt(D_sq);
    
    if (D > (L2 + L3) || D < abs(L2 - L3)) {
        Serial.printf(">> [Error] Target (%.1f, %.1f, %.1f) Unreachable! D=%.2f\n", x, y, z, D);
        return false;
    }
    
    float cos_C = (L2 * L2 + L3 * L3 - D_sq) / (2.0f * L2 * L3);
    float C = acos(constrain(cos_C, -1.0f, 1.0f));
    target.j3 = (M_PI - C); 
    
    float beta = atan2(z_wrist, r_wrist); 
    float cos_gamma = (L2 * L2 + D_sq - L3 * L3) / (2.0f * L2 * D);
    float gamma = acos(constrain(cos_gamma, -1.0f, 1.0f));
    
    float t2 = beta + gamma; 
    target.j2 = (M_PI / 2.0f) - t2;
    float t3 = t2 - target.j3;
    target.j4 = t3 - alpha;

    Serial.printf(">> Moving to (%.1f, %.1f, %.1f) | Angles: J1:%.2f, J2:%.2f, J3:%.2f, J4:%.2f\n", 
                  x, y, z, target.j1, target.j2, target.j3, target.j4);

    base_servo.writeMicroseconds(radToPWM(target.j1, true));
    sts_bus2.WritePosEx(15, radToSTS(target.j2, false), 1000, 30);
    sts_bus1.WritePosEx(12, radToSTS(target.j3, false), 1000, 30);
    sts_bus2.WritePosEx(14, radToSTS(target.j4, false), 1000, 30); 
    return true;
}

void goHome() {
    Serial.println(">> Action: Homing (Vertical)...");
    base_servo.writeMicroseconds(1500);
    sts_bus2.WritePosEx(15, 2048, 500, 30);
    sts_bus1.WritePosEx(12, 2048, 500, 30);
    sts_bus2.WritePosEx(14, 2048, 500, 30);
    delay(2000);
}

void subscription_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    if (msg->position.size >= 4) {
        base_servo.writeMicroseconds(radToPWM(msg->position.data[0], true));
        sts_bus2.WritePosEx(15, radToSTS(msg->position.data[1], false), 1000, 30);
        sts_bus1.WritePosEx(12, radToSTS(msg->position.data[2], false), 1000, 30);
        sts_bus2.WritePosEx(14, radToSTS(msg->position.data[3], false), 1000, 30);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(921600);
    // 注意：如果指令没反应，请尝试注释掉下面这一行 micro-ROS 的初始化
    // set_microros_serial_transports(Serial); 

    Serial2.begin(1000000, SERIAL_8N1, 16, BUS_IO_17); 
    gpio_matrix_out(BUS_IO_14, U2TXD_OUT_IDX, false, false);
    sts_bus2.pSerial = &Serial2;

    Serial1.begin(1000000, SERIAL_8N1, 4, BUS_IO_5); 
    sts_bus1.pSerial = &Serial1;

    ESP32PWM::allocateTimer(0);
    base_servo.setPeriodHertz(50);
    base_servo.attach(PWM_SERVO_PIN, 500, 2500);

    delay(1000);
    sts_bus2.EnableTorque(15, 1);
    sts_bus1.EnableTorque(12, 1);
    sts_bus2.EnableTorque(14, 1);
    
    goHome();
    Serial.println(">> System Ready. Commands: 'H' (Home), 'T' (Test Point), 'S' (Safe Standby)");
}

void loop() {
    // 串口指令解析 (增加更强反馈)
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        Serial.printf(">> Received Char: '%c'\n", cmd);
        
        if (cmd == 'H') {
            goHome();
        } else if (cmd == 'T') {
            moveTo(110.0, 0.0, 60.0, 0.0); // 移动到测试点 A
        } else if (cmd == 'S') {
            moveTo(60.0, 0.0, 100.0, 0.5); // 移动到安全待机位置 B
        }
        
        // 清除缓冲区中的换行符等杂质
        while(Serial.available() > 0) Serial.read();
    }

    // 暂时禁用了自动 loop 移动，方便你测试串口指令
    // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}