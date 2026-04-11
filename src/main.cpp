#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <SCServo.h> 
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include "rom/gpio.h"

// --- 1. 硬件引脚 ---
const int LED_PIN       = 2;
const int PWM_SERVO_PIN = 15; // joint1
const int BUS_IO_17     = 17; // joint2 (ID 15)
const int BUS_IO_5      = 5;  // joint3 (ID 12)
const int BUS_IO_14     = 14; // joint4 (ID 14)

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

// --- 3. 订阅回调 (平滑控制版) ---
void subscription_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    if (msg->position.size >= 4) {
        // A. PWM 舵机 (joint1) - 设置为反向
        base_servo.writeMicroseconds(radToPWM(msg->position.data[0], true));
        
        // B. 总线舵机 (平滑参数控制)
        // 参数说明：ID, 位置, 速度(1000), 加速度(30)
        // 速度 1000 表示平稳运行，加速度 30 消除卡顿感
        
        // ID 15 (joint2) - 正向
        sts_bus2.WritePosEx(15, radToSTS(msg->position.data[1], false), 1000, 30);
        
        // ID 12 (joint3) - 正向
        sts_bus1.WritePosEx(12, radToSTS(msg->position.data[2], false), 1000, 30);
        
        // ID 14 (joint4) - 设置为反向
        sts_bus2.WritePosEx(14, radToSTS(msg->position.data[3], true), 1000, 30);
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
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}