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
const int PWM_SERVO_PIN = 15;
const int BUS_IO_17     = 17;
const int BUS_IO_5      = 5;
const int BUS_IO_14     = 14;

SMS_STS sts_bus2, sts_bus1;
Servo base_servo;

// --- 2. micro-ROS 变量 ---
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 为 JointState 消息分配足够的静态内存
double pos_data[10];
double vel_data[10];
double eff_data[10];
rosidl_runtime_c__String name_data[10];
char name_buffers[10][20];

// --- 3. 映射逻辑：将 RViz 弧度转为舵机指令 ---
uint16_t radToSTS(float rad) {
    // STS 舵机：2048 为中位 (0 rad)
    float pos = (rad / 3.1415926) * 2048 + 2048;
    return (uint16_t)constrain(pos, 0, 4095);
}
uint16_t radToPWM(float rad) {
    // PWM 舵机：1500us 为中位 (0 rad)
    float us = (rad / 1.5708) * 1000 + 1500;
    return (uint16_t)constrain(us, 500, 2500);
}

// --- 4. 订阅回调：拖动滑块时此函数应被触发 ---
void subscription_callback(const void * msgin) {
    // 只要收到消息，灯就会闪，说明通信通了
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    
    // 这里的映射关系：
    // msg->position.data[0] -> joint1 (底座)
    // msg->position.data[1] -> joint2 (ID 15)
    // msg->position.data[2] -> joint3 (ID 12)
    // msg->position.data[3] -> joint4 (ID 14)
    
    if (msg->position.size >= 4) {
        base_servo.writeMicroseconds(radToPWM(msg->position.data[0]));
        sts_bus2.WritePosEx(15, radToSTS(msg->position.data[1]), 1500, 50);
        sts_bus1.WritePosEx(12, radToSTS(msg->position.data[2]), 1500, 50);
        sts_bus2.WritePosEx(14, radToSTS(msg->position.data[3]), 1500, 50);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // 硬件串口与 GPIO 矩阵映射 (保持接线不动)
    Serial2.begin(1000000, SERIAL_8N1, 16, BUS_IO_17); 
    gpio_matrix_out(BUS_IO_14, U2TXD_OUT_IDX, false, false);
    sts_bus2.pSerial = &Serial2;

    Serial1.begin(1000000, SERIAL_8N1, 4, BUS_IO_5); 
    sts_bus1.pSerial = &Serial1;

    ESP32PWM::allocateTimer(0);
    base_servo.setPeriodHertz(50);
    base_servo.attach(PWM_SERVO_PIN, 500, 2500);

    // micro-ROS 内存预分配 (关键修复)
    msg.position.data = pos_data;
    msg.position.capacity = 10;
    msg.velocity.data = vel_data;
    msg.velocity.capacity = 10;
    msg.effort.data = eff_data;
    msg.effort.capacity = 10;
    msg.name.data = name_data;
    msg.name.capacity = 10;
    for(int i=0; i<10; i++) {
        msg.name.data[i].data = name_buffers[i];
        msg.name.data[i].capacity = 20;
    }

    delay(2000); // 等待 Agent 启动

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_twin_arm", "", &support);

    // 订阅话题名必须与 ROS 2 中的一致：joint_states
    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states");
    
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
    // 提高执行频率
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}