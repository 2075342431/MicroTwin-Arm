#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include "RobotArm.h"

// 实例化机械臂对象
RobotArm arm(24.0, 62.0, 40.0, 45.0);

// micro-ROS 变量
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 内存预分配
double pos_data[6];
rosidl_runtime_c__String name_data[6];
char name_buffers[6][20];

void subscription_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg_in = (const sensor_msgs__msg__JointState *)msgin;
    arm.setJoints(msg_in->position.data, msg_in->position.size);
}

void setup() {
    Serial.begin(921600);
    set_microros_serial_transports(Serial);

    // 初始化机械臂
    arm.begin();
    arm.goHome();

    // micro-ROS 初始化
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
    rclc_subscription_init_default(&subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states");
    
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    Serial.println(">> Robot Arm System Ready.");
}

void loop() {
    // 处理指令
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'H') arm.goHome();
        if (cmd == 'T') arm.moveTo(100.0, 0.0, 50.0, 0.0);
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}