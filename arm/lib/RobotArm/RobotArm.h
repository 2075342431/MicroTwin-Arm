#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <Arduino.h>
#include <SCServo.h>
#include <ESP32Servo.h>
#include <math.h>

struct Point3D {
    float x, y, z;
};

struct JointAngles {
    float j1, j2, j3, j4;
};

class RobotArm {
public:
    void saveHome();      // 将当前位置写入 EPROM 作为新的 2048 零位

    RobotArm(float l1 = 24.0f, float l2 = 62.0f, float l3 = 40.0f, float l4 = 45.0f);

    void begin();
    bool moveTo(float x, float y, float z, float alpha = 0.0f);
    void setJoints(const double* rads, size_t size);
    void goHome();
    Point3D getForwardKinematics(JointAngles angles);

    // --- 新增：校准相关功能 ---
    void calibrateHome(); // 释放扭矩，进入手动调零模式
    void printHome();     // 打印当前所有舵机的物理位置

private:
    float L1, L2, L3, L4;
    SMS_STS sts_bus2, sts_bus1;
    Servo base_servo;

    uint16_t radToSTS(float rad, bool reverse = false);
    uint16_t radToPWM(float rad, bool reverse = true);

    const int PIN_PWM = 15;
    const int PIN_BUS_17 = 17; // ID 15
    const int PIN_BUS_5  = 5;  // ID 12
    const int PIN_BUS_14 = 14; // ID 14
};

#endif
