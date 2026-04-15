#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <Arduino.h>
#include <SCServo.h>
#include <ESP32Servo.h>
#include <math.h>
#include <Preferences.h>  

struct Point3D {float x, y, z;};
struct JointAngles {float j1, j2, j3, j4;};

class RobotArm {
public:
    // 构造函数：初始化物理参数
    RobotArm(float l1 = 24.0f, float l2 = 62.0f, float l3 = 40.0f, float l4 = 20.0f);

    // 硬件初始化
    void begin();

    // 运动控制接口
    bool moveTo(float x, float y, float z, float alpha = 0.0f);
    void setJoints(const double* rads, size_t size);
    void goHome();

    void calibrateHome(); // 校准回原点位置
    void savehome();
    void loadhome();
    void printHome();// 打印当前保存的home位置

    // 辅助功能
    Point3D getForwardKinematics(JointAngles angles); // 计算正向运动学

private:
    // 物理长度
    float L1, L2, L3, L4;

    float offset1 = 0.0f;
    float offset2 = 0.0f;
    float offset3 = 0.0f;
    float offset4 = 0.0f;

    Preferences prefs; // 用于存储校准数据

    // 硬件对象
    SMS_STS sts_bus2, sts_bus1;
    Servo base_servo;

    // 内部转换逻辑
    uint16_t radToSTS(float rad, bool reverse = false);
    uint16_t radToPWM(float rad, bool reverse = true);

    // 硬件引脚定义 (内部封装)
    const int PIN_PWM = 15;
    const int PIN_BUS_17 = 17; // ID 15
    const int PIN_BUS_5  = 5;  // ID 12
    const int PIN_BUS_14 = 14; // ID 14
};

#endif
