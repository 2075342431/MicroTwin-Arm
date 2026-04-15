#include "RobotArm.h"
#include <Preferences.h>
#include "rom/gpio.h"



RobotArm::RobotArm(float l1, float l2, float l3, float l4)
RobotArm::RobotArm(float l1, float l2, float l3, float l4)
    : L1(l1), L2(l2), L3(l3), L4(l4) {}


void RobotArm::begin()
{

void RobotArm::begin()
{
    // 串口 2 配置 (ID 15 & ID 14)
    Serial2.begin(1000000, SERIAL_8N1, 16, PIN_BUS_17);
    Serial2.begin(1000000, SERIAL_8N1, 16, PIN_BUS_17);
    pinMode(PIN_BUS_14, OUTPUT);
    gpio_matrix_out(PIN_BUS_14, U2TXD_OUT_IDX, false, false);
    sts_bus2.pSerial = &Serial2;

    // 串口 1 配置 (ID 12)
    Serial1.begin(1000000, SERIAL_8N1, 4, PIN_BUS_5);
    Serial1.begin(1000000, SERIAL_8N1, 4, PIN_BUS_5);
    sts_bus1.pSerial = &Serial1;

    // PWM 舵机配置
    ESP32PWM::allocateTimer(0);
    base_servo.setPeriodHertz(50);
    base_servo.attach(PIN_PWM, 500, 2500);

    delay(500);
    sts_bus2.EnableTorque(15, 1);
    sts_bus1.EnableTorque(12, 1);
    sts_bus2.EnableTorque(14, 1);

    loadhome();

    loadhome();
}

void RobotArm::goHome() {
    // 自动加上零位偏移
    double homeAngles[4] = {
        offset1,
        offset2,
        offset3,
        offset4
    };
    setJoints(homeAngles, 4);
    // 自动加上零位偏移
    double homeAngles[4] = {
        offset1,
        offset2,
        offset3,
        offset4
    };
    setJoints(homeAngles, 4);
}

uint16_t RobotArm::radToSTS(float rad, bool reverse)
{
    if (reverse)
        rad = -rad;
uint16_t RobotArm::radToSTS(float rad, bool reverse)
{
    if (reverse)
        rad = -rad;
    return (uint16_t)((rad / M_PI) * 2048.0f + 2048.0f);
}

uint16_t RobotArm::radToPWM(float rad, bool reverse)
{
    if (reverse)
        rad = -rad;
uint16_t RobotArm::radToPWM(float rad, bool reverse)
{
    if (reverse)
        rad = -rad;
    return (uint16_t)((rad / 1.570796f) * 1000.0f + 1500.0f);
}

void RobotArm::setJoints(const double* rads, size_t size) {
    if (size >= 4)
    {
    if (size >= 4)
    {
        base_servo.writeMicroseconds(radToPWM(rads[0], true));
        sts_bus2.WritePosEx(15, radToSTS(rads[1], false), 1000, 30);
        sts_bus1.WritePosEx(12, radToSTS(rads[2], false), 1000, 30);
        sts_bus2.WritePosEx(14, radToSTS(rads[3], false), 1000, 30); // 之前测试 false 更准
    }
}

bool RobotArm::moveTo(float x, float y, float z, float alpha) {
bool RobotArm::moveTo(float x, float y, float z, float alpha) {
    JointAngles target;
    target.j1 = atan2(y, x);
    float r_total = sqrt(x * x + y * y);
    float r_wrist = r_total - L4 * cos(alpha);
    float z_wrist = z - L1 - L4 * sin(alpha);
    float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float D = sqrt(D_sq);

    if (D > (L2 + L3) || D < abs(L2 - L3))
        return false;


    if (D > (L2 + L3) || D < abs(L2 - L3))
        return false;

    float cos_C = (L2 * L2 + L3 * L3 - D_sq) / (2.0f * L2 * L3);
    float C = acos(constrain(cos_C, -1.0f, 1.0f));
    target.j3 = (M_PI - C);

    float beta = atan2(z_wrist, r_wrist);
    target.j3 = (M_PI - C);

    float beta = atan2(z_wrist, r_wrist);
    float cos_gamma = (L2 * L2 + D_sq - L3 * L3) / (2.0f * L2 * D);
    float gamma = acos(constrain(cos_gamma, -1.0f, 1.0f));

    float t2 = beta + gamma;

    float t2 = beta + gamma;
    target.j2 = (M_PI / 2.0f) - t2;
    float t3 = t2 - target.j3;
    target.j4 = t3 - alpha;

    double rads[4] = {target.j1, target.j2, target.j3, target.j4};
    setJoints(rads, 4);
    return true;
}

Point3D RobotArm::getForwardKinematics(JointAngles angles) {
    Point3D p;
    float t2 = (M_PI / 2.0f) - angles.j2;
    float t3 = t2 - angles.j3;
    float t4 = t3 - angles.j4;
    float r = L2 * cos(t2) + L3 * cos(t3) + L4 * cos(t4);
    p.x = r * cos(angles.j1);
    p.y = r * sin(angles.j1);
    p.z = L1 + L2 * sin(t2) + L3 * sin(t3) + L4 * sin(t4);
    return p;
}

void RobotArm::loadhome()
{
    prefs.begin("robot_arm", true);
    offset1 = prefs.getFloat("offset1", 0.0f);
    offset2 = prefs.getFloat("offset2", 0.0f);
    offset3 = prefs.getFloat("offset3", 0.0f);
    offset4 = prefs.getFloat("offset4", 0.0f);
    prefs.end();
}

void RobotArm::calibrateHome()
{
    sts_bus2.EnableTorque(15, 0);
    sts_bus1.EnableTorque(12, 0);
    sts_bus2.EnableTorque(14, 0);
    offset1 = offset2 = offset3 = offset4 = 0.0f;

    Serial.println("请将机械臂移动到校准位置，然后输入当前各关节的实际角度（单位：度，逗号分隔）：");
}

void RobotArm::savehome()
{
    offset1 = 0.0f;
    offset2 = 0.0f;
    offset3 = 0.0f;
    offset4 = 0.0f;
    prefs.begin("robot_arm", false);
    prefs.putFloat("offset1", offset1);
    prefs.putFloat("offset2", offset2);
    prefs.putFloat("offset3", offset3);
    prefs.putFloat("offset4", offset4);
    prefs.end();

    Serial.println("✅ 新零位已保存！");
}

void RobotArm::printHome() {
    Serial.print("Home偏移: ");
    Serial.print(offset1);
    Serial.print(" ");
    Serial.print(offset2);
    Serial.print(" ");
    Serial.print(offset3);
    Serial.print(" ");
    Serial.println(offset4);
}