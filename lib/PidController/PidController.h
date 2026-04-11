#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

    void setTarget(float target);
    float compute(float current, float dt);
    void setIntegralLimits(float min_sum, float max_sum); // 添加积分限幅设置函数

private:
    float target_ = 0.0f;
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;


    //pid
    float error_ = 0.0f;
    float error_sum_ = 0.0f;
    float last_error_ = 0.0f;
    float integral_sum_max_ = 0.0f; // 积分项上限
    float integral_sum_min_ = 0.0f; // 积分项下限

};

#endif