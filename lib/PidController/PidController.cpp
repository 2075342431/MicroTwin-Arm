#include "PidController.h"

PidController::PidController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
}

void PidController::setTarget(float target)
{
    target_ = target;
}

float PidController::compute(float current, float dt)
{
    error_ = target_ - current; // 误差 = 目标速度 - 当前实际速度
    error_sum_ += error_ * dt;  // 误差积分
    
    // 积分限幅
    if (integral_sum_max_ != 0.0f || integral_sum_min_ != 0.0f) { // 只有设置了限幅才进行操作
        if (error_sum_ > integral_sum_max_) {
            error_sum_ = integral_sum_max_;
        } else if (error_sum_ < integral_sum_min_) {
            error_sum_ = integral_sum_min_;
        }
    }

    float d_error = (error_ - last_error_) / dt; // 误差微分
    last_error_ = error_;

    // 经典 PID 计算公式
    return (kp_ * error_) + (ki_ * error_sum_) + (kd_ * d_error);
}

void PidController::setIntegralLimits(float min_sum, float max_sum) {
    integral_sum_min_ = min_sum;
    integral_sum_max_ = max_sum;
}