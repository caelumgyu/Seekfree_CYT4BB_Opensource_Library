#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"

#define PID_PERIOD TIME_DELAY // PID计算周期，单位为秒
#define PWM_DUTY_MAX 10000

typedef struct
{
    float Kp;       // 比例系数
    float Ki;       // 积分系数
    float Kd;       // 微分系数
    float err;      // 当前误差
    float desire;   // 目标值
    float measure;  // 测量值
    float last_err; // 上次误差
    float integral; // 积分累积值
    float output;   // PID输出值
    float out_min;  // 输出下限 (抗积分饱和)
    float out_max;  // 输出上限 (抗积分饱和)
} PID_Struct;

// 单次PID计算
void PID_Calc(PID_Struct *pid);
// 串级PID计算
void PID_Calc_chain(PID_Struct *pid_outer, PID_Struct *pid_inner);

#endif