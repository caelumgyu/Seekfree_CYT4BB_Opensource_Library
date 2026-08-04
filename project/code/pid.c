#include "pid.h"

// 单次PID计算
void PID_Calc(PID_Struct *pid)
{
    // 每个 PID 使用自己的计算周期（秒）；未设置（<=0）时回退到默认 1ms
    float period = (pid->period > 0.0f) ? pid->period : PID_PERIOD;

    pid->err = pid->desire - pid->measure; // 计算误差
    pid->integral += pid->err;             // 积分累积

    if (pid->last_err == 0)
    {
        pid->last_err = pid->err; // 第一次计算时，微分项为0
    }

    float der = pid->err - pid->last_err;                                                           // 微分项
    pid->output = pid->Kp * pid->err + (pid->Ki * pid->integral * period) + (pid->Kd * der / period); // PID输出
    pid->last_err = pid->err;                                                                       // 更新上次误差

    // 抗积分饱和: 输出超限时钳位输出并回退同向积分
    if (pid->out_max > pid->out_min)
    {
        if (pid->output > pid->out_max)
        {
            pid->output = pid->out_max;
            if (pid->err > 0.0f)           // 误差仍为正, 积分在错误方向累积
                pid->integral -= pid->err; // 回退本次累加
        }
        else if (pid->output < pid->out_min)
        {
            pid->output = pid->out_min;
            if (pid->err < 0.0f)
                pid->integral -= pid->err;
        }
    }
}
// 串级PID计算
void PID_Calc_chain(PID_Struct *pid_outer, PID_Struct *pid_inner)
{
    // 先计算外环
    PID_Calc(pid_outer);
    // 将外环的输出作为内环的目标值
    //  取负因为单级pid输出反了
    // pid_inner->desire = -pid_outer->output;
    pid_inner->desire = pid_outer->output;
    // 再计算内环
    PID_Calc(pid_inner);
}
