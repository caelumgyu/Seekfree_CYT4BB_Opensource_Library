// ----------------- adrc.cpp -----------------
#include "adrc.h"

void LADRC_1st_Update(LADRC_1st_Struct *adrc, float desire, float measure, float dt,uint8 duty)
{
    // 1. 线性扩张状态观测器 (LESO)
    float e = adrc->z1 - measure;

    float beta1 = 2.0f * adrc->wo;
    float beta2 = adrc->wo * adrc->wo;

    // z1正常积分
    adrc->z1 += (adrc->z2 + adrc->b0 * adrc->u - beta1 * e) * dt;

    // ================== z2 抗积分饱和与限幅 (核心修改) ==================
    float z2_dot = -beta2 * e;

    // 如果输出已经达到最大负限幅(-8000)，且 z2_dot 还要继续让 z2 变大(导致u更负)，则停止积分
    if (adrc->u <= -duty * 100.0f && z2_dot > 0.0f)
    {
        z2_dot = 0.0f;
    }
    // 如果输出已经达到最大正限幅(+8000)，且 z2_dot 还要继续让 z2 变小(导致u更正)，则停止积分
    else if (adrc->u >= duty * 100.0f && z2_dot < 0.0f)
    {
        z2_dot = 0.0f;
    }

    adrc->z2 += z2_dot * dt;

    // 给 z2 一个绝对值的物理硬限幅！不要让它无限增长！
    // 比如最大控制量是 8000，那么 z2 最大不应该超过 8000 * b0
    float max_z2 = duty * 100.0f * adrc->b0 * 0.8f; // 留一点余量设为0.8
    adrc->z2 = compare_float(adrc->z2, -max_z2, max_z2);
    // =================================================================

    // 2. 误差反馈控制 (线性 PD)
    float kp = adrc->wc;
    float u0 = kp * (desire - adrc->z1);

    // 3. 扰动补偿
    adrc->u = (u0 - adrc->z2) / adrc->b0;

    // 4. 输出限幅
    adrc->u = compare_float(adrc->u, -duty * 100.0f, duty * 100.0f);
}

int compare(int x, int min, int max)
{
    if (x < min)
    {
        x = min;
    }
    else if (x > max)
    {
        x = max;
    }
    return x;
}
float compare_float(float x, float min, float max)
{
    if (x < min)
    {
        x = min;
    }
    else if (x > max)
    {
        x = max;
    }
    return x;
}