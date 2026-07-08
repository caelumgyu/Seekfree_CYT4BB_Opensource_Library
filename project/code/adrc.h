// ----------------- adrc.h -----------------
#ifndef __ADRC_H__
#define __ADRC_H__

#include "zf_common_headfile.h"

typedef struct
{
    // 3个核心需要调的参数
    float b0; // 系统增益（极其重要，决定了补偿的力度）
    float wo; // 观测器带宽 (omega_o)
    float wc; // 控制器带宽 (omega_c)

    // 内部状态变量 (不需要外部赋值)
    float z1; // 观测出的当前角速度
    float z2; // 观测出的总扰动 (ADRC的灵魂)
    float u;  // 最终的控制输出
} LADRC_1st_Struct;

int compare(int x, int min, int max);
float compare_float(float x, float min, float max);

// 运算周期 (对应你的 PID_PERIOD)
void LADRC_1st_Update(LADRC_1st_Struct *adrc, float desire, float measure, float dt, uint8 duty);

#endif