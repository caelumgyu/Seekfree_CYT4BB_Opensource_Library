/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "pid.h"
#include "adrc.h"
#include "vl53l8cx.h"
#include "small_driver_uart_control.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
extern LADRC_1st_Struct gyro_y_adrc;
extern LADRC_1st_Struct gyro_x_adrc;
extern LADRC_1st_Struct gyro_z_adrc;
extern float out_yaw;
extern float out_roll;
extern float out_pitch;

extern PID_Struct distance_pid;
extern PID_Struct velocity_pid;
extern float global_output;
extern float voltage;
extern bool flow_complete;
extern volatile uint8 tof_new_flag;


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    imu660rc_init(IMU660RC_QUARTERNION_480HZ);
    small_driver_uart_init();
    lora3a22_init();
    // dl1a_init();
    uint8 result = vl53l8cx_device_init();
    printf("VL53L8CX init result: %d\n", result);
    // upflow302_receive_init();
    // vl53l8cx_init();
    gpio_init(P19_0, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    adc_init(ADC0_CH21_P07_5, ADC_12BIT);

    pit_ms_init(PIT_CH1, TIME_DELAY * 1000 * 20); // 用于获取tof和光流
    pit_ms_init(PIT_CH0, TIME_DELAY * 1000);
    // 遥控器按键状态
    //pit_ms_init(PIT_CH1, 10);
    
    gpio_set_level(P19_0, GPIO_LOW);
    uint16 adc_value = 0;
    float voltage = 12.0f;
    // 此处编写用户代码 例如外设初始化代码等
    while (true)
    {
        vl53l8cx_get_distance();
        tof_new_flag = 1; // notify 1kHz ISR for one altitude-hold update
        // 此处编写需要循环执行的代码
        // printf("Data:%0.2f,   %0.2f,   %0.2f\n", imu660rc_pitch, imu660rc_roll, imu660rc_yaw);
        // printf("Data:%0.2f,   %0.2f,   %0.2f\n", imu660rc_acc_x, imu660rc_acc_y, imu660rc_acc_z);
        static uint8 print_count = 0;
        if (print_count >= 5) // lower print rate (~10Hz) to avoid serial stalling the TOF poll period
        {
            print_count = 0;
            printf("Data:%0.2f,   %0.2f,   %0.2f,   %0.2f,    %d\n", distance_pid.output, velocity_pid.output, global_output, distance_pid.desire, vl53l8cx_distance_mm);
        }
        else
        {
            print_count++;
        }
        // printf("out:%0.2f   ,%0.2f   ,%0.2f \r\n", out_roll, out_pitch, out_yaw);

        // ADC测电压 1秒一次
        static uint8 adc_count = 0;
        if (adc_count < 50)
        {
            adc_count++;
        }
        else
        {
            adc_count = 0;
            adc_value = adc_mean_filter_convert(ADC0_CH21_P07_5, 10);
            voltage = (adc_value) * (223.0f / 20.0f) * (3.3f / 4096.0f);
            // printf("Voltage:%0.2f\n", voltage);
        }

        system_delay_ms(20);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
