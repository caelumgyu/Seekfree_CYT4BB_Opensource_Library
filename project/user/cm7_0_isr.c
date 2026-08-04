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
 * 文件名称          cm7_0_isr
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-9      pudding            first version
 * 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
 * 2025-2-4      pudding            优化串口中断逻辑，防止意外干扰导致的卡死问题，优化串口波特率计算逻辑
 * 2025-2-4      pudding            新增两个串口接口
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "pid.h"
#include "adrc.h"
#include "vl53l8cx.h"
#include "small_driver_uart_control.h"

bool flow_complete = false;
bool dir1 = false; // 一般情况下true会让电机顺时针旋转
bool dir2 = true;  // 但是具体情况还要看电机的接线方式，可能需要调整
bool dir3 = false; // 这里的电机1因为接线不同导致反转，所以设置为false，其他三个电机接线方式相同，所以设置为true
bool dir4 = false; // 如果陀螺仪数据与预期的旋转方向相反，可以通过调整这些方向变量来修正
int16_t output_duty1 = 0;
int16_t output_duty2 = 0;
int16_t output_duty3 = 0;
int16_t output_duty4 = 0;
float global_output = 0;
float last_global_output = 0;
float out_pitch;
float out_roll;
float out_yaw;
int16 dist;

float last_filtered_distance_mm = 0; // 记录上一次高度求微分
float current_vel_mm_s = 0;          // 当前垂直速度
float base_hover_throttle = 6800.0f; // 基础悬停油门
volatile uint8 tof_new_flag = 0;     // TOF 新数据标志（主循环置位，定高计算消费）
uint32 last_tof_ms = 0;              // 上一次定高计算时的毫秒时间戳（用于实测 dt）
static uint32 isr_ms = 0;            // 1kHz 中断毫秒计数
static float filtered_distance_mm = 0; // TOF 距离低通滤波（50Hz 更新）
#pragma location = 0x28001014
__no_init float data_arr[4];
// __no_init uint32 beacon_lost;
// #pragma location = 0x28001018
// __no_init float car_position[2];
// uint32 last_beacon_lost = 0;

// PID结构体(先调内环，后调外环)
PID_Struct pitch_pid = {.Kp = 2.4f, .Ki = 0.00f, .Kd = 0.00f, .out_min = -2000.0f, .out_max = 2000.0f};
PID_Struct roll_pid = {.Kp = 2.4f, .Ki = 0.00f, .Kd = 0.00f, .out_min = -2000.0f, .out_max = 2000.0f};
PID_Struct yaw_pid = {.Kp = 0.8f, .Ki = 0.00f, .Kd = 0.00f, .out_min = -800.0f, .out_max = 800.0f};

// 定高（外环 Ki 补偿悬停油门偏差：PID_Calc 积分项乘 PID_PERIOD=0.001，外环 50Hz 执行，等效 0.1 duty/s/每mm误差；内环 Kd 先置 0，速度测量修好前只会放大噪声）
PID_Struct distance_pid = {.Kp = 1.2f, .Ki = 2.0f, .Kd = 0.0f, .out_min = -1200.0f, .out_max = 1200.0f, .desire = 200.0f};
PID_Struct velocity_pid = {.Kp = 2.8f, .Ki = 0.0f, .Kd = 0.0f, .out_min = -2700.0f, .out_max = 2700.0f};

// 跟车
PID_Struct position_x_pid = {.Kp = 0.068f, .Ki = 0.0f, .Kd = 0.000012f, .out_min = -8.0f, .out_max = 8.0f};
PID_Struct position_y_pid = {.Kp = 0.068f, .Ki = 0.0f, .Kd = 0.000012f, .out_min = -8.0f, .out_max = 8.0f};

PID_Struct acc_y_pid = {.Kp = 0.8f, .Ki = 0.00f, .Kd = 0.00f, .out_min = -2000.0f, .out_max = 2000.0f};
LADRC_1st_Struct gyro_y_adrc = {.b0 = 3.8f, .wo = 88.0f, .wc = 6.8f, .z1 = 0, .z2 = 0}; // 俯仰角速度
LADRC_1st_Struct gyro_x_adrc = {.b0 = 3.8f, .wo = 98.0f, .wc = 7.2f, .z1 = 0, .z2 = 0}; // 横滚角速度
LADRC_1st_Struct gyro_z_adrc = {.b0 = 3.2f, .wo = 58.0f, .wc = 4.8f, .z1 = 0, .z2 = 0}; // 偏航角速度

// LADRC_1st_Struct *LADRC_p[3] = {&gyro_x_adrc, &gyro_y_adrc, &gyro_z_adrc};

void send_uart_motol(float duty1, float duty2, float duty3, float duty4);

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr() // 定时器通道 0 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH0);
    isr_ms++;

    // 遥控器拨码开关
    if (lora3a22_uart_transfer.switch_key[1] == 1)
    {
        imu660rc_get_gyro();
        imu660rc_get_acc();

        // static uint8 no_start = 0;
        // static int16 no_start_angle = 0;
        // static int16 no_start_count = 0;
        static float filtered_gyro_distance_mm = 0;

        // vl53l8cx_get_data();
        // vl53l8cx_get_center_distance(&dist); // 获取中心区域平均距离
        // if (dist == 65535)
        //     dist = 149;

        // TOF低通滤波已移到 50Hz 门控定高计算中（见下方 tof_new_flag 段），1kHz 下滤波无意义

        // static uint8 count = 0;
        // if(count > 100){
        //     printf("Data:%d,   %d,   %d\n", imu660rc_gyro_x-x_zero, imu660rc_gyro_y-y_zero, imu660rc_gyro_z-z_zero);
        //     count = 0;
        // }
        // count++;

        imu660rc_gyro_x = imu660rc_gyro_x - x_zero;
        imu660rc_gyro_y = imu660rc_gyro_y - y_zero;
        imu660rc_gyro_z = imu660rc_gyro_z - z_zero;

        // SCB_CleanInvalidateDCache_by_Addr(&beacon_lost, sizeof(beacon_lost));
        // SCB_CleanInvalidateDCache_by_Addr(car_position, sizeof(car_position));
        SCB_CleanInvalidateDCache_by_Addr(data_arr, sizeof(data_arr));

        position_x_pid.desire = data_arr[1];
        position_x_pid.measure = 0;
        position_y_pid.desire = data_arr[2];
        position_y_pid.measure = 0;
        PID_Calc(&position_x_pid);
        PID_Calc(&position_y_pid);

        // printf("Data:%d,   %d,   %d\n", imu660rc_gyro_x-x_zero, imu660rc_gyro_y-y_zero, imu660rc_gyro_z-z_zero);
        //  俯仰角
        pitch_pid.desire = position_x_pid.output;
        // pitch_pid.desire = -lora3a22_uart_transfer.joystick[2] / 100;          // 目标值为水平
        // pitch_pid.desire = compare_float(data_arr[1] / 10, -3.0f, 3.0f);
        pitch_pid.measure = imu660rc_pitch;                                    // 当前俯仰角
        float gyro_y_meas = (imu660rc_gyro_y / imu660rc_transition_factor[1]); // 当前Y轴角速度

        // 横滚角
        roll_pid.desire = position_y_pid.output;
        // roll_pid.desire = -lora3a22_uart_transfer.joystick[3] / 100;           // 目标值为水平
        // roll_pid.desire = compare_float((data_arr[2] / 10), -3.0f, 3.0f);
        roll_pid.measure = imu660rc_roll;                                      // 当前横滚角
        float gyro_x_meas = (imu660rc_gyro_x / imu660rc_transition_factor[1]); // 当前X轴角速度

        // // 偏航角
        // yaw_pid.desire = imu660rc_yaw;                                         // 方向不变
        // yaw_pid.measure = -lora3a22_uart_transfer.joystick[0] / 100;           // 当前偏航角
        // float gyro_z_meas = (imu660rc_gyro_z / imu660rc_transition_factor[1]); // 当前Z轴角速度

        // if (no_start == 0 && filtered_gyro_distance_mm >= 1000) // 高度得大于1000
        // {
        //     yaw_pid.desire = imu660rc_yaw;
        //     yaw_pid.measure = no_start_angle;
        //     if (no_start_count < 20)
        //     {
        //         no_start_count++;
        //     }
        //     else
        //     {
        //         no_start_count = 0;
        //         no_start_angle += 1;
        //     }
        //     if (no_start_angle > 360)
        //     {
        //         no_start = 1;
        //     }
        //     goto nostart;
        // }

        static float base_search_yaw = 0.0f;
        static uint16 search_timer = 0;
        static uint16 beacon_timer = 0;
        static uint16 beacon_real_loast = 0;
        static uint16 lost_timer = 0;
        static uint16 land_timer = 0;
        static uint8 search_start = 0;
        static uint8 search_state = 0;
        static uint8 last_beacon_status = 1;
        static uint8 land_start = 0;
        float target_yaw = base_search_yaw;
        // if (no_start == 1)
        // {

            if (land_timer > 8000) // 等待八秒
            {
                data_arr[3] = land_start;
                goto land;
            }

            if (data_arr[0] == 1) // 识别到了信标
            {
                lost_timer = 0;
                if (beacon_real_loast > 200)
                { // 防止将小车突然被识别成信标(留大概5帧
                    land_timer = 0;
                }

                if (last_beacon_status == 0) // 上一次是丢失状态
                {
                    beacon_real_loast = 0;
                    beacon_timer = 0;
                    last_beacon_status = 1;
                }
                else
                {
                    beacon_real_loast++;
                    beacon_timer++;
                    if (beacon_timer > 100)
                    {
                        beacon_timer = 0;
                        base_search_yaw = imu660rc_yaw; // 记录当前偏航角作为基准
                    }
                }

                target_yaw = imu660rc_yaw;
                // 重置搜索状态，为下一次丢失做准备
                last_beacon_status = 1;
                search_timer = 0;
                search_state = 0;
                search_start = 0;
            }
            else if (data_arr[0] == 0) // 未识别到信标
            {
                beacon_real_loast = 0;
                beacon_timer = 0;
                if (last_beacon_status == 1)
                {
                    // base_search_yaw = imu660rc_yaw;
                    // target_yaw = base_search_yaw;
                    // 粗略计算灭灯次数
                    land_start += 1;
                    if (land_start > 10)
                    {
                        land_start = 10;
                    }
                    last_beacon_status = 0;
                    search_timer = 0;
                    search_start = 0;
                    search_state = 0;
                }
                else
                {
                    if (land_start == 10)
                    {
                        land_timer++;
                    }

                    lost_timer++;
                    if (lost_timer > 100 && search_start == 0)
                    {
                        lost_timer = 0;
                        search_start = 1;
                        search_state = !search_state;
                    }
                    if (search_start == 1)
                    {
                        search_timer++;

                        if (search_timer > 1500)
                        {
                            search_timer = 0;
                            search_state = !search_state;
                        }

                        if (search_state == 0)
                        {
                            target_yaw = base_search_yaw;
                        }
                        else
                        {
                            target_yaw = base_search_yaw + 130.0f;
                        }

                        if (target_yaw > 180.0f)
                            target_yaw -= 360.0f;
                        else if (target_yaw < -180.0f)
                            target_yaw += 360.0f;
                    }
                }
            }
        //}
        // 偏航角
        yaw_pid.desire = imu660rc_yaw; // 当前IMU偏航角 (-180到180)
        // static uint8 yaw_count = 0;
        // if (yaw_count < 40) // 每40ms累加一次
        // {
        //     yaw_count++;
        // }
        // else
        // {
        //     yaw_count = 0;
        //     yaw_pid.measure -= lora3a22_uart_transfer.joystick[0] / 500.0f; // 累加摇杆量作为目标
        // }
        yaw_pid.measure = target_yaw;
    // yaw_pid.measure = 0;
    // nostart:

        // 算出两者的原始差值
        float yaw_diff = yaw_pid.desire - yaw_pid.measure;

        if (yaw_diff > 180.0f)
        {
            yaw_pid.measure += 360.0f;
        }
        else if (yaw_diff < -180.0f)
        {
            yaw_pid.measure -= 360.0f;
        }

        float gyro_z_meas = (imu660rc_gyro_z / imu660rc_transition_factor[1]); // 当前Z轴角速度

        // ===== 定高（50Hz，由 TOF 新数据门控；I2C 读取保留在主循环，中断内不做阻塞通信）=====
        // 一毫秒累加一次（摇杆微调目标高度）
        static uint8 distance_count = 0;
        if (distance_count < 15)
        {
            distance_count++;
        }
        else if (distance_count >= 15)
        {
            distance_count = 0;
            distance_pid.desire += lora3a22_uart_transfer.joystick[1] / 500;
        }

        distance_pid.desire = compare_float(distance_pid.desire, -600.0f, 2000.0f);

        // 姿态外环保持 1kHz
        PID_Calc(&pitch_pid);
        PID_Calc(&roll_pid);
        PID_Calc(&yaw_pid);

        if (tof_new_flag)
        {
            tof_new_flag = 0;

            // 实测采样间隔 dt，避免主循环 printf 等造成的周期抖动
            float dt = (isr_ms - last_tof_ms) / 1000.0f;
            last_tof_ms = isr_ms;
            if (dt > 0.05f) dt = 0.05f; // 异常间隔限幅
            if (dt < 0.01f) dt = 0.02f;

            // 50Hz 低通滤波 + 姿态修正
            filtered_distance_mm = 0.5f * vl53l8cx_distance_mm + 0.5f * filtered_distance_mm;
            filtered_gyro_distance_mm = cosf(PI / 180 * imu660rc_roll) * filtered_distance_mm * cosf(PI / 180 * imu660rc_pitch); // 根据姿态调整距离测量值

            // 垂直速度：实测 dt 微分 + 低通滤波
            float raw_vel_mm_s = (filtered_gyro_distance_mm - last_filtered_distance_mm) / dt;
            current_vel_mm_s = 0.4f * raw_vel_mm_s + 0.6f * current_vel_mm_s;
            last_filtered_distance_mm = filtered_gyro_distance_mm;

            // 外环（去掉 ±20mm 死区，避免死区内零增益导致的极限环）+ 内环串级
            distance_pid.measure = filtered_gyro_distance_mm; // 外环测量值
            velocity_pid.measure = current_vel_mm_s;          // 内环测量值
            PID_Calc_chain(&distance_pid, &velocity_pid);

            // 最终全局油门 = 悬停基准值 + 串级内环的输出补偿量
            global_output = base_hover_throttle + velocity_pid.output;
            global_output = compare_float(global_output, MIN_DUTY * 100.0f, 95 * 100.0f);
        }

        // 作用给电机
        // 三者为叠加关系 (根据陀螺仪和四旋翼的方位关系来调整)
        //   电机1 =  俯仰角PID输出 + 横滚角PID输出 + 偏航角PID输出
        //   电机2 =  俯仰角PID输出 - 横滚角PID输出 - 偏航角PID输出
        //   电机3 = -俯仰角PID输出 + 横滚角PID输出 - 偏航角PID输出
        //   电机4 = -俯仰角PID输出 - 横滚角PID输出 + 偏航角PID输出
        //   逆时针1111********2222顺时针
        //         1111********2222
        //         ****************
        //         ****************
        //   顺时针3333********4444逆时针
        //         3333********4444

        // send_uart_motol(global_output + out_pitch + out_roll + out_yaw, // 电机1
        //                 global_output + out_pitch - out_roll - out_yaw, // 电机2
        //                 global_output - out_pitch + out_roll - out_yaw, // 电机3
        //                 global_output - out_pitch - out_roll + out_yaw  // 电机4
        // );

        // send_uart_motol(global_output - out_roll - out_pitch, // 电机1
        //                 global_output - out_roll + out_pitch, // 电机2
        //                 global_output + out_roll - out_pitch, // 电机3
        //                 global_output + out_roll + out_pitch  // 电机4
        // );

        if (vl53l8cx_distance_mm >= 120 || lora3a22_uart_transfer.switch_key[2] == 1)
        {
            // LADRC_1st_Update(adrc结构体, 期望值, 实际值, 周期时间)gyro_z_meas
            LADRC_1st_Update(&gyro_y_adrc, pitch_pid.output, gyro_y_meas, TIME_DELAY, MAX_DUTY);
            LADRC_1st_Update(&gyro_x_adrc, roll_pid.output, gyro_x_meas, TIME_DELAY, MAX_DUTY);
            LADRC_1st_Update(&gyro_z_adrc, yaw_pid.output, gyro_z_meas, TIME_DELAY, 20);

            out_pitch = compare_float(gyro_y_adrc.u, -MAX_DUTY * 100.0f, MAX_DUTY * 100.0f);
            out_roll = compare_float(gyro_x_adrc.u, -MAX_DUTY * 100.0f, MAX_DUTY * 100.0f);
            out_yaw = compare_float(gyro_z_adrc.u, -20 * 100.0f, 20 * 100.0f);

            // typedef struct
            // {
            //     uint8 head;      // 帧头
            //     uint8 device_id; // 设备id

            //     int16 upflow302_x;    // 光流_x
            //     int16 upflow302_y;    // 光流_y
            //     int16 upflow302_us;   // 光流_时间差
            //     int16 upflow302_us_a; // 预留位

            //     uint8 upflow302_valid;   // 光流_状态 0不可用  245  可用
            //     uint8 upflow302_version; // 光流_版本号

            //     uint8 sum_check; // 和校验
            //     uint8 sum_end;   // 和校验

            // } upflow302_receive_struct;
            // 等待光流稳定

            // send_uart_motol(global_output - out_roll - out_pitch - out_yaw, // 电机1  成品
            //                 global_output - out_roll + out_pitch + out_yaw, // 电机2
            //                 global_output + out_roll - out_pitch + out_yaw, // 电机3
            //                 global_output + out_roll + out_pitch - out_yaw  // 电机4
            // );
            // send_uart_motol(global_output + out_roll + out_pitch - out_yaw, // 电机1  自制
            //                 global_output - out_roll + out_pitch + out_yaw, // 电机2
            //                 global_output - out_roll - out_pitch - out_yaw, // 电机3
            //                 global_output + out_roll - out_pitch + out_yaw  // 电机4
            // );
            send_uart_motol(global_output + out_roll - out_pitch + out_yaw, // 电机1  当前
                            global_output - out_roll - out_pitch - out_yaw, // 电机2
                            global_output + out_roll + out_pitch - out_yaw, // 电机3
                            global_output - out_roll + out_pitch + out_yaw  // 电机4
            );
        }
        else /* if (dl1a_distance_mm < 150 && lora3a22_uart_transfer.switch_key[2] == 0)*/
        {
            send_uart_motol(global_output, // 电机1
                            global_output, // 电机2
                            global_output, // 电机3
                            global_output  // 电机4
            );
        }

        if (0)
        {
        land:
            // static uint16 land_count = 0;
            //  俯仰角
            // pitch_pid.desire = -lora3a22_uart_transfer.joystick[2] / 100;          // 目标值为水平
            pitch_pid.desire = -compare_float(data_arr[1] / 10, -3.0f, 3.0f);
            pitch_pid.measure = imu660rc_pitch;                                    // 当前俯仰角
            float gyro_y_meas = (imu660rc_gyro_y / imu660rc_transition_factor[1]); // 当前Y轴角速度

            // 横滚角
            // roll_pid.desire = -lora3a22_uart_transfer.joystick[3] / 100;           // 目标值为水平
            roll_pid.desire = -compare_float((data_arr[2] / 10), -3.0f, 3.0f);
            roll_pid.measure = imu660rc_roll;                                      // 当前横滚角
            float gyro_x_meas = (imu660rc_gyro_x / imu660rc_transition_factor[1]); // 当前X轴角速度

            yaw_pid.desire = imu660rc_yaw;
            yaw_pid.measure = 0;
            // 算出两者的原始差值
            float yaw_diff = yaw_pid.desire - yaw_pid.measure;

            if (yaw_diff > 180.0f)
            {
                yaw_pid.measure += 360.0f;
            }
            else if (yaw_diff < -180.0f)
            {
                yaw_pid.measure -= 360.0f;
            }

            float gyro_z_meas = (imu660rc_gyro_z / imu660rc_transition_factor[1]); // 当前Z轴角速度

            // 距离
            static float filtered_gyro_distance_mm = 0;
            filtered_gyro_distance_mm = cosf(PI / 180 * imu660rc_roll) * filtered_distance_mm * cosf(PI / 180 * imu660rc_pitch); // 根据姿态调整距离测量值

            // 垂直方向速度和滤波
            float raw_vel_mm_s = (filtered_gyro_distance_mm - last_filtered_distance_mm) / (TIME_DELAY * 50); // tof频率是50hz
            current_vel_mm_s = 0.8f * raw_vel_mm_s + 0.2f * current_vel_mm_s;
            last_filtered_distance_mm = filtered_gyro_distance_mm;

            // 一毫秒累加一次
            static uint8 distance_count = 0;
            if (distance_count < 15)
            {
                distance_count++;
            }
            else if (distance_count >= 15)
            {
                distance_count = 0;
                // distance_pid.desire += lora3a22_uart_transfer.joystick[1] / 500;
                distance_pid.desire -= 2.4;
            }

            distance_pid.desire = compare_float(distance_pid.desire, -600.0f, 1700.0f);

            // global_output += lora3a22_uart_transfer.joystick[1]/1000;

            // 外环使用P控制
            PID_Calc(&pitch_pid);
            PID_Calc(&roll_pid);
            PID_Calc(&yaw_pid);

            float err_distance = distance_pid.desire - filtered_gyro_distance_mm; // 计算原始误差

            if (err_distance > 20.0f)
            {
                distance_pid.measure = distance_pid.desire - (err_distance - 20.0f);
            }
            else if (err_distance < -20.0f)
            {
                distance_pid.measure = distance_pid.desire - (err_distance + 20.0f);
            }
            else
            {

                distance_pid.measure = distance_pid.desire;
            }
            // distance_pid.measure = filtered_gyro_distance_mm; // 外环测量值
            velocity_pid.measure = current_vel_mm_s; // 内环测量值
            PID_Calc_chain(&distance_pid, &velocity_pid);

            // 最终全局油门 = 悬停基准值 + 串级内环的输出补偿量
            global_output = base_hover_throttle + velocity_pid.output;
            global_output = compare_float(global_output, MIN_DUTY * 100.0f, 95 * 100.0f);

            // 全局油门低通滤波
            // global_output = 0.6f * global_output + 0.4f * last_global_output;
            // last_global_output = global_output;
            // global_output =1000;

            // 作用给电机
            // 三者为叠加关系 (根据陀螺仪和四旋翼的方位关系来调整)
            //   电机1 =  俯仰角PID输出 + 横滚角PID输出 + 偏航角PID输出
            //   电机2 =  俯仰角PID输出 - 横滚角PID输出 - 偏航角PID输出
            //   电机3 = -俯仰角PID输出 + 横滚角PID输出 - 偏航角PID输出
            //   电机4 = -俯仰角PID输出 - 横滚角PID输出 + 偏航角PID输出
            //   逆时针1111********2222顺时针
            //         1111********2222
            //         ****************
            //         ****************
            //   顺时针3333********4444逆时针
            //         3333********4444

            // send_uart_motol(global_output + out_pitch + out_roll + out_yaw, // 电机1
            //                 global_output + out_pitch - out_roll - out_yaw, // 电机2
            //                 global_output - out_pitch + out_roll - out_yaw, // 电机3
            //                 global_output - out_pitch - out_roll + out_yaw  // 电机4
            // );

            // send_uart_motol(global_output - out_roll - out_pitch, // 电机1
            //                 global_output - out_roll + out_pitch, // 电机2
            //                 global_output + out_roll - out_pitch, // 电机3
            //                 global_output + out_roll + out_pitch  // 电机4
            // );

            if (vl53l8cx_distance_mm >= 150 /*|| lora3a22_uart_transfer.switch_key[2] == 1*/)
            {
                // LADRC_1st_Update(adrc结构体, 期望值, 实际值, 周期时间)gyro_z_meas
                LADRC_1st_Update(&gyro_y_adrc, pitch_pid.output, gyro_y_meas, TIME_DELAY, MAX_DUTY);
                LADRC_1st_Update(&gyro_x_adrc, roll_pid.output, gyro_x_meas, TIME_DELAY, MAX_DUTY);
                LADRC_1st_Update(&gyro_z_adrc, yaw_pid.output, gyro_z_meas, TIME_DELAY, 20);

                out_pitch = compare_float(gyro_y_adrc.u, -MAX_DUTY * 100.0f, MAX_DUTY * 100.0f);
                out_roll = compare_float(gyro_x_adrc.u, -MAX_DUTY * 100.0f, MAX_DUTY * 100.0f);
                out_yaw = compare_float(gyro_z_adrc.u, -20 * 100.0f, 20 * 100.0f);

                // typedef struct
                // {
                //     uint8 head;      // 帧头
                //     uint8 device_id; // 设备id

                //     int16 upflow302_x;    // 光流_x
                //     int16 upflow302_y;    // 光流_y
                //     int16 upflow302_us;   // 光流_时间差
                //     int16 upflow302_us_a; // 预留位

                //     uint8 upflow302_valid;   // 光流_状态 0不可用  245  可用
                //     uint8 upflow302_version; // 光流_版本号

                //     uint8 sum_check; // 和校验
                //     uint8 sum_end;   // 和校验

                // } upflow302_receive_struct;
                // 等待光流稳定

                // send_uart_motol(global_output - out_roll - out_pitch - out_yaw, // 电机1  成品
                //                 global_output - out_roll + out_pitch + out_yaw, // 电机2
                //                 global_output + out_roll - out_pitch + out_yaw, // 电机3
                //                 global_output + out_roll + out_pitch - out_yaw  // 电机4
                // );
                // send_uart_motol(global_output + out_roll + out_pitch - out_yaw, // 电机1  自制
                //                 global_output - out_roll + out_pitch + out_yaw, // 电机2
                //                 global_output - out_roll - out_pitch - out_yaw, // 电机3
                //                 global_output + out_roll - out_pitch + out_yaw  // 电机4
                // );
                send_uart_motol(global_output + out_roll - out_pitch + out_yaw, // 电机1  当前
                                global_output - out_roll - out_pitch - out_yaw, // 电机2
                                global_output + out_roll + out_pitch - out_yaw, // 电机3
                                global_output - out_roll + out_pitch + out_yaw  // 电机4
                );
            }
            else /* if (dl1a_distance_mm < 150 && lora3a22_uart_transfer.switch_key[2] == 0)*/
            {
                // send_uart_motol(0, // 电机1
                //                 0, // 电机2
                //                 0, // 电机3
                //                 0  // 电机4
                // );
                small_driver_set_duty(0, 0, 0, 0);
            }
        }
    }
    else if (lora3a22_uart_transfer.switch_key[1] == 0)
    {
        small_driver_set_duty(0, 0, 0, 0);
        // 归零误差和输出
        gyro_y_adrc.z1 = 0;
        gyro_y_adrc.z2 = 0;
        gyro_x_adrc.z1 = 0;
        gyro_x_adrc.z2 = 0;
        gyro_z_adrc.z1 = 0;
        gyro_z_adrc.z2 = 0;
        gyro_y_adrc.u = 0;
        gyro_x_adrc.u = 0;
        gyro_z_adrc.u = 0;

        // 归零PID
        pitch_pid.integral = 0;
        roll_pid.integral = 0;
        yaw_pid.integral = 0;
        distance_pid.integral = 0;

        velocity_pid.integral = 0;

        pitch_pid.last_err = 0;
        roll_pid.last_err = 0;
        yaw_pid.last_err = 0;
        distance_pid.last_err = 0;

        velocity_pid.last_err = 0;
        current_vel_mm_s = 0;
        yaw_pid.measure = imu660rc_yaw;
        last_filtered_distance_mm = vl53l8cx_distance_mm; // 同步当前高度防起飞突变
        filtered_distance_mm = vl53l8cx_distance_mm;      // 同步滤波状态，防起飞突变
        current_vel_mm_s = 0;                             // 归零垂直速度
        tof_new_flag = 0;                                 // 丢弃悬挂的旧数据标志
        last_tof_ms = isr_ms;                             // 重置 dt 基准
        // SCB_CleanInvalidateDCache_by_Addr(&beacon_lost, sizeof(beacon_lost));
        // printf("%d\n" , beacon_lost);
        // SCB_CleanInvalidateDCache_by_Addr(car_position, sizeof(car_position));
        // printf("%d, %d\n", car_position[0], car_position[1]);
        // SCB_CleanInvalidateDCache_by_Addr(data_arr, sizeof(data_arr));
        // printf("%d, %0.2f, %0.2f\n", (int)data_arr[0], data_arr[1], data_arr[2]);
        if (lora3a22_uart_transfer.switch_key[3] == 1)
        {
            send_uart_motol(2000, // 电机1
                            2000, // 电机2
                            2000, // 电机3
                            2000  // 电机4
            );
        }
    }

    // printf("%d\n", lora3a22_uart_transfer.switch_key[1]);
}

void pit0_ch1_isr() // 定时器通道 1 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);
    // dl1a_get_distance();
    //  vl53l8cx_get_distance();

    // upflow302_receive_callback();
}

void pit0_ch2_isr() // 定时器通道 2 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH2);
}

void pit0_ch10_isr() // 定时器通道 10 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH10);
}

void pit0_ch11_isr() // 定时器通道 11 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH11);
}

void pit0_ch12_isr() // 定时器通道 12 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH12);
}

void pit0_ch13_isr() // 定时器通道 13 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH13);
}

void pit0_ch14_isr() // 定时器通道 14 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH14);
}

void pit0_ch15_isr() // 定时器通道 15 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH15);
}

void pit0_ch16_isr() // 定时器通道 16 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH16);
}

void pit0_ch17_isr() // 定时器通道 17 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH17);
}

void pit0_ch18_isr() // 定时器通道 18 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH18);
}

void pit0_ch19_isr() // 定时器通道 19 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH19);
}

void pit0_ch20_isr() // 定时器通道 20 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH20);
}

void pit0_ch21_isr() // 定时器通道 21 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************

// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr(void)
{
    if (uart_isr_mask(UART_0)) // 串口0接收中断
    {

#if DEBUG_UART_USE_INTERRUPT       // 如果开启 debug 串口中断
        debug_interrupr_handler(); // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                             // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
    }
    else // 串口0发送中断
    {
    }
}

void uart1_isr(void)
{
    if (uart_isr_mask(UART_1)) // 串口1接收中断
    {

        wireless_module_uart_handler(); // 无线模块统一回调函数
    }
    else // 串口1发送中断
    {
    }
}

void uart2_isr(void)
{
    if (uart_isr_mask(UART_2)) // 串口2接收中断
    {
        gnss_uart_callback(); // GPS模块回调函数
    }
    else // 串口2发送中断
    {
    }
}

void uart3_isr(void)
{
    if (uart_isr_mask(UART_3)) // 串口3接收中断
    {
    }
    else // 串口3发送中断
    {
    }
}

void uart4_isr(void)
{
    if (uart_isr_mask(UART_4)) // 串口4接收中断
    {

        uart_receiver_handler(); // 串口接收机回调函数
    }
    else // 串口4发送中断
    {
    }
}

void uart5_isr(void)
{
    if (uart_isr_mask(UART_5)) // 串口5接收中断
    {
    }
    else // 串口5发送中断
    {
    }
}

void uart6_isr(void)
{
    if (uart_isr_mask(UART_6)) // 串口6接收中断
    {

        wireless_module_uart_handler();
    }
    else // 串口6发送中断
    {
    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr() // 外部 GPIO_0 中断服务函数
{
}

void gpio_1_exti_isr() // 外部 GPIO_1 中断服务函数
{
    if (exti_flag_get(P01_0)) // 示例P1_0端口外部中断判断
    {
    }
    if (exti_flag_get(P01_1))
    {
    }
}

void gpio_2_exti_isr() // 外部 GPIO_2 中断服务函数
{
    if (exti_flag_get(P02_0))
    {
    }
    if (exti_flag_get(P02_4))
    {
    }
}

void gpio_3_exti_isr() // 外部 GPIO_3 中断服务函数
{
}

void gpio_4_exti_isr() // 外部 GPIO_4 中断服务函数
{
}

void gpio_5_exti_isr() // 外部 GPIO_5 中断服务函数
{
}

void gpio_6_exti_isr() // 外部 GPIO_6 中断服务函数
{
}

void gpio_7_exti_isr() // 外部 GPIO_7 中断服务函数
{
}

void gpio_8_exti_isr() // 外部 GPIO_8 中断服务函数
{
}

void gpio_9_exti_isr() // 外部 GPIO_9 中断服务函数
{
}

void gpio_10_exti_isr() // 外部 GPIO_10 中断服务函数
{
}

void gpio_11_exti_isr() // 外部 GPIO_11 中断服务函数
{
}

void gpio_12_exti_isr() // 外部 GPIO_12 中断服务函数
{
}

void gpio_13_exti_isr() // 外部 GPIO_13 中断服务函数
{
}

void gpio_14_exti_isr() // 外部 GPIO_14 中断服务函数
{

    if (exti_flag_get(P14_5))
    {
        imu660rc_callback();
    }
}

void gpio_15_exti_isr() // 外部 GPIO_15 中断服务函数
{
}

void gpio_16_exti_isr() // 外部 GPIO_16 中断服务函数
{
}

void gpio_17_exti_isr() // 外部 GPIO_17 中断服务函数
{
}

void gpio_18_exti_isr() // 外部 GPIO_18 中断服务函数
{
}

void gpio_19_exti_isr() // 外部 GPIO_19 中断服务函数
{
}

void gpio_20_exti_isr() // 外部 GPIO_20 中断服务函数
{
}

void gpio_21_exti_isr() // 外部 GPIO_21 中断服务函数
{
}

void gpio_22_exti_isr() // 外部 GPIO_22 中断服务函数
{
}

void gpio_23_exti_isr() // 外部 GPIO_23 中断服务函数
{
}
// **************************** 外部中断函数 ****************************

void send_uart_motol(float duty1, float duty2, float duty3, float duty4)
{
    // 取负因为pid输出反了
    // output_duty1 = -duty1 * (PWM_DUTY_MAX / 100); // 计算输出占空比
    // output_duty2 = -duty2 * (PWM_DUTY_MAX / 100); // 计算输出占空比
    // output_duty3 = -duty3 * (PWM_DUTY_MAX / 100); // 计算输出占空比
    // output_duty4 = -duty4 * (PWM_DUTY_MAX / 100); // 计算输出占空比
    output_duty1 = compare_float(duty1, MIN_DUTY * 100.0f, MAX_DUTY * 100.0f); // 计算输出占空比
    output_duty2 = compare_float(duty2, MIN_DUTY * 100.0f, MAX_DUTY * 100.0f); // 计算输出占空比
    output_duty3 = compare_float(duty3, MIN_DUTY * 100.0f, MAX_DUTY * 100.0f); // 计算输出占空比
    output_duty4 = compare_float(duty4, MIN_DUTY * 100.0f, MAX_DUTY * 100.0f); // 计算输出占空比

    output_duty1 = dir1 ? output_duty1 : -output_duty1; // 根据方向调整占空比
    output_duty2 = dir2 ? output_duty2 : -output_duty2; // 根据方向调整占空比
    output_duty3 = dir3 ? output_duty3 : -output_duty3; // 根据方向调整占空比
    output_duty4 = dir4 ? output_duty4 : -output_duty4; // 根据方向调整占空比

    // printf("duty: %5d  ,  %5d  ,  %5d  ,  %5d  \r\n", output_duty1, output_duty2, output_duty3, output_duty4);

    small_driver_set_duty(output_duty1, output_duty2, output_duty3, output_duty4);
}
