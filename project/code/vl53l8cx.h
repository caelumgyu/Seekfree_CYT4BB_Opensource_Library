#ifndef _ZF_DEVICE_VL53L8CX_H_
#define _ZF_DEVICE_VL53L8CX_H_

#include "zf_common_typedef.h"

//=================================================定义 VL53L8CX 模块 通信方式================================================
// 0 = 软件 IIC,  1 = 硬件 IIC,  2 = 硬件 SPI
#define VL53L8CX_COMM_MODE                                  (1)

//=================================================定义 VL53L8CX 模块 配置参数================================================
#if (VL53L8CX_COMM_MODE == 0)   // 软件 IIC
//====================================================定义 软件 IIC 参数====================================================
#define VL53L8CX_SOFT_IIC_DELAY                             (100)               // 软件 IIC 延时，数值越小通信速率越高
#define VL53L8CX_SCL_PIN                                    (P05_2)             // 软件 IIC SCL 引脚
#define VL53L8CX_SDA_PIN                                    (P05_1)             // 软件 IIC SDA 引脚
//====================================================定义 软件 IIC 参数====================================================
#elif (VL53L8CX_COMM_MODE == 1) // 硬件 IIC
//====================================================定义 硬件 IIC 参数====================================================
// SCB5, P07_2(SCL)、P07_1(SDA)，需要外接上拉电阻。P07_1/2→SCB5, P06_1/2→SCB4, P18_1/2→SCB1
#define VL53L8CX_HARD_IIC_SCB_SEL                            (5)                 // SCB 编号：1=SCB1(P18), 4=SCB4(P06), 5=SCB5(P07)
#define VL53L8CX_HARD_IIC_SCL_PIN                           (P07_2)
#define VL53L8CX_HARD_IIC_SDA_PIN                           (P07_1)
#define VL53L8CX_HARD_IIC_SPEED_HZ                          (400000)            // IIC 速率 400kHz
//====================================================定义 硬件 IIC 参数====================================================
#elif (VL53L8CX_COMM_MODE == 2) // 硬件 SPI
//====================================================定义 硬件 SPI 参数====================================================
// SPI1, SCB8, P12 引脚组，Mode 3 (CPOL=1,CPHA=1)，最大 3MHz
#define VL53L8CX_SPI_INDEX                                 (SPI_1)             // SPI 模块索引
#define VL53L8CX_SPI_MODE                                  (SPI_MODE3)         // SPI 模式 3
#define VL53L8CX_SPI_SPEED_HZ                              (1*1000*1000)       // SPI 时钟 3 MHz
#define VL53L8CX_SPI_NCS_PIN                               (SPI1_CS0_P12_3)   // SPI CS 引脚
#define VL53L8CX_SPI_MISO_PIN                              (SPI1_MISO_P12_0)  // SPI MISO 引脚
#define VL53L8CX_SPI_MOSI_PIN                              (SPI1_MOSI_P12_1)  // SPI MOSI 引脚
#define VL53L8CX_SPI_CLK_PIN                               (SPI1_CLK_P12_2)   // SPI SCK 引脚
//====================================================定义 硬件 SPI 参数====================================================
#endif

#define VL53L8CX_LPN_PIN                                    (P05_3)             // LPN / XSHUT 复位引脚

#define VL53L8CX_INT_ENABLE                                 (0)                 // 是否开启 INT 引脚中断，0-轮询模式
#if VL53L8CX_INT_ENABLE
#define VL53L8CX_INT_PIN                                    (P05_4)
#endif

#define VL53L8CX_TIMEOUT_COUNT                              (1000)              // 等待超时计数

#define VL53L8CX_DEV_ADDR                                   (0x52 >> 1)         // 7 位 IIC 设备地址（SPI 模式下不使用）

#define VL53L8CX_RANGING_FREQ_HZ                            (60)                // 测距频率 Hz（4x4 模式下合法范围 1~60）
//=================================================定义 VL53L8CX 模块 配置参数================================================


//=================================================定义 VL53L8CX 模块 全局变量================================================
extern uint8  vl53l8cx_finsh_flag;                          // 测距完成标志位
extern uint16 vl53l8cx_distance_mm;                         // 融合后的距离，单位毫米
//=================================================定义 VL53L8CX 模块 全局变量================================================


//=================================================定义 VL53L8CX 模块 接口函数================================================
void   vl53l8cx_get_distance   (void);                        // 轮询更新，获取融合后的单距离
void   vl53l8cx_int_handler    (void);                        // INT 中断响应函数
uint8  vl53l8cx_device_init    (void);                        // 初始化 VL53L8CX，0-成功，1-检测失败，2-固件加载失败，3-分辨率设置失败，4-目标顺序设置失败，5-测距频率设置失败，6-启动测距失败
#if (VL53L8CX_COMM_MODE == 1)
void   vl53l8cx_iic_hardware_init (void);                     // 硬件 IIC 初始化
#endif
//=================================================定义 VL53L8CX 模块 接口函数================================================

#endif
