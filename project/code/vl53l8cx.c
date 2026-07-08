#include "zf_common_debug.h"

#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"
#include "vl53l8cx/vl53l8cx_api.h"
#include "vl53l8cx.h"


//=================================================定义 VL53L8CX 模块 全局变量================================================
uint8  vl53l8cx_finsh_flag = 0;
uint16 vl53l8cx_distance_mm = 0;
//=================================================定义 VL53L8CX 模块 全局变量================================================


//=================================================定义 VL53L8CX 模块 内部变量================================================
#if (VL53L8CX_COMM_MODE == 0)
soft_iic_info_struct          vl53l8cx_iic_obj;                                 // 软件 IIC 结构体
#endif
static VL53L8CX_Configuration vl53l8cx_dev;                                     // VL53L8CX 驱动配置结构体
static VL53L8CX_ResultsData   vl53l8cx_results;                                 // 测距结果缓冲
static uint8                  vl53l8cx_init_flag = 0;                           // 初始化成功标志
static uint8                  vl53l8cx_resolution = VL53L8CX_RESOLUTION_4X4;    // 当前分辨率
//=================================================定义 VL53L8CX 模块 内部变量================================================


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算中值
// 参数说明     data            待排序距离数组
// 参数说明     len             数组长度
// 返回参数     int16           中值距离
// 使用示例     vl53l8cx_distance_median(valid_dist, valid_count);
// 备注信息     用于抑制少数小面积近物（如电源线）对整体高度的影响
//-------------------------------------------------------------------------------------------------------------------
static int16_t vl53l8cx_distance_median (int16_t *data, uint16_t len)
{
    uint16_t i, j;
    int16_t key;

    if (1 == len)
    {
        return data[0];
    }

    // 插入排序
    for (i = 1; i < len; i ++)
    {
        key = data[i];
        j = i;
        while ((j > 0) && (data[j - 1] > key))
        {
            data[j] = data[j - 1];
            j --;
        }
        data[j] = key;
    }

    if (len & 0x01)
    {
        return data[len / 2];
    }
    return ((data[len / 2 - 1] + data[len / 2]) / 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取融合后的距离
// 参数说明     无
// 返回参数     无
// 使用示例     vl53l8cx_get_distance();
// 备注信息     轮询更新；最终距离为所有有效区域距离的中值，可防止小面积近物干扰定高
//-------------------------------------------------------------------------------------------------------------------
void vl53l8cx_get_distance (void)
{
    if (!vl53l8cx_init_flag)
    {
        return;
    }

    uint8_t             is_ready = 0;
    uint8_t             status;
    int16_t             valid_dist[VL53L8CX_RESOLUTION_8X8];
    uint16_t            valid_count = 0;
    uint8_t             i;
    uint8_t             target_status;
    int16_t             distance_temp;

    status = vl53l8cx_check_data_ready(&vl53l8cx_dev, &is_ready);
    if ((0 != status) || (0 == is_ready))
    {
        return;
    }

    status = vl53l8cx_get_ranging_data(&vl53l8cx_dev, &vl53l8cx_results);
    if (0 != status)
    {
        return;
    }

    for (i = 0; i < vl53l8cx_resolution; i ++)
    {
        target_status = vl53l8cx_results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i];
        if ((5 == target_status) || (9 == target_status))
        {
            distance_temp = vl53l8cx_results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i];
            if (distance_temp >= 0)
            {
                valid_dist[valid_count ++] = distance_temp;
            }
        }
    }

    if (0 == valid_count)
    {
        vl53l8cx_finsh_flag = 0;
        return;
    }

    vl53l8cx_distance_mm = (uint16_t)vl53l8cx_distance_median(valid_dist, valid_count);
    vl53l8cx_finsh_flag  = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VL53L8CX INT 中断响应函数
// 参数说明     无
// 返回参数     无
// 使用示例     vl53l8cx_int_handler();
//-------------------------------------------------------------------------------------------------------------------
void vl53l8cx_int_handler (void)
{
#if VL53L8CX_INT_ENABLE
    vl53l8cx_get_distance();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 VL53L8CX
// 参数说明     无
// 返回参数     uint8           0-成功，1-检测失败，2-固件加载失败，3-分辨率设置失败，4-目标顺序设置失败，5-测距频率设置失败，6-启动测距失败
// 使用示例     vl53l8cx_device_init();
// 备注信息     加载固件需要几百毫秒，会阻塞
//-------------------------------------------------------------------------------------------------------------------
uint8 vl53l8cx_device_init (void)
{
    uint8_t status = 0;
    uint8_t is_alive = 0;

#if (VL53L8CX_COMM_MODE == 0)
    soft_iic_init(&vl53l8cx_iic_obj, VL53L8CX_DEV_ADDR, VL53L8CX_SOFT_IIC_DELAY,
                  VL53L8CX_SCL_PIN, VL53L8CX_SDA_PIN);
#elif (VL53L8CX_COMM_MODE == 1)
    vl53l8cx_iic_hardware_init();
#elif (VL53L8CX_COMM_MODE == 2)
    /* SPI 初始化：SPI1, Mode 3, P12 引脚组 */
    spi_init(VL53L8CX_SPI_INDEX, VL53L8CX_SPI_MODE, VL53L8CX_SPI_SPEED_HZ,
             VL53L8CX_SPI_CLK_PIN, VL53L8CX_SPI_MOSI_PIN, VL53L8CX_SPI_MISO_PIN, VL53L8CX_SPI_NCS_PIN);
#endif

    gpio_init(VL53L8CX_LPN_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    do
    {
        system_delay_ms(50);
        gpio_low(VL53L8CX_LPN_PIN);
        system_delay_ms(10);
        gpio_high(VL53L8CX_LPN_PIN);
        system_delay_ms(50);

        vl53l8cx_dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

#if (VL53L8CX_COMM_MODE == 2)
        // [新增]：强行发一次空读，利用 NCS 的下降沿将传感器锁死在 SPI 模式
        uint8_t dummy_val;
        VL53L8CX_RdByte(&vl53l8cx_dev.platform, 0x7FFF, &dummy_val);
        system_delay_ms(2);
#endif

        status = vl53l8cx_is_alive(&vl53l8cx_dev, &is_alive);
        if ((0 != status) || (0 == is_alive))
        {
            zf_log(0, "VL53L8CX not alive at default address.");
            status = 1;
            break;
        }

        status = vl53l8cx_init(&vl53l8cx_dev);
        if (0 != status)
        {
            zf_log(0, "VL53L8CX firmware load failed.");
            status = 2;
            break;
        }

        status = vl53l8cx_set_resolution(&vl53l8cx_dev, VL53L8CX_RESOLUTION_4X4);
        if (0 != status)
        {
            zf_log(0, "VL53L8CX set resolution failed.");
            status = 3;
            break;
        }

        status = vl53l8cx_set_target_order(&vl53l8cx_dev, VL53L8CX_TARGET_ORDER_CLOSEST);
        if (0 != status)
        {
            zf_log(0, "VL53L8CX set target order failed.");
            status = 4;
            break;
        }

        status = vl53l8cx_set_ranging_frequency_hz(&vl53l8cx_dev, VL53L8CX_RANGING_FREQ_HZ);
        if (0 != status)
        {
            zf_log(0, "VL53L8CX set ranging frequency failed.");
            status = 5;
            break;
        }

        status = vl53l8cx_start_ranging(&vl53l8cx_dev);
        if (0 != status)
        {
            zf_log(0, "VL53L8CX start ranging failed.");
            status = 6;
            break;
        }

        vl53l8cx_get_resolution(&vl53l8cx_dev, &vl53l8cx_resolution);
        vl53l8cx_init_flag = 1;
        status = 0;
    } while (0);

#if VL53L8CX_INT_ENABLE
    exti_init(VL53L8CX_INT_PIN, EXTI_TRIGGER_FALLING);
    vl53l8cx_int_handler();
    vl53l8cx_finsh_flag = 0;
#endif

    return status;
}
