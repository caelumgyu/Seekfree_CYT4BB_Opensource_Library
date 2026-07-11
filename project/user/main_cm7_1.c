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
 * 文件名称          main_cm7_1
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
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

#define width MT9V03X_W
#define height MT9V03X_H
#define center_point_w (width / 2)
#define center_point_h (height / 2)
#define max_scan_line (width * height)

#define THRESH_LOW 80
#define THRESH_MID 120
#define THRESH_CAR 200
#define THRESH_HIGH 255

#define WIFI_OPEN 1
#define WIFI_SSID_TEST "arch"
#define WIFI_PASSWORD_TEST "111555999"
#define TCP_TARGET_IP "10.215.162.119"
#define TCP_TARGET_PORT "8086"
#define WIFI_LOCAL_PORT "6666"

uint8 image_arr[height][width] = {0};
bool visited[height][width] = {false};
uint8 image_copy[height][width];
bool beacon_loast = false;

typedef struct
{
    int area;
    int sum_weight;
    int sum_x;
    int sum_y;
    int64_t sum_x2;
    int64_t sum_y2;
    int64_t sum_xy;
    int min_x, max_x;
    int min_y, max_y;
    float cx, cy;

    float mu20, mu02, mu11;

    float dir_x, dir_y;
} Blob;

typedef struct
{
    float x;
    float y;
} PointF;

typedef struct
{
    uint8 x1, x2, y;
} Span;

float distance_c(PointF a, PointF b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

float point_to_line_dist_c(PointF p, PointF p1, PointF p2, PointF *foot)
{
    float vx = p2.x - p1.x;
    float vy = p2.y - p1.y;
    float wx = p.x - p1.x;
    float wy = p.y - p1.y;

    float c1 = wx * vx + wy * vy;
    float c2 = vx * vx + vy * vy;

    if (c2 < 1e-6f)
    {
        *foot = p1;
        return distance_c(p, p1);
    }

    float t = c1 / c2;
    foot->x = p1.x + t * vx;
    foot->y = p1.y + t * vy;

    return distance_c(p, *foot);
}

typedef struct
{
    bool locked;
    int last_x;
    int last_y;
    int vx;
    int vy;
    int roi_radius;
    int search_min_x, search_max_x;
    int search_min_y, search_max_y;
} BeaconTracker;

Blob find_blob(uint8 start_x, uint8 start_y, uint8 min_val, uint8 max_val)
{
    Blob blob = {0};
    blob.min_x = start_x;
    blob.max_x = start_x;
    blob.min_y = start_y;
    blob.max_y = start_y;

    Span stack[1024];
    int stack_top = 0;
    stack[stack_top++] = (Span){(uint8)start_x, (uint8)start_x, (uint8)start_y};

    visited[start_y][start_x] = true;
    uint8 val = image_arr[start_y][start_x];

    blob.area++;
    blob.sum_weight += val;
    blob.sum_x += start_x * val;
    blob.sum_y += start_y * val;
    blob.sum_x2 += start_x * start_x * val;
    blob.sum_y2 += start_y * start_y * val;
    blob.sum_xy += start_x * start_y * val;

    while (stack_top > 0)
    {
        Span span = stack[--stack_top];
        uint8 y = span.y;
        uint8 xleft = span.x1;
        uint8 xright = span.x2;

        while (xleft > 0 && !visited[y][xleft - 1] && image_arr[y][xleft - 1] >= min_val && image_arr[y][xleft - 1] <= max_val)
        {
            xleft--;
            visited[y][xleft] = true;
            val = image_arr[y][xleft];
            blob.area++;
            blob.sum_weight += val;
            blob.sum_x += xleft * val;
            blob.sum_y += y * val;
            blob.sum_x2 += xleft * xleft * val;
            blob.sum_y2 += y * y * val;
            blob.sum_xy += xleft * y * val;
            if (xleft < blob.min_x)
                blob.min_x = xleft;
            if (xleft > blob.max_x)
                blob.max_x = xleft;
            if (y < blob.min_y)
                blob.min_y = y;
            if (y > blob.max_y)
                blob.max_y = y;
        }

        while (xright < width - 1 && !visited[y][xright + 1] && image_arr[y][xright + 1] >= min_val && image_arr[y][xright + 1] <= max_val)
        {
            xright++;
            visited[y][xright] = true;
            val = image_arr[y][xright];
            blob.area++;
            blob.sum_weight += val;
            blob.sum_x += xright * val;
            blob.sum_y += y * val;
            blob.sum_x2 += xright * xright * val;
            blob.sum_y2 += y * y * val;
            blob.sum_xy += xright * y * val;
            if (xright < blob.min_x)
                blob.min_x = xright;
            if (xright > blob.max_x)
                blob.max_x = xright;
            if (y < blob.min_y)
                blob.min_y = y;
            if (y > blob.max_y)
                blob.max_y = y;
        }

        if (y > 0)
        {
            uint8 i = xleft;
            while (i <= xright)
            {
                if (!visited[y - 1][i] && image_arr[y - 1][i] >= min_val && image_arr[y - 1][i] <= max_val)
                {
                    uint8 span_start = i;
                    while (i <= xright && !visited[y - 1][i] && image_arr[y - 1][i] >= min_val && image_arr[y - 1][i] <= max_val)
                    {
                        visited[y - 1][i] = true;
                        val = image_arr[y - 1][i];
                        blob.area++;
                        blob.sum_weight += val;
                        blob.sum_x += i * val;
                        blob.sum_y += (y - 1) * val;
                        blob.sum_x2 += i * i * val;
                        blob.sum_y2 += (y - 1) * (y - 1) * val;
                        blob.sum_xy += i * (y - 1) * val;
                        if (i < blob.min_x)
                            blob.min_x = i;
                        if (i > blob.max_x)
                            blob.max_x = i;
                        if (y - 1 < blob.min_y)
                            blob.min_y = y - 1;
                        if (y - 1 > blob.max_y)
                            blob.max_y = y - 1;
                        i++;
                    }
                    stack[stack_top++] = (Span){span_start, (uint8)(i - 1), (uint8)(y - 1)};
                }
                else
                {
                    i++;
                }
            }
        }

        if (y < height - 1)
        {
            uint8 i = xleft;
            while (i <= xright)
            {
                if (!visited[y + 1][i] && image_arr[y + 1][i] >= min_val && image_arr[y + 1][i] <= max_val)
                {
                    uint8 span_start = i;
                    while (i <= xright && !visited[y + 1][i] && image_arr[y + 1][i] >= min_val && image_arr[y + 1][i] <= max_val)
                    {
                        visited[y + 1][i] = true;
                        val = image_arr[y + 1][i];
                        blob.area++;
                        blob.sum_weight += val;
                        blob.sum_x += i * val;
                        blob.sum_y += (y + 1) * val;
                        blob.sum_x2 += i * i * val;
                        blob.sum_y2 += (y + 1) * (y + 1) * val;
                        blob.sum_xy += i * (y + 1) * val;
                        if (i < blob.min_x)
                            blob.min_x = i;
                        if (i > blob.max_x)
                            blob.max_x = i;
                        if (y + 1 < blob.min_y)
                            blob.min_y = y + 1;
                        if (y + 1 > blob.max_y)
                            blob.max_y = y + 1;
                        i++;
                    }
                    stack[stack_top++] = (Span){span_start, (uint8)(i - 1), (uint8)(y + 1)};
                }
                else
                {
                    i++;
                }
            }
        }
    }

    if (blob.sum_weight > 0)
    {
        blob.cx = (float)blob.sum_x / blob.sum_weight;
        blob.cy = (float)blob.sum_y / blob.sum_weight;
    }
    return blob;
}

Blob detect_beacon(BeaconTracker *tracker, Blob *exclude_beacon, Blob *car)
{
    Blob best_beacon = {0};

    if (tracker->locked)
    {
        int predict_x = tracker->last_x + tracker->vx;
        int predict_y = tracker->last_y + tracker->vy;

        // 可能有问题，有问题再说
        tracker->search_min_x = (predict_x - tracker->roi_radius < 0) ? 0 : predict_x - tracker->roi_radius;
        tracker->search_max_x = (predict_x + tracker->roi_radius >= width) ? width - 1 : predict_x + tracker->roi_radius;
        tracker->search_min_y = (predict_y - tracker->roi_radius < 0) ? 0 : predict_y - tracker->roi_radius;
        tracker->search_max_y = (predict_y + tracker->roi_radius >= height) ? height - 1 : predict_y + tracker->roi_radius;
    }
    else
    {
        tracker->search_min_x = 0;
        tracker->search_max_x = width - 1;
        tracker->search_min_y = 0;
        tracker->search_max_y = height - 1;
    }

    for (int y = tracker->search_min_y; y <= tracker->search_max_y; y++)
    {
        for (int x = tracker->search_min_x; x <= tracker->search_max_x; x++)
        {
            if (visited[y][x])
                continue;

            uint8 pixel = image_arr[y][x];
            if (pixel >= THRESH_MID)
            {
                Blob b = find_blob(x, y, THRESH_MID, THRESH_HIGH);

                if (car != NULL && car->area > 0)
                {
                    float dist1 = sqrt((b.cx - car->cx) * (b.cx - car->cx) + (b.cy - car->cy) * (b.cy - car->cy));
                    if (dist1 < 15.0f)
                        continue;
                }

                int w = (b.max_x - b.min_x) + 1;
                int h = (b.max_y - b.min_y) + 1;

                if (b.area > 3 && b.area < 500 && w > 1 && h > 1)
                {

                    if (exclude_beacon != NULL && exclude_beacon->area > 0)
                    {
                        float dist = sqrt((b.cx - exclude_beacon->cx) * (b.cx - exclude_beacon->cx) + (b.cy - exclude_beacon->cy) * (b.cy - exclude_beacon->cy));
                        if (dist < 12.0f)
                        {
                            continue;
                        }
                    }

                    float aspect_ratio = (float)w / h;
                    int bounding_box_area = w * h;
                    float fill_ratio = (float)b.area / bounding_box_area;

                    if (aspect_ratio > 0.5f && aspect_ratio < 2.0f)
                    {
                        if (fill_ratio > 0.6f && fill_ratio < 1.0f)
                        {
                            float box_center_x = b.min_x + w / 2.0f;
                            float box_center_y = b.min_y + h / 2.0f;
                            float center_offset = sqrt((b.cx - box_center_x) * (b.cx - box_center_x) + (b.cy - box_center_y) * (b.cy - box_center_y));

                            if (center_offset < (w * 0.3f))
                            {
                                if (b.area > best_beacon.area)
                                {
                                    best_beacon = b;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (best_beacon.area > 0)
    {
        if (tracker->locked)
        {
            tracker->vx = best_beacon.cx - tracker->last_x;
            tracker->vy = best_beacon.cy - tracker->last_y;
        }
        else
        {
            tracker->vx = 0;
            tracker->vy = 0;
        }
        tracker->locked = true;
        tracker->last_x = best_beacon.cx;
        tracker->last_y = best_beacon.cy;
    }
    else
    {
        tracker->locked = false;
    }

    return best_beacon;
}

Blob detect_car()
{
    Blob car = {0};
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (visited[y][x])
                continue;

            uint8 pixel = image_arr[y][x];
            if (pixel >= THRESH_CAR)
            {
                Blob yb = find_blob(x, y, THRESH_CAR, THRESH_HIGH);

                int w = (yb.max_x - yb.min_x) + 1;
                int h = (yb.max_y - yb.min_y) + 1;

                if (yb.area > 30 && yb.area < 800 && w > 5 && h > 5)
                {

                    float aspect_ratio = (float)w / h;
                    int bounding_box_area = w * h;
                    float fill_ratio = (float)yb.area / bounding_box_area;

                    if (aspect_ratio > 0.1f && aspect_ratio < 10.0f)
                    {
                        if (fill_ratio > 0.15f && fill_ratio < 0.6f)
                        {
                            if (yb.area > car.area)
                            {
                                car = yb;
                            }
                        }
                    }
                }
            }
        }
    }
    return car;
}

void calculate_pca(Blob *blob)
{
    if (blob->sum_weight <= 0)
    {
        blob->mu20 = 0;
        blob->mu02 = 0;
        blob->mu11 = 0;
        blob->dir_x = 0;
        blob->dir_y = -1;
        return;
    }

    float inv255 = 1.0f / 255.0f;
    float cx = blob->cx;
    float cy = blob->cy;

    blob->mu20 = (blob->sum_x2 - (float)blob->sum_x * cx) * inv255;
    blob->mu02 = (blob->sum_y2 - (float)blob->sum_y * cy) * inv255;
    blob->mu11 = (blob->sum_xy - (float)blob->sum_x * cy) * inv255;

    float T1 = blob->mu20 + blob->mu02;
    float T2 = sqrt((blob->mu20 - blob->mu02) * (blob->mu20 - blob->mu02) + 4 * blob->mu11 * blob->mu11);
    float lambda2 = (T1 - T2) / 2.0f; // lambda2为较小的特征值

    // 计算得到的vx,vy是短轴方向
    float vx = -blob->mu11;
    float vy = blob->mu20 - lambda2;
    float len = sqrt(vx * vx + vy * vy);
    if (len > 0.001f)
    {
        vx /= len;
        vy /= len;
    }
    else
    {
        vx = 0;
        vy = -1;
    }

    int count_pos = 0;
    int count_neg = 0;
    float sum_long_var_pos = 0.0f;
    float sum_long_var_neg = 0.0f;

    for (int y = blob->min_y; y <= blob->max_y; y++)
    {
        for (int x = blob->min_x; x <= blob->max_x; x++)
        {
            if (image_arr[y][x] >= THRESH_MID)
            {
                float dx = x - cx;
                float dy = y - cy;

                float proj_short = dx * vx + dy * vy;
                // 长轴和短轴垂直
                float proj_long = dx * (-vy) + dy * vx;
                float long_sq = proj_long * proj_long;

                if (proj_short > 0)
                {
                    count_pos++;
                    sum_long_var_pos += long_sq;
                }
                else if (proj_short < 0)
                {
                    count_neg++;
                    sum_long_var_neg += long_sq;
                }
            }
        }
    }

    float var_pos = (count_pos > 0) ? (sum_long_var_pos / count_pos) : 0.0f;
    float var_neg = (count_neg > 0) ? (sum_long_var_neg / count_neg) : 0.0f;

    // 方差较大的一侧是开口，方差较小的一侧是尖端。
    if (var_pos > var_neg)
    {
        // 如果 proj_short > 0的长轴方差较大，说明原正方向指向了开口
        // 需要反转指向尖端
        vx = -vx;
        vy = -vy;
    }

    blob->dir_x = vx;
    blob->dir_y = vy;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_info_init();             // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    mt9v03x_init();

    uart_init(UART_4, 115200, UART4_TX_P14_1, UART4_RX_P14_0);
    //uart_rx_interrupt(UART_4, 1);

#if WIFI_OPEN
    wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST);
    if (1 != WIFI_SPI_AUTO_CONNECT) // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while (wifi_spi_socket_connect( // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                      // 指定使用TCP方式通讯
            TCP_TARGET_IP,              // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,            // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))           // 指定本机的端口号
        {
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);
        }
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);

#endif

    BeaconTracker tracker1 = {false, 0, 0, 0, 0, 30, 0, 0, 0, 0};
    BeaconTracker tracker2 = {false, 0, 0, 0, 0, 30, 0, 0, 0, 0};

    char car_dat[30] = {0};
    float filtered_car_dir_x = 0.0f;
    float filtered_car_dir_y = -1.0f;
    float alpha = 0.4f;

    while (true)
    {
        memcpy(image_arr[0], mt9v03x_image[0], max_scan_line);

        memset(visited, 0, sizeof(visited));

        Blob car = detect_car();

        memset(visited, 0, sizeof(visited));

        Blob beacon1 = {0};
        Blob beacon2 = {0};

        if (!tracker1.locked && tracker2.locked)
        {
            beacon2 = detect_beacon(&tracker2, NULL, &car);
            beacon1 = detect_beacon(&tracker1, &beacon2, &car);
        }
        else
        {
            beacon1 = detect_beacon(&tracker1, NULL, &car);
            beacon2 = detect_beacon(&tracker2, &beacon1, &car);
        }

        if (beacon1.area > 0 && beacon2.area > 0)
        {
            if (beacon1.cx > beacon2.cx)
            {
                Blob temp_b = beacon1;
                beacon1 = beacon2;
                beacon2 = temp_b;

                BeaconTracker temp_t = tracker1;
                tracker1 = tracker2;
                tracker2 = temp_t;
            }
        }

        if (car.area > 0)
        {
            calculate_pca(&car);

            filtered_car_dir_x = alpha * car.dir_x + (1.0f - alpha) * filtered_car_dir_x;
            filtered_car_dir_y = alpha * car.dir_y + (1.0f - alpha) * filtered_car_dir_y;

            float len = sqrt(filtered_car_dir_x * filtered_car_dir_x + filtered_car_dir_y * filtered_car_dir_y);
            if (len > 0.001f)
            {
                car.dir_x = filtered_car_dir_x / len;
                car.dir_y = filtered_car_dir_y / len;
            }
            else
            {
                car.dir_x = 0;
                car.dir_y = -1;
            }
        }
        
        if (beacon1.area == 0 && beacon2.area == 0)
        {
            beacon_loast = true;
        }

        float err_fwd = 0.0f, err_lat = 0.0f;
        Blob target_beacon = {0};
        if (beacon1.area > 0 && beacon2.area > 0 && car.area > 0)
        {
            float dist1 = (beacon1.cx - car.cx) * (beacon1.cx - car.cx) + (beacon1.cy - car.cy) * (beacon1.cy - car.cy);
            float dist2 = (beacon2.cx - car.cx) * (beacon2.cx - car.cx) + (beacon2.cy - car.cy) * (beacon2.cy - car.cy);
            target_beacon = (dist1 < dist2) ? beacon1 : beacon2;
        }
        else if (beacon1.area > 0)
        {
            target_beacon = beacon1;
        }
        else if (beacon2.area > 0)
        {
            target_beacon = beacon2;
        }

        if (target_beacon.area > 0 && car.area > 0)
        {
            float vec_x = target_beacon.cx - car.cx;
            float vec_y = target_beacon.cy - car.cy;

            float right_x = car.dir_y;
            float right_y = -car.dir_x;
            err_fwd = vec_x * car.dir_x + vec_y * car.dir_y;
            err_lat = vec_x * right_x + vec_y * right_y;
        }
        sprintf(car_dat, "%0.1f,%0.1f\n", err_fwd, err_lat);
        // printf("%s  ,  %0.2f,   %0.2f\n",car_dat,beacon1.cx,beacon1.cy);
        uart_write_string(UART_4, car_dat);

#if WIFI_OPEN
        // 画图
        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        if (tracker1.locked)
        {
            for (int i = tracker1.search_min_x; i <= tracker1.search_max_x; i++)
            {
                image_copy[tracker1.search_min_y][i] = 255;
                image_copy[tracker1.search_max_y][i] = 255;
            }
            for (int i = tracker1.search_min_y; i <= tracker1.search_max_y; i++)
            {
                image_copy[i][tracker1.search_min_x] = 255;
                image_copy[i][tracker1.search_max_x] = 255;
            }
        }
        if (tracker2.locked)
        {
            for (int i = tracker2.search_min_x; i <= tracker2.search_max_x; i++)
            {
                image_copy[tracker2.search_min_y][i] = 255;
                image_copy[tracker2.search_max_y][i] = 255;
            }
            for (int i = tracker2.search_min_y; i <= tracker2.search_max_y; i++)
            {
                image_copy[i][tracker2.search_min_x] = 255;
                image_copy[i][tracker2.search_max_x] = 255;
            }
        }
        if(beacon1.area > 0)
        {
            for (int x = beacon1.min_x; x <= beacon1.max_x; x++)
            {
                image_copy[beacon1.min_y][x] = 255;
                image_copy[beacon1.max_y][x] = 255;
            }
            for (int y = beacon1.min_y; y <= beacon1.max_y; y++)
            {
                image_copy[y][beacon1.min_x] = 255;
                image_copy[y][beacon1.max_x] = 255;
            }
        }
        if(beacon2.area > 0)
        {
            for (int x = beacon2.min_x; x <= beacon2.max_x; x++)
            {
                image_copy[beacon2.min_y][x] = 255;
                image_copy[beacon2.max_y][x] = 255;
            }
            for (int y = beacon2.min_y; y <= beacon2.max_y; y++)
            {
                image_copy[y][beacon2.min_x] = 255;
                image_copy[y][beacon2.max_x] = 255;
            }
        }
        if (car.area > 0)
        {
            for (int x = car.min_x; x <= car.max_x; x++)
            {
                image_copy[car.min_y][x] = 255;
                image_copy[car.max_y][x] = 255;
            }
            for (int y = car.min_y; y <= car.max_y; y++)
            {
                image_copy[y][car.min_x] = 255;
                image_copy[y][car.max_x] = 255;
            }
        }

        // 大于阈值改为255，小于阈值改为0
        for (int i = 0; i < MT9V03X_H; i++)
        {
            for (int j = 0; j < MT9V03X_W; j++)
            {
                if (image_copy[i][j] > THRESH_MID)
                {
                    image_copy[i][j] = 255;
                }
                else
                {
                    image_copy[i][j] = 0;
                }
            }
        }

            seekfree_assistant_camera_send();

#endif
    }
}