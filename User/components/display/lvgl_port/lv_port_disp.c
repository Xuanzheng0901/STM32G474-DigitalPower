#include "lv_port_disp.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "display.h"
#include "../stm_ssd1306/ssd1306.h"

static void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

static SemaphoreHandle_t xGuiSemaphore = NULL; // LVGL互斥信号量
volatile bool disp_flush_enabled = true;
static uint8_t disp_buf1[DISP_BUF_SIZE];
static uint8_t disp_buf2[DISP_BUF_SIZE];


void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/**
 * @brief 将缓冲区内容刷新到显示屏
 * @param disp 显示对象指针
 * @param area 刷新区域
 * @param px_map 像素数据指针（I1 格式）
 * @note 由于使用双全缓冲模式，area 始终是整个屏幕区域
 */
static void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    (void)area;
    if(!disp_flush_enabled)
    {
        lv_display_flush_ready(disp);
        return;
    }

    uint8_t *pos = px_map + 8; // LVGL绘图缓冲区首8字节为信息
    static uint8_t convert_buffer[DISP_BUF_SIZE - 8];

    uint16_t stride = DISP_HOR_RES / 8; // LVGL 每行的字节数
    uint16_t pages = DISP_VER_RES / 8;  // OLED 的页数

    for(uint16_t page = 0; page < pages; page++)
    {
        for(uint16_t col = 0; col < stride; col++)
        {
            // 1. 定位到 LVGL 缓冲区中这个 8x8 像素块的起始地址
            uint8_t *src = &pos[(page * 8) * stride + col];
            uint8_t b[8];
            // 连续读取 8 行的数据（即这个 8x8 方块的 8 个字节）
            for(uint8_t i = 0; i < 8; i++)
            {
                b[i] = src[i * stride];
            }

            // 2. 定位到目标转换缓冲区的写入地址
            // page 决定是大行，col * 8 决定在这页的第几个水平像素
            uint8_t *dst = &convert_buffer[page * DISP_HOR_RES + col * 8];

            // 3. 开始执行 8x8 矩阵转置
            for(uint8_t i = 0; i < 8; i++)
            {
                // 提取源字节的对应列 (从左到右: MSB -> LSB)
                uint8_t mask = 0x80 >> i;
                uint8_t out_byte = 0;

                // 拼装成目标字节 (LSB在最上: b0 -> b7)
                for(uint8_t j = 0; j < 8; j++)
                {
                    if(b[j] & mask)
                        out_byte |= 0x01 << j;
                }

                // 4. 直接赋值（彻底免去先memset后|=的操作）
                dst[i] = out_byte;
            }
        }
    }
    // 绘制到屏幕 (DMA 发送)
    display_draw(convert_buffer);

    /* 通知 LVGL 刷新完成 */
    lv_display_flush_ready(disp);
}

/*
 *  lvgl渲染出来的buffer:
 *  <--   0th   byte    --> <--   1st    byte   -->
 *   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
 *
 *  我们需要的缓冲区:
 *  <--   0th   byte    -->
 *   0  8 16 24 32 40 48 56 ...
 *  <--   1st   byte    -->
 *   1  9 17
 *   2 10
 *   3 11
 *   4 12
 *   5 13
 *   6 14
 *   7 15
 *
 *  所以需要进行转换
 */


bool lvgl_port_lock(uint32_t timeout_ms)
{
    if(xGuiSemaphore == NULL)
    {
        return false;
    }

    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(xGuiSemaphore, ticks) == pdTRUE;
}

void lvgl_port_unlock(void)
{
    if(xGuiSemaphore != NULL)
    {
        xSemaphoreGive(xGuiSemaphore);
    }
}

void lvgl_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t uMSToWait = 0;
    while(1)
    {
        if(lvgl_port_lock(portMAX_DELAY))
        {
            uMSToWait = lv_timer_handler();
            lvgl_port_unlock();
            vTaskDelay(uMSToWait);
        }
    }
}

void lv_port_disp_init(void)
{
    display_init();

    xGuiSemaphore = xSemaphoreCreateMutex();
    lv_init();
    lv_display_t *disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_resolution(disp, DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(disp, disp_buf1, disp_buf2, DISP_BUF_SIZE, LV_DISPLAY_RENDER_MODE_DIRECT); // 双全缓冲模式
    lv_display_set_flush_cb(disp, disp_flush);

    if(xTaskCreate(lvgl_task, "LVGL", 1024, NULL,
                   6, NULL) == pdPASS)
        LOGI("LVGL", "Display初始化完成");
    else
        LOGE("LVGL", "Display任务创建失败");
}
