#include "lv_port_disp.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "../stm_ssd1306/ssd1306.h"

#define DISP_HOR_RES    128
#define DISP_VER_RES    64
#define DISP_BUF_SIZE   (DISP_HOR_RES * DISP_VER_RES / 8 + 8)

static void disp_init(void);

static void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

static SemaphoreHandle_t xGuiSemaphore = NULL; // LVGL互斥信号量
volatile bool disp_flush_enabled = true;
static uint8_t disp_buf1[DISP_BUF_SIZE];
static uint8_t disp_buf2[DISP_BUF_SIZE];

static void disp_init(void)
{
    ssd1306_init();
}

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

    uint8_t *pos = px_map + 8;
    static uint8_t ssd1306_buffer[128 * 8] = {0};
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));

    /* 转换格式：水平扫描 → 垂直页模式 */
    for(uint16_t y = 0; y < DISP_VER_RES; y++)
    {
        for(uint16_t x = 0; x < DISP_HOR_RES; x++)
        {
            /* 从 LVGL 缓冲区读取像素 (MSB 优先) */
            uint32_t src_byte_idx = (y * DISP_HOR_RES + x) >> 3;
            uint8_t src_bit_offset = x % 8;
            uint8_t pixel = (pos[src_byte_idx] >> (7 - src_bit_offset)) & 0x01;

            /* 写入 SSD1306 缓冲区（页模式）*/
            if(pixel)
            {
                // 这部分逻辑是完美正确的
                uint16_t page = y >> 3;
                uint8_t bit = y % 8;
                uint32_t dst_idx = page * DISP_HOR_RES + x;
                ssd1306_buffer[dst_idx] |= (1 << bit);
            }
        }
    }

    ssd1306_draw(ssd1306_buffer);

    /* 通知 LVGL 刷新完成 */
    lv_display_flush_ready(disp);
}

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
    // printf("Hello LVGL\n");
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
    disp_init();

    xGuiSemaphore = xSemaphoreCreateMutex();
    lv_init();
    lv_display_t *disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_resolution(disp, DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(disp, disp_buf1, disp_buf2, DISP_BUF_SIZE, LV_DISPLAY_RENDER_MODE_DIRECT); // 双全缓冲模式
    lv_display_set_flush_cb(disp, disp_flush);
    xTaskCreate(lvgl_task, "LVGL", 1024, NULL, 6, NULL);
}
