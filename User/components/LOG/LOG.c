#include "LOG.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"

#define ANSI_RESET  "\033[0m"
#define ANSI_COLOR  "\033[0;%sm"

#define LOG_MEM_POOL_SIZE   16

static log_level_t s_log_level = LOG_VERBOSE;

static xQueueHandle log_queue = NULL;
static xQueueHandle log_mem_pool_queue = NULL;
static xSemaphoreHandle log_uart_sem = NULL;
static TaskHandle_t log_task_handle = NULL;

static log_data_t log_mem_pool[LOG_MEM_POOL_SIZE];

static uint32_t get_time_ms(void)
{
    return xTaskGetTickCount();
}

static void log_get_level_info(log_level_t level, char *level_char, const char **color)
{
    switch(level)
    {
        case LOG_ERROR:
            *level_char = 'E';
            *color = "31";
            break;
        case LOG_WARN:
            *level_char = 'W';
            *color = "33";
            break;
        case LOG_INFO:
            *level_char = 'I';
            *color = "32";
            break;
        case LOG_DEBUG:
            *level_char = 'D';
            *color = "39";
            break;
        default:
            *level_char = 'V';
            *color = "90";
            break;
    }
}

static int log_format_message(char *raw, const char *color, char level_char, uint32_t ts, const char *tag,
                              const char *format, va_list args)
{
    int offset = 0;
    static const char reset_str[] = "\033[0m\n";
    const int reserve_len = sizeof(reset_str) - 1; // 编译期计算，替代代价较高的 strlen
    int max_len_for_text = LOG_RAW_MAX_LEN - reserve_len - 1;

    int n = snprintf(raw + offset, max_len_for_text - offset,
                     "\033[0;%sm%c (%lu) %s: ",
                     color, level_char, ts, tag);
    if(n < 0)
        n = 0;
    if(n >= max_len_for_text - offset)
        n = max_len_for_text - offset;
    offset += n;

    if(offset < max_len_for_text)
    {
        n = vsnprintf(raw + offset, max_len_for_text - offset, format, args);
        if(n < 0)
            n = 0;
        if(n >= max_len_for_text - offset)
            n = max_len_for_text - offset;
        offset += n;
    }

    memcpy(raw + offset, reset_str, reserve_len + 1); // 替代 strcpy，使用已知长度的高效拷贝
    offset += reserve_len;

    return offset;
}

void log_set_level(log_level_t level)
{
    s_log_level = level;
}

void log_write(log_level_t level, const char *tag, const char *format, ...)
{
    if(level > s_log_level || log_queue == NULL)
        return;

    const char *color;
    char level_char;
    log_get_level_info(level, &level_char, &color);

    log_data_t *log_buffer = NULL;

    if(xQueueReceive(log_mem_pool_queue, &log_buffer, 0) != pdTRUE)
        return;

    uint32_t ts = get_time_ms();
    va_list args;
    va_start(args, format);
    log_buffer->len = log_format_message(log_buffer->data, color, level_char, ts, tag, format, args);
    va_end(args);

    if(xQueueSend(log_queue, &log_buffer, 0) != pdTRUE)
    {
        xQueueSend(log_mem_pool_queue, &log_buffer, 0);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(huart == &huart3)
    {
        vTaskNotifyGiveFromISR(log_task_handle, &xHigherPriorityTaskWoken);
    }
}

void log_send_task(void *args)
{
    log_data_t *recv_buf_ptr = NULL;
    while(1)
    {
        if(xQueueReceive(log_queue, &recv_buf_ptr, portMAX_DELAY) == pdTRUE)
        {
            if(HAL_UART_Transmit_DMA(&huart3, (const uint8_t *)recv_buf_ptr->data, recv_buf_ptr->len) != HAL_OK)
            {
                // 如果失败，手动释放内存并继续循环，否则会死锁
                xQueueSend(log_mem_pool_queue, &recv_buf_ptr, 0);
                continue;
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //等待DMA传输完成
            xQueueSend(log_mem_pool_queue, &recv_buf_ptr, 0);
        }
    }
}

void log_init(log_level_t level)
{
    log_set_level(level);
    log_mem_pool_queue = xQueueCreate(LOG_MEM_POOL_SIZE, sizeof(log_data_t*));
    for(uint8_t i = 0; i < LOG_MEM_POOL_SIZE; i++)
    {
        log_data_t *ptr = &log_mem_pool[i];
        xQueueSend(log_mem_pool_queue, &ptr, portMAX_DELAY); //将内存池指针放入队列
    }

    log_queue = xQueueCreate(LOG_MEM_POOL_SIZE, sizeof(log_data_t*));
    log_uart_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(log_uart_sem);

    xTaskCreate(log_send_task, "log_send_task", 256, NULL, 12, &log_task_handle);

    LOGI("LOG", "日志服务已启动");
}
