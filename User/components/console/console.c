#include "console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "LOG.h"
#include <stdio.h>
#include <string.h>

extern void set_hrtim_prop(uint32_t freq, int16_t phase_shift_degree);

#define CONSOLE_RX_BUF_SIZE 64
static uint8_t console_rx_buf[CONSOLE_RX_BUF_SIZE] = {0};
static TaskHandle_t consoleTaskHandle;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(consoleTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void console_task(void *pvParameters)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, console_rx_buf, CONSOLE_RX_BUF_SIZE);

    while(1)
    {
        uint32_t ulNotificationValue;
        if(xTaskNotifyWait(0, portMAX_DELAY, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
        {
            uint32_t freq = 0;
            int16_t phase = 0;
            LOGI("CONSOLE", "%s", console_rx_buf);

            // 解析字符串格式： "频率 相位" ，例如 "100000 45"
            // 注意：console_rx_buf 可能有 \r \n，sscanf 会自动忽略空白符
            if(sscanf((char *)console_rx_buf, "%lu %hd", &freq, &phase) == 2)
            {
                if(freq > 0)
                {
                    set_hrtim_prop(freq, phase);
                }
            }
            else
            {
                LOGI("CONSOLE", "Parse error. Input: %s", console_rx_buf);
            }

            memset(console_rx_buf, 0, CONSOLE_RX_BUF_SIZE);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, console_rx_buf, CONSOLE_RX_BUF_SIZE);
        }
    }
}

void console_init(void)
{
    xTaskCreate(console_task, "console_task", 1024, NULL, 5, &consoleTaskHandle);
}
