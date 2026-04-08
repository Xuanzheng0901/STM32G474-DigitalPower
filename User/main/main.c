#include "adc.h"
#include "FreeRTOS.h"
#include "main.h"

#include <math.h>

#include "task.h"
#include "stdio_ext.h"
#include "string.h"
#include "hrtim.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "spi.h"

extern TaskHandle_t adc_task_handle;

#define TABLE_SIZE 400
#define PERIOD_VAL 34000

uint32_t SineTable[TABLE_SIZE];
volatile uint32_t sin_index = 0;

void LED_task0(void *arg)
{
    LOGI("LED", "Task Running. Stack: %p", xTaskGetCurrentTaskHandle());
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    //  2. 严谨判断：只有当 Master 定时器的 Registers Update 触发时才执行
    if(TimerIdx == HRTIM_TIMERINDEX_MASTER)
    {
        uint32_t duty_A = SineTable[sin_index] >> 1;
        uint32_t duty_B = (PERIOD_VAL - SineTable[sin_index]) >> 1; // 34000 - duty_A

        // 翻转 PC0 供示波器观察，如果 PC0 变成 20kHz 脉冲，说明中断完全正常
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

        //  3. 写入 A 和 B 的占空比（绝不去改 Master 的 CMP1 寄存器）
        __HAL_HRTIM_SETCOMPARE(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, 17000 - duty_A);
        __HAL_HRTIM_SETCOMPARE(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, 17000 + duty_A);

        __HAL_HRTIM_SETCOMPARE(hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, 17000 - duty_B);
        __HAL_HRTIM_SETCOMPARE(hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, 17000 + duty_B);

        // sin_index++;
        if(++sin_index >= TABLE_SIZE)
        {
            sin_index = 0;
        }
    }
}

void app_main(void)
{
    log_init(LOG_INFO);
    LOGI("MAIN", "Hello world!");

    float M = 0.8f;
    for(int i = 0; i < TABLE_SIZE; i++)
    {
        float theta = (2.0f * (float)M_PI * (float)i) / (float)TABLE_SIZE;
        SineTable[i] = (uint32_t)(17000.0f * (1.0f + M * sinf(theta)) + 0.5f);
    }

    // printf("helloworld\n");
    xTaskCreate(LED_task0, "LED", 256, NULL, 10, NULL);
    ui_init();
    pid_ctrl_init();
    ADC_init();

    HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_TIMER_B);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_SimpleBaseStart_IT(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
}
