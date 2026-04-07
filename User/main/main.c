#include "adc.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "stdio_ext.h"
#include "string.h"
#include "hrtim.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "spi.h"
#include "mppt.h"

extern TaskHandle_t adc_task_handle;

void LED_task0(void *arg)
{
    LOGI("LED", "Task Running. Stack: %p", xTaskGetCurrentTaskHandle());
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
    }
}

void app_main(void)
{
    log_init(LOG_INFO);
    LOGI("MAIN", "Hello world!");


    // printf("helloworld\n");
    xTaskCreate(LED_task0, "LED", 256, NULL, 10, NULL);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    // hhrtim1.Instance->sTimerxRegs[0].CMP1CxR = 0;
    // hhrtim1.Instance->sTimerxRegs[4].CMP1CxR = 0;
    ui_init();
    MPPT_init();
    ADC_init();
}
