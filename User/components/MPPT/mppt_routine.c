#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "mppt_internal.h"
#include "mppt.h"
#include "hrtim.h"

extern QueueHandle_t adc_queue;
// 实时量: 电流单位A, 电压单位mV(与UI显示接口保持一致)
static float now_current_A = 0.0f, now_voltage_mV = 0.0f;

// 电压二阶IIR平滑系数: 新值+一阶历史+二阶历史
#define RC_ALPHA_DENO_1        0.85f
#define RC_ALPHA_DENO_2        0.10f
#define RC_ALPHA_DENO_3        0.05f

// PWM占空比控制边界
#define MPPT_DUTY_MIN          500U
#define MPPT_DUTY_MAX          54000U
#define MPPT_DUTY_INIT         6000U

// 固定扰动步长(单位: 占空比计数)
#define MPPT_DUTY_STEP         50U

/*
 * MPPT_POWER_EPS_W: 功率变化死区阈值(单位W)。
 * |dP| 小于该值时认为主要来自ADC噪声或纹波，不立即触发方向反转。
 */
#define MPPT_POWER_EPS_W       0.05f

// 控制环分频: 每收到MPPT_CTRL_UPDATE_DIV次ADC块数据更新一次占空比
#define MPPT_CTRL_UPDATE_DIV   1U
#define MPPT_TASK_PRIORITY     12U

void pwm_set_duty(uint32_t pwm_duty) //占空比0-54400
{
    if(pwm_duty <= 27200)
    {
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, 27200 - (pwm_duty >> 1) + 1);
    }

    else
    {
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, (pwm_duty >> 1) + 1);
    }

    hhrtim1.Instance->sTimerxRegs[TIMER_E].CMP1CxR = pwm_duty;
}

// UI读取接口: index=0返回mV, index=1返回A
float get_voltage_value(uint8_t index)
{
    if(index == 0)
        return now_voltage_mV;
    if(index == 1)
        return now_current_A;

    return 0.0f;
}

static float last_last_voltage_mV = 0.0f;
static float last_last_current_mA = 0.0f;

// 把DMA块数据转换为电压/电流工程量，并做电压去噪
static inline void adc_data_process(uint32_t *data_buf)
{
    float origin_voltage_sum = 0.0f, origin_current_sum = 0.0f;
    for(uint8_t i = 0; i < ADC_BUFFER_LENGTH / 2; i++)
    {
        origin_voltage_sum += data_buf[i] & 0x0FFF;
        origin_current_sum += data_buf[i] >> 16;
    }
    // origin_voltage_sum / 4095 * 3300 / 20(样本量) = adc引脚上的电压
    // * 20 = 运放输入电压(运放20分压)
    // * 1.0079(adc误差修正)
    float temp_voltage_mV = origin_voltage_sum * 3300.0f / 4095.0f / (float)(ADC_BUFFER_LENGTH / 2) * 20.0f *
                            1.0079f;

    float temp_current_mA = origin_current_sum * 3300.0f / 4095.0f / (ADC_BUFFER_LENGTH / 2) * 2.0f / 1.05f; //单位A

    //对均值进行二阶rc滤波
    temp_voltage_mV = RC_ALPHA_DENO_1 * temp_voltage_mV + RC_ALPHA_DENO_2 * now_voltage_mV +
                      RC_ALPHA_DENO_3 * last_last_voltage_mV;

    temp_current_mA = RC_ALPHA_DENO_1 * temp_current_mA + RC_ALPHA_DENO_2 * now_current_A +
                      RC_ALPHA_DENO_3 * last_last_current_mA;

    last_last_voltage_mV = now_voltage_mV;
    last_last_current_mA = now_current_A;

    now_voltage_mV = temp_voltage_mV;
    now_current_A = temp_current_mA;
}

// 占空比边界钳位，防止越界写入HRTIM
static inline uint32_t clamp_duty(int32_t duty)
{
    if(duty < (int32_t)MPPT_DUTY_MIN)
        return MPPT_DUTY_MIN;
    if(duty > (int32_t)MPPT_DUTY_MAX)
        return MPPT_DUTY_MAX;
    return (uint32_t)duty;
}

// MPPT主循环(P&O): 基于功率增量决定扰动方向，固定步长扰动
void mppt_routine(void *args)
{
    static uint32_t *buf_ptr;
    uint32_t duty = MPPT_DUTY_INIT;
    int8_t direction = 1;
    uint8_t update_div_cnt = 0;

    float prev_power_w = 0.0f;
    uint8_t initialized = 0;

    pwm_set_duty(duty);

    (void)args;
    printf("MPPT started\n");
    while(1)
    {
        //1. 等待ADC数据
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) == pdTRUE)
        {
            adc_data_process(buf_ptr);

            if(++update_div_cnt < MPPT_CTRL_UPDATE_DIV)
                continue;
            update_div_cnt = 0;

            // P(W) = V(mV) * I(A) / 1000
            float now_power_w = (now_voltage_mV * now_current_A) / 1000000.0f;

            if(!initialized)
            {
                prev_power_w = now_power_w;
                initialized = 1;
                continue;
            }

            // 一阶差分: 当前功率变化量
            float d_power_w = now_power_w - prev_power_w;

            // 硬反转: 功率明确下降(超过死区)
            if(d_power_w < -MPPT_POWER_EPS_W)
            {
                direction = -direction;
            }

            duty = clamp_duty((int32_t)duty + direction * (int32_t)MPPT_DUTY_STEP);

            if((duty == MPPT_DUTY_MIN && direction < 0) || (duty == MPPT_DUTY_MAX && direction > 0))
                direction = -direction;

            pwm_set_duty(duty);
            prev_power_w = now_power_w;
        }
    }
}

// 创建MPPT任务，初始化后由调度器接管
void MPPT_init(void)
{
    xTaskCreate(mppt_routine, "MPPT", 2048, NULL, MPPT_TASK_PRIORITY, NULL);
}
