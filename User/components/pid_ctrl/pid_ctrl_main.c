#include "pid_ctrl_internal.h"
#include "FreeRTOS.h"
#include "hrtim.h"
#include "main.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t adc_queue;

static pid_ctrl_block_handle_t pid_handle = NULL;
QueueHandle_t pid_ctrl_queue_mV = NULL; //单位为mV
static float now_current_A = 0.0f, now_voltage_mV = 0.0f;

// --- 定义常量 ---
#define PWM_PERIOD_ARR    PWM_Period
#define INPUT_VOLTAGE_MV  30000.0f // 输入电压 (暂时假设30V)
#define MAX_DUTY_RATIO    0.80f    // 最大占空比限制 (理论4倍升压)
#define RC_ALPHA_DENO_1   0.85f
#define RC_ALPHA_DENO_2   0.10f
#define RC_ALPHA_DENO_3   0.05f

static uint32_t voltage_to_duty(float expect_mV)
{
    if(expect_mV < 10.0f)
        return 0;

    float denominator = expect_mV + INPUT_VOLTAGE_MV;

    if(denominator < 100.0f)
        denominator = 100.0f;

    float duty_cycle = expect_mV / denominator;

    // 硬件保护
    if(duty_cycle > MAX_DUTY_RATIO)
        duty_cycle = MAX_DUTY_RATIO;

    return (uint32_t)(duty_cycle * PWM_PERIOD_ARR);
}

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

float get_voltage_value(uint8_t index)
{
    if(index == 0)
        return now_voltage_mV;
    if(index == 1)
        return now_current_A;

    return 0.0f;
}

static float last_last_voltage_mV = 0.0f;
static float last_last_current_A = 0.0f;

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

    float temp_current_A = origin_current_sum * 3300.0f / 4095.0f / (ADC_BUFFER_LENGTH / 2) * 2.0f / 1050.0f; //单位mA

    //对均值进行二阶rc滤波
    temp_voltage_mV = RC_ALPHA_DENO_1 * temp_voltage_mV + RC_ALPHA_DENO_2 * now_voltage_mV +
                      RC_ALPHA_DENO_3 * last_last_voltage_mV;

    temp_current_A = RC_ALPHA_DENO_1 * temp_current_A + RC_ALPHA_DENO_2 * now_current_A +
                     RC_ALPHA_DENO_3 * last_last_current_A;

    last_last_voltage_mV = now_voltage_mV;
    last_last_current_A = now_current_A;

    now_voltage_mV = temp_voltage_mV;
    now_current_A = temp_current_A;
}


static void PID_ctrl_routine(void *pvParameters)
{
    static uint32_t target_voltage_mV = 0;
    static uint32_t target_voltage_buffer_mV = 0;

    static float next_output_voltage_mV = 0.0f;

    static uint32_t *buf_ptr;
    while(1)
    {
        //1. 等待ADC数据
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) == pdTRUE)
        {
            //2. 查询target是否改变
            if(pdPASS == xQueueReceive(pid_ctrl_queue_mV, &target_voltage_buffer_mV, 0))
            {
                if(target_voltage_mV != target_voltage_buffer_mV)
                {
                    target_voltage_mV = target_voltage_buffer_mV;
                    // pid_reset_ctrl_block(pid_handle); //使用增量式pid更改target后不能重置
                }
            }
            adc_data_process(buf_ptr);

            //4. 进行pid计算
            float error_mV = (float)target_voltage_mV - now_voltage_mV;

            pid_compute(pid_handle, error_mV, &next_output_voltage_mV);
            uint32_t output_duty = voltage_to_duty(next_output_voltage_mV);
            pwm_set_duty(output_duty);
        }
    }
}

void pid_set_voltage(uint32_t mv)
{
    if(NULL == pid_ctrl_queue_mV)
        return;
    xQueueSend(pid_ctrl_queue_mV, &mv, portMAX_DELAY);
}

void pid_ctrl_init(void)
{
    pid_ctrl_config_t pid_cfg = {
        .init_param = {
            .kp           = 0.1f,
            .ki           = 0.06f,
            .kd           = 0.06f,
            .max_output   = 90000.0f,
            .min_output   = 0.0f,
            .max_integral = 1000000.0f,
            .min_integral = -1000000.0f,
            .cal_type     = PID_CAL_TYPE_INCREMENTAL,
        }
    };
    pid_new_control_block(&pid_cfg, &pid_handle);
    pid_ctrl_queue_mV = xQueueCreate(6, sizeof(uint32_t));
    xTaskCreate(PID_ctrl_routine, "PID", 2048, NULL, 15, NULL);
    pid_set_voltage(0);
}
