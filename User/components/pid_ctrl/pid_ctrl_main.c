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

// float adc_batch_lowpass_filter(uint32_t *adc_raw_buf, uint8_t flag)
// {
//     float sum_filtered = 0.0f;  // 存储滤波后数据总和（避免溢出）
//     static int32_t adc_filtered_buf[ADC_BUFFER_LENGTH / 2];
//     // 1. 初始化滤波第一个值（原始值）
//     adc_filtered_buf[0] = flag ? adc_raw_buf[0] & 0x0000FFFF : adc_raw_buf[0] >> 16;
//     // 2. 对20个数据逐次做一阶RC低通滤波
//     for(uint8_t i = 1; i < ADC_BUFFER_LENGTH / 2; i++)
//     {
//         // 核心滤波公式（整数运算：等效 filtered = α*last + (1-α)*raw）
//         // 放大100倍避免小数，最后除以100，保证精度
//         adc_filtered_buf[i] = (uint32_t)(
//             FILTER_ALPHA * (float)adc_filtered_buf[i - 1] + (1.0f - FILTER_ALPHA) * flag
//                 ? (float)(adc_raw_buf[0] & 0x0000FFFF)
//                 : (float)(adc_raw_buf[i] >> 16));
//
//         // 边界保护：防止ADC值异常
//         if(adc_filtered_buf[i] > 4095)
//             adc_filtered_buf[i] = 4095;
//         if(adc_filtered_buf[i] < 0)
//             adc_filtered_buf[i] = 0;
//     }
//
//     // 3. 计算滤波后20个数据的均值（最终输出，进一步平滑）
//     for(uint8_t i = 0; i < ADC_BUFFER_LENGTH / 2; i++)
//     {
//         sum_filtered += (float)adc_filtered_buf[i];
//     }
//     return (sum_filtered / (float)(ADC_BUFFER_LENGTH / 2));
// }

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

void pwm_set_duty(uint32_t pwm_duty)
{
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, (pwm_duty >> 1) + 1);
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


static void PID_ctrl_routine(void *pvParameters)
{
    static uint32_t target_voltage_mV = 0;
    static uint32_t target_voltage_buffer_mV = 0;

    static float next_output_voltage_mV = 0.0f;

    static uint32_t *buf_ptr;

    static float temp_current_voltage_mV = 0.0f;
    static float last_voltage_mV = 0.0f;
    static float last_last_voltage_mV = 0.0f;
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

            //3. 计算实际电压/电流
            // float origin_voltage_sum = adc_batch_lowpass_filter(buf_ptr, 1);
            // float origin_current_sum = adc_batch_lowpass_filter(buf_ptr, 0);
            float origin_voltage_sum = 0.0f, origin_current_sum = 0.0f;
            for(uint8_t i = 0; i < ADC_BUFFER_LENGTH / 2; i++)
            {
                origin_voltage_sum += buf_ptr[i] & 0x0FFF;
                origin_current_sum += buf_ptr[i] >> 16;
            }
            // 读取到的电压  20分压
            temp_current_voltage_mV = origin_voltage_sum / 4095.0f * 3300.0f * 20.0f / (float)(ADC_BUFFER_LENGTH / 2)
                                      * 1.0048f;
            if(temp_current_voltage_mV >= 10.0f)
                temp_current_voltage_mV += 60.0f;
            if(temp_current_voltage_mV < 0)
                temp_current_voltage_mV = 0;

            now_current_A = origin_current_sum / 4095.0f * 3300.0f * 2.0f / 1000.0f / (
                                ADC_BUFFER_LENGTH / 2); //单位A

            //4. 对均值进行二阶rc滤波
            temp_current_voltage_mV = RC_ALPHA_DENO_1 * temp_current_voltage_mV + RC_ALPHA_DENO_2 * last_voltage_mV +
                                      RC_ALPHA_DENO_3 * last_last_voltage_mV;
            now_voltage_mV = temp_current_voltage_mV;
            last_last_voltage_mV = last_voltage_mV;
            last_voltage_mV = temp_current_voltage_mV;


            //4. 进行pid计算
            float error_mV = (float)target_voltage_mV - temp_current_voltage_mV;

            pid_compute(pid_handle, error_mV, &next_output_voltage_mV);
            uint32_t output_duty = voltage_to_duty(next_output_voltage_mV);
            pwm_set_duty(output_duty);
        }
        // vTaskDelay(1);
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
