#include "pid_ctrl_internal.h"
#include "FreeRTOS.h"
#include <math.h>
#include "hrtim.h"
#include "main.h"
#include "adc.h"
#include "task.h"
#include "queue.h"
#include "kalman.h"

// --- 全局共享变量（连接快环与慢环的桥梁） ---
// 由电压慢环计算得出，供给电流快环使用，代表电感电流的幅值指令
volatile float Global_Current_Ref_Amplitude = 0.0f;

extern QueueHandle_t adc_queue;
QueueHandle_t pid_ctrl_queue_mV = NULL; //单位为mV

static pid_ctrl_block_handle_t vbus_pid_handle = NULL;    // 电压外环 (跑在 FreeRTOS)
static pid_ctrl_block_handle_t current_pid_handle = NULL; // 电流内环 (跑在 HRTIM 极高频中断)

static float now_current_A = 0.0f, now_voltage_mV = 0.0f;

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    if(TimerIdx == HRTIM_TIMERINDEX_MASTER)
    {
        // 1. 获取【瞬时】交流输入电压形状 (馒头波) 和【瞬时】电感电流
        // // float vac_instant = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) * 转换系数;
        uint16_t vac_instant = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
        uint16_t il_instant = HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_1);

        // 2. 提取电网电压形状 (取绝对值，保证是馒头波)
        float vac_shape = (float)vac_instant / 4095.0f; // 归一化到 0~1

        // 3. 核心乘法器：目标瞬时电流 = 电网电压形状 * 电流幅值指令(来自电压外环)
        // target_il_instant 单位为 A
        float target_il_instant = vac_shape * Global_Current_Ref_Amplitude;

        // 需要将当前电感电流的ADC值也转换为 A，才能与 target_il_instant 比较
        // 这里假设转换系数为 1.4652f (参考 adc_data_process 中的系数)
        float il_instant_A = (float)il_instant * 1.4652f / (ADC_BUFFER_LENGTH / 2.0f); // 注意这里的系数要和实际对应

        // 4. 执行电流环 PID 计算
        float duty_cycle = 0.0f;
        float error_i = target_il_instant - il_instant_A;
        pid_compute(current_pid_handle, error_i, &duty_cycle);

        // 5. 占空比限幅保护 (Boost PFC 占空比在 0 ~ 0.95 之间)
        if(duty_cycle > 0.95f)
            duty_cycle = 0.95f;
        if(duty_cycle < 0.0f)
            duty_cycle = 0.0f;

        // 6. 将计算结果写入 HRTIM 寄存器更新 PWM (假设驱动 TIMER_A)
        uint32_t compare_val = (uint32_t)(duty_cycle * PWM_Period);
        __HAL_HRTIM_SETCOMPARE(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, compare_val);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}

float get_voltage_value(uint8_t index)
{
    if(index == 0)
        return now_voltage_mV;
    if(index == 1)
        return now_current_A;

    return 0.0f;
}

static void adc_data_process(uint32_t *data_buf)
{
    static kalman_1d_state_t kf_voltage;
    static kalman_1d_state_t kf_current;
    static uint8_t is_kf_initialized = 0;

    uint32_t u_sum = 0, i_sum = 0;
    uint16_t len = ADC_BUFFER_LENGTH / 2;

    for(uint16_t i = 0; i < len; i++)
    {
        u_sum += data_buf[i] & 0x0FFF;
        i_sum += data_buf[i] >> 16;
    }

    float u_raw_f = (float)u_sum * 14.652f / len;
    float i_raw_f = (float)i_sum * 1.4652f / len;

    if(!is_kf_initialized)
    {
        // 参数调整说明：
        // Q越大，跟踪越快，滤波效果越弱；Q越小，系统越稳定，但存在滞后
        // R越大，滤波效果越强，认为传感器噪声大；R越小，越相信传感器测量值
        kalman_1d_init(&kf_voltage, u_raw_f, 10.0f, 0.5f, 50.0f); // 电压Q=0.5, R=50
        kalman_1d_init(&kf_current, i_raw_f, 1.0f, 0.01f, 1.0f); // 电流Q=0.01, R=1.0
        is_kf_initialized = 1;
    }

    now_voltage_mV = kalman_1d_update(&kf_voltage, u_raw_f);
    now_current_A = kalman_1d_update(&kf_current, i_raw_f);
}


static void PID_ctrl_routine(void *pvParameters)
{
    static uint32_t target_voltage_mV = 0;
    static uint32_t target_voltage_buffer_mV = 0;
    static float current_amplitude_cmd = 0.0f; // 算出来的幅值指令
    static uint32_t *buf_ptr;

    while(1)
    {
        //1. 等待ADC数据
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) == pdTRUE)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);

            // 2. 查询是否修改了目标电压
            if(pdPASS == xQueueReceive(pid_ctrl_queue_mV, &target_voltage_buffer_mV, 0))
            {
                if(target_voltage_mV != target_voltage_buffer_mV)
                {
                    target_voltage_mV = target_voltage_buffer_mV;
                }
            }
            adc_data_process(buf_ptr);

            // 4. 进行电压环 PID 计算
            float error_mV = (float)target_voltage_mV - now_voltage_mV;

            // 电压环计算出的是：需要给电流环多大的幅值
            pid_compute(vbus_pid_handle, error_mV, &current_amplitude_cmd);
            if(current_amplitude_cmd < 0.01f)
                current_amplitude_cmd = 0.0f;

            // 6. 更新全局变量，供底层的 HRTIM 中断使用
            Global_Current_Ref_Amplitude = current_amplitude_cmd;

            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
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
    pid_ctrl_config_t vbus_pid_cfg = {
        .init_param = {
            .kp           = 0.00005f,
            .ki           = 0.000005f,
            .kd           = 0.00006f,
            .max_output   = 5.0f, // 最大允许请求5A峰值电流
            .min_output   = 0.0f,
            .max_integral = 5.0f,
            .min_integral = 0.0f,
            .cal_type     = PID_CAL_TYPE_INCREMENTAL, // 或者位置式 (POSITIONAL)
        }
    };
    pid_new_control_block(&vbus_pid_cfg, &vbus_pid_handle);

    pid_ctrl_config_t current_pid_cfg = {
        .init_param = {
            .kp           = 0.05f,     // 电流环 Kp 需要大很多，快速响应
            .ki           = 0.001f,
            .kd           = 0.0f,
            .max_output   = 0.95f,     // 最大占空比 95%
            .min_output   = 0.0f,      // 最小占空比 0%
            .max_integral = 0.5f,
            .min_integral = -0.5f,
            .cal_type     = PID_CAL_TYPE_INCREMENTAL,
        }
    };
    pid_new_control_block(&current_pid_cfg, &current_pid_handle);

    pid_ctrl_queue_mV = xQueueCreate(6, sizeof(uint32_t));
    xTaskCreate(PID_ctrl_routine, "PID", 2048, NULL, 15, NULL);
    pid_set_voltage(0);
}
