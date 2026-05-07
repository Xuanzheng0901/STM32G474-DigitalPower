#include "pid_ctrl_internal.h"
#include "FreeRTOS.h"
#include <math.h>
#include "hrtim.h"
#include "main.h"
#include "task.h"
#include "queue.h"
#include "kalman.h"
#include "PID.h"

extern QueueHandle_t adc_queue;

static pid_ctrl_block_handle_t pid_handle = NULL;
QueueHandle_t pid_ctrl_queue_mA = NULL; //单位为mV
static float now_high_current_mA = 0.0f, now_high_voltage_mV = 0.0f;
static float now_low_current_mA = 0.0f, now_low_voltage_mV = 0.0f;
static uint16_t mode = MODE_SLEEP;

static float fai_A = 0.0f, fai_B = 0.0f, fai_C = 0.0f;

float get_pid_value(uint8_t index)
{
    if(index == 0)
        return now_high_voltage_mV;
    if(index == 1)
        return now_high_current_mA;
    if(index == 2)
        return now_low_voltage_mV;
    if(index == 3)
        return now_low_current_mA;

    return 0.0f;
}

void set_hrtim_prop(uint32_t freq, int16_t phase_shift_degree)
{
    static uint32_t current_prescaler = HRTIM_PRESCALERRATIO_MUL8;
    const static float factor_tbl[8] = {32.0f, 16.0f, 8.0f, 4.0f, 2.0f, 1.0f, 0.5f, 0.25f};

    uint32_t period = (uint32_t)(170000000.0f * factor_tbl[current_prescaler] / (float)freq);
    while(period >= 0xFF00)
    {
        if(current_prescaler == HRTIM_PRESCALERRATIO_DIV4)
            break;
        current_prescaler++;

        period = (uint32_t)(170000000.0f * factor_tbl[current_prescaler] / (float)freq);
    }

    while(period <= 0x7000)
    {
        if(current_prescaler == HRTIM_PRESCALERRATIO_MUL32)
            break;
        current_prescaler--;

        period = (uint32_t)(170000000.0f * factor_tbl[current_prescaler] / (float)freq);
    }

    int32_t offset = (int32_t)((float)phase_shift_degree / 360.0f * (float)period);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

    HAL_HRTIM_WaveformCountStop(&hhrtim1,HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);

    __HAL_HRTIM_SETCLOCKPRESCALER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, current_prescaler);
    __HAL_HRTIM_SETCLOCKPRESCALER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, current_prescaler);
    __HAL_HRTIM_SETCLOCKPRESCALER(&hhrtim1, HRTIM_TIMERINDEX_MASTER, current_prescaler);

    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, period);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, period);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, period);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, period / 2);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, period / 2);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1,
                           offset >= 0 ? offset : period + offset);

    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_MASTER, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);

    // HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

    // LOGI("HRTIM", "Freq: %lu Hz, Phase: %hd deg", freq, phase_shift_degree);
}

// 把DMA块数据转换为电压/电流工程量，并做电压去噪
static void adc_data_process(uint32_t *data_buf)
{
    static kalman_1d_state_t kf_high_voltage;
    static kalman_1d_state_t kf_high_current;
    static kalman_1d_state_t kf_low_voltage;
    static kalman_1d_state_t kf_low_current;
    static uint8_t is_kf_initialized = 0;
    uint16_t len = ADC_BUFFER_LENGTH / 2;

    uint32_t v_sum[2] = {0}; //0为高侧, 1为低侧
    uint32_t i_sum[2] = {0};

    //低16位对应ADC1,差分采样电流, rank1对应高侧
    for(uint32_t i = 0; i < len; i += 2)
    {
        v_sum[0] += data_buf[i] >> 16 & 0x0FFF;
        i_sum[0] += data_buf[i] & 0x0FFF;
        v_sum[1] += data_buf[i + 1] >> 16 & 0x0FFF;
        i_sum[1] += data_buf[i + 1] & 0x0FFF;
    }

    // 计算平均值 (每个通道采样 len/2 次)
    float v_avg_high = (float)v_sum[0] / ((float)len / 2.0f);
    float v_avg_low = (float)v_sum[1] / ((float)len / 2.0f);
    float i_avg_high = (float)i_sum[0] / ((float)len / 2.0f);
    float i_avg_low = (float)i_sum[1] / ((float)len / 2.0f);

    float raw_high_voltage_mV = v_avg_high * (3000.0f / 4095.0f);
    float raw_high_current_A = i_avg_high * (3000.0f / 4095.0f);
    float raw_low_voltage_mV = v_avg_low * (3000.0f / 4095.0f);
    float raw_low_current_A = i_avg_low * (3000.0f / 4095.0f);

    // 使用第一次测量值作为初始状态可以加快滤波器的收敛速度
    if(!is_kf_initialized)
    {
        // Q越大，跟踪越快，滤波效果越弱；Q越小，系统越稳定，但存在滞后
        // R越大，滤波效果越强，认为传感器噪声大；R越小，越相信传感器测量值
        kalman_1d_init(&kf_high_voltage, raw_high_voltage_mV, 10.0f, 0.5f, 50.0f); // 电压Q=0.5, R=50
        kalman_1d_init(&kf_high_current, raw_high_current_A, 1.0f, 0.01f, 1.0f); // 电流Q=0.01, R=1.0
        kalman_1d_init(&kf_low_voltage, raw_low_voltage_mV, 10.0f, 0.5f, 50.0f); // 电压Q=0.5, R=50
        kalman_1d_init(&kf_low_current, raw_low_current_A, 1.0f, 0.01f, 1.0f); // 电流Q=0.01, R=1.0
        is_kf_initialized = 1;
    }

    now_high_voltage_mV = kalman_1d_update(&kf_high_voltage, raw_high_voltage_mV);
    now_high_current_mA = kalman_1d_update(&kf_high_current, raw_high_current_A);
    now_low_voltage_mV = kalman_1d_update(&kf_low_voltage, raw_low_voltage_mV);
    now_low_current_mA = kalman_1d_update(&kf_low_current, raw_low_current_A);
}


static void PID_ctrl_routine(void *pvParameters)
{
    static uint16_t mode_buffer = MODE_SLEEP;
    static uint16_t target_current_mA = 0;
    static uint16_t target_current_mA_buffer = 0;
    static uint32_t queue_recv_buffer = 0;

    static uint8_t initial_count = 0;
    static float output = 0.0f;

    static uint32_t *buf_ptr;

    while(1)
    {
        //1. 等待ADC数据
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) == pdTRUE)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
            //2. 查询target是否改变
            if(pdPASS == xQueueReceive(pid_ctrl_queue_mA, &queue_recv_buffer, 0))
            {
                mode_buffer = queue_recv_buffer >> 16;
                target_current_mA_buffer = queue_recv_buffer & 0xFFFF;
                if(mode != mode_buffer)
                {
                    mode = mode_buffer;
                    initial_count = 0;
                    pid_reset_ctrl_block(pid_handle);
                }
                if(target_current_mA != target_current_mA_buffer)
                {
                    target_current_mA = target_current_mA_buffer;
                }
            }
            adc_data_process(buf_ptr);

            switch(mode)
            {
                case MODE_SLEEP:
                    if(initial_count++ == 0)
                    {
                        HAL_HRTIM_WaveformCountStop(
                            &hhrtim1,HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
                        continue;
                    }

                    continue;
                case MODE_1TO2:
                    if(initial_count++ == 0)
                    {
                        HAL_HRTIM_WaveformCountStart(
                            &hhrtim1,HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
                        continue;
                        //计算faiA和faiC
                    }
                    if(now_low_voltage_mV <= 3000)
                    {
                        submode = 1;
                        target_current_mA = 500;
                    }
                    else
                    {
                        submode = 2;
                        target_current_mA = target_current_mA_buffer;
                    }
                    float error_mA = (float)target_current_mA - now_low_current_mA;
                    pid_compute(pid_handle, error_mA, &output);
                    if(output < 0.01f)
                        output = 0.0f;

                //转换成移相角/频率
                case MODE_2TO1:
                case MODE_AUTO:
                default:
                    break;
            }
            //4. 进行pid计算
            float error_mA = (float)target_current_mA - now_high_current_mA;

            pid_compute(pid_handle, error_mA, &output);
            if(output < 0.01f)
                output = 0.0f;
            // set_mod_ratio_by_factor(output);
            // LOGI("PID", "output: %.6f", output);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
        }
    }
}

void pid_set_current(uint32_t mA)
{
    if(NULL == pid_ctrl_queue_mA)
        xQueueSend(pid_ctrl_queue_mA, &mA, portMAX_DELAY);
}

void pid_ctrl_init(void)
{
    pid_ctrl_config_t pid_cfg = {
        .init_param = {
            .kp           = 0.00005f,
            .ki           = 0.000005f,
            .kd           = 0.00006f,
            .max_output   = 0.96f,
            .min_output   = 0.0f,
            .max_integral = 1000000.0f,
            .min_integral = -1000000.0f,
            .cal_type     = PID_CAL_TYPE_INCREMENTAL,
        }
    };
    pid_new_control_block(&pid_cfg, &pid_handle);
    pid_ctrl_queue_mA = xQueueCreate(6, sizeof(uint32_t));
    xTaskCreate(PID_ctrl_routine, "PID", 2048, NULL, 15, NULL);
    pid_set_current(0);
}
