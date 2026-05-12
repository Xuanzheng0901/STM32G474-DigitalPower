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
static uint16_t submode = 0;

int32_t get_pid_value(uint8_t index)
{
    if(index == 0)
        return (int32_t)now_high_voltage_mV;
    if(index == 1)
        return (int32_t)now_high_current_mA;
    if(index == 2)
        return (int32_t)now_low_voltage_mV;
    if(index == 3)
        return (int32_t)now_low_current_mA;
    if(index == 4)
        return mode | (submode << 8);

    return 0;
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
    int32_t i_sum[2] = {0};

    //低16位对应ADC1,差分采样电流, rank1对应高侧
    for(uint32_t i = 0; i < len; i += 2)
    {
        v_sum[0] += data_buf[i] >> 16 & 0x0FFF;
        i_sum[0] += (int32_t)(data_buf[i] & 0x0FFF) - 2047;
        v_sum[1] += data_buf[i + 1] >> 16 & 0xFFFF;
        i_sum[1] += (int32_t)(data_buf[i + 1] & 0xFFFF) - 2047;
    }

    // 计算平均值 (每个通道采样 len/2 次)
    float v_avg_high = (float)v_sum[0] / ((float)len / 2.0f);
    float v_avg_low = (float)v_sum[1] / ((float)len / 2.0f);
    float i_avg_high = (float)i_sum[0] / ((float)len / 2.0f);
    float i_avg_low = (float)i_sum[1] / ((float)len / 2.0f);

    if(i_avg_high <= 0.0f && i_avg_high >= -10.0f)
        i_avg_high = 0.0f;
    if(i_avg_low <= 0.0f && i_avg_low >= -10.0f)
        i_avg_low = 0.0f;

    float raw_high_voltage_mV = v_avg_high * (3000.0f / 4095.0f) * 6.666666f;
    float raw_high_current_A = i_avg_high * (3000.0f / 2048.0f) / 8.2f * 20.0f;
    float raw_low_voltage_mV = v_avg_low * (3000.0f / 4095.0f) * 6.666666f;
    float raw_low_current_A = i_avg_low * (3000.0f / 2048.0f) / 8.2f * 20.0f;

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

    now_high_voltage_mV = trunc(kalman_1d_update(&kf_high_voltage, raw_high_voltage_mV));
    now_high_current_mA = trunc(kalman_1d_update(&kf_high_current, raw_high_current_A));
    now_low_voltage_mV = trunc(kalman_1d_update(&kf_low_voltage, raw_low_voltage_mV));
    now_low_current_mA = trunc(kalman_1d_update(&kf_low_current, raw_low_current_A));
}

// ==========================================================
// 硬件参数与系统宏定义
// ==========================================================
#define HRTIM_CLK_HZ     5.44e9f  // 定时器主时钟: 170M * 32
#define FS_MAX           135000.0f      // 最高开关频率 (对应极轻载/空载)
#define FS_MIN           95000.0f       // 最低开关频率 (对应满载)
// #define SWITCH_DELAY_CYCLES 500
#define TRANSFORMER_N (3.33333f)

#define SWITCH_DELAY_CYCLES 50          // 模式切换时在SLEEP模式停留的周期数

// 宏定义：开启和关闭全部8个桥臂的输出
#define HRTIM_ALL_OUTPUTS (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                           HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
                           HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | \
                           HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2)

void HRTIM_Update_Timing(float fs_hz, float alpha_p_rad, float alpha_s_rad, float theta_rad, power_dir_t dir)
{
    // 1. 频率限幅保护
    if(fs_hz > FS_MAX)
        fs_hz = FS_MAX;
    if(fs_hz < FS_MIN)
        fs_hz = FS_MIN;

    // 2. 根据新频率计算最新的周期 Ticks
    uint32_t new_period_ticks = (uint32_t)(HRTIM_CLK_HZ / fs_hz);
    uint32_t half_period = new_period_ticks / 2;

    // 3. 根据最新的周期，将弧度(rad)转换为 Ticks
    float rad_to_ticks = (float)new_period_ticks / (2.0f * 3.14159265f);

    uint32_t alpha_p_t = (uint32_t)(alpha_p_rad * rad_to_ticks);
    uint32_t alpha_s_t = (uint32_t)(alpha_s_rad * rad_to_ticks);
    uint32_t theta_t = (uint32_t)(theta_rad * rad_to_ticks);

    // 4. 计算 Master Timer 负责同步的四个移相重置点 (CMP)
    uint32_t cmp2;

    uint32_t cmp1 = 0; // Timer A (初级上桥) 永远为基准0

    // Timer C (初级下桥) 滞后 A 180度 + alpha_p
    uint32_t cmp3 = (cmp1 + half_period + alpha_p_t) % new_period_ticks;

    // Timer B (次级上桥) 相对于 A 的偏移 theta
    if(dir == DIR_FORWARD)
    {
        cmp2 = (cmp1 + theta_t) % new_period_ticks;
    }
    else
    {
        // 反向：次级超前初级，相当于滞后 (Period - theta)
        cmp2 = (cmp1 + new_period_ticks - theta_t) % new_period_ticks;
    }

    // Timer D (次级下桥) 滞后 B 180度 + alpha_s
    uint32_t cmp4 = (cmp2 + half_period + alpha_s_t) % new_period_ticks;

    // --------------------------------------------------------
    // 5. 硬件极值约束保护
    // --------------------------------------------------------
    // Master CMP2 和 CMP4 最小不能低于 96 (硬件限制)
    if(cmp2 < 96)
        cmp2 = 96;
    if(cmp4 < 96)
        cmp4 = 96;

    // CMP 不能大于等于 Period，否则无法触发同步
    if(cmp2 >= new_period_ticks)
        cmp2 = new_period_ticks - 1;
    if(cmp3 >= new_period_ticks)
        cmp3 = new_period_ticks - 1;
    if(cmp4 >= new_period_ticks)
        cmp4 = new_period_ticks - 1;

    // --------------------------------------------------------
    // 6. 一次性将所有值写入 HRTIM 寄存器
    // (依赖 HRTIM 的 Preload 影子寄存器机制同步生效)
    // --------------------------------------------------------
    // 更新周期
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, new_period_ticks);

    // 更新 A, B, C, D 的 50% 占空比翻转点
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, half_period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, half_period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, half_period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, half_period);

    // 更新 Master 的 4 个移相控制点
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, cmp1);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, cmp2);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, cmp3);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, cmp4);

    // opt: 将计数器置0, 但是不置0也不会有什么问题
    // __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_MASTER, 0);
    // __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    // __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);
    // __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, 0);
    // __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, 0);
}

// ==========================================================
// SPS 控制宏定义
// ==========================================================
#define NOMINAL_FREQ     90000.0f  // 固定频率 100kHz
#define THETA_MAX_RAD    (1.5707963f-0.0174533f*30.0f)   // 最大移相角 90度 (PI/2)
#define THETA_MIN_RAD    (-0.0174533f*2.0f)       // 最小移相角 0度

static void PID_ctrl_routine(void *pvParameters)
{
    uint8_t pid_tick = 0;
    static uint16_t target_current_mA = 0;
    static uint16_t target_current_mA_buffer = 0;
    static uint32_t queue_recv_buffer = 0;
    static uint32_t mode_tick_count = 0;

    // PID 变量：此时 PID 输出的是移相角 theta
    static float theta_output = 0.0f;
    static float error_mA = 0.0f;
    static uint32_t *buf_ptr;

    static mode_switch_state_t switch_state = SWITCH_STATE_IDLE;
    static uint16_t switch_timer = 0;
    static uint16_t target_mode_request = MODE_SLEEP;

    // SPS 模式下，alpha 始终为 0
    float alpha_p = 0.0f;
    float alpha_s = 0.0f;
    power_dir_t current_dir = DIR_FORWARD;

    float current_freq = NOMINAL_FREQ;

    while(1)
    {
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) == pdTRUE)
        {
            // 1. 处理模式切换逻辑 (保持不变)
            if(pdPASS == xQueueReceive(pid_ctrl_queue_mA, &queue_recv_buffer, 0))
            {
                uint16_t recv_mode = queue_recv_buffer >> 16;
                uint16_t recv_target = queue_recv_buffer & 0xFFFF;

                if(mode != recv_mode && switch_state == SWITCH_STATE_IDLE)
                {
                    target_mode_request = recv_mode;
                    target_current_mA_buffer = recv_target;
                    mode = MODE_SLEEP;
                    switch_state = SWITCH_STATE_SLEEP_TRANSITION;
                    switch_timer = 0;
                    mode_tick_count = 0;
                    pid_reset_ctrl_block(pid_handle);
                    theta_output = 0.0f; // 切换时重置移相角为 0
                }
                else if(switch_state == SWITCH_STATE_IDLE)
                {
                    target_current_mA_buffer = recv_target;
                }
            }

            adc_data_process(buf_ptr);

            if(switch_state == SWITCH_STATE_SLEEP_TRANSITION)
            {
                if(++switch_timer >= SWITCH_DELAY_CYCLES)
                {
                    mode = target_mode_request;
                    target_current_mA = target_current_mA_buffer;
                    switch_state = SWITCH_STATE_IDLE;
                    mode_tick_count = 0;
                }
            }

            float v_high = now_high_voltage_mV < 10.0f ? 10.0f : now_high_voltage_mV;
            float v_low = now_low_voltage_mV;

            // 计算当前的电压增益 M
            float M = TRANSFORMER_N * v_low / v_high;

            if(M <= 1.0f)
            {
                alpha_s = 0.0f;
                // 用 fmaxf 保证开方不为负数
                theta_output = asinf(sqrtf(fmaxf(0.0f, 1.0f - M)));
                alpha_p = theta_output;
                // alpha_p = 0.0f;
            }
            else
            {
                alpha_p = 0.0f;
                theta_output = asinf(sqrtf(fmaxf(0.0f, 1.0f - 1.0f / M)));
                alpha_s = theta_output;
                // alpha_s = 0.0f;
            }

            // 2. 执行模式具体逻辑
            switch(mode)
            {
                case MODE_SLEEP:
                    if(mode_tick_count == 0)
                    {
                        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_ALL_OUTPUTS);
                        HRTIM_Update_Timing(NOMINAL_FREQ, 0, 0, 0, DIR_FORWARD);
                        mode_tick_count++;
                    }
                    continue;

                case MODE_1TO2: // 正向充电
                    if(now_low_voltage_mV <= 3000)
                    {
                        submode = 1;
                        target_current_mA = 500;
                        current_freq = 135000.0f;
                    }

                    else
                    {
                        submode = 2;
                        target_current_mA = target_current_mA_buffer;
                        if(target_current_mA < 500)
                        {
                            target_current_mA = 500;
                        }
                        current_freq = NOMINAL_FREQ + (3100 - target_current_mA) * 15.0f;
                        // alpha_p = 0.0f;
                        // alpha_s = 0.0f;
                    }
                    alpha_p = 0.0f;
                    // alpha_s = 0.0f;


                    error_mA = (float)target_current_mA - now_low_current_mA;
                    current_dir = DIR_FORWARD;

                    if(mode_tick_count == 0)
                    {
                        // SPS 模式下，移相角越大功率越大，所以 Kp 为正
                        pid_ctrl_parameter_t sps_param = {
                            .kp           = 0.0001f,     // SPS 角度很敏感，Kp 从小开始
                            .ki           = 0.00001f,
                            .kd           = 0.00001f,
                            .max_output   = THETA_MAX_RAD,
                            .min_output   = THETA_MIN_RAD,
                            .max_integral = 100.0f,
                            .min_integral = -100.0f,
                            .cal_type     = PID_CAL_TYPE_INCREMENTAL,
                        };
                        pid_update_parameters(pid_handle, &sps_param);
                        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                        mode_tick_count++;
                    }
                    break;

                case MODE_2TO1: // 反向放电
                    if(now_high_voltage_mV <= 10000)
                    {
                        submode = 1;
                        target_current_mA = 100;
                        current_freq = 135000.0f;
                    }

                    else
                    {
                        submode = 2;
                        target_current_mA = target_current_mA_buffer;
                        if(target_current_mA < 100)
                        {
                            target_current_mA = 100;
                        }
                        current_freq = NOMINAL_FREQ + (1000 - target_current_mA) * 40.0f;
                    }

                    alpha_p = 0.0f;

                    error_mA = (float)target_current_mA + now_high_current_mA;
                    current_dir = DIR_REVERSE;

                    if(mode_tick_count == 0)
                    {
                        pid_ctrl_parameter_t sps_param = {
                            .kp           = 0.0001f,
                            .ki           = 0.00001f,
                            .kd           = 0.00001f,
                            .max_output   = THETA_MAX_RAD,
                            .min_output   = 0.0f,
                            .max_integral = 100.0f,
                            .min_integral = -100.0f,
                            .cal_type     = PID_CAL_TYPE_INCREMENTAL,
                        };
                        pid_update_parameters(pid_handle, &sps_param);
                        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                        mode_tick_count++;
                    }
                    break;

                default:
                    continue;
            }

            // 3. PID 计算：输出目标 theta
            pid_compute(pid_handle, error_mA, &theta_output);

            // 4. 更新硬件：固定频率，调节 theta，alpha 保持为 0
            HRTIM_Update_Timing(NOMINAL_FREQ, alpha_p, alpha_s, theta_output, current_dir);
            if(++pid_tick >= 100)
            {
                LOGI("PID", "%.2f %.2f %.2f %.2f", now_low_current_mA, now_high_current_mA, error_mA, theta_output);
                pid_tick = 0;
            }

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        }
    }
}

void pid_set_current(uint32_t mA)
{
    uint32_t buf = mode << 16 | mA;
    if(NULL != pid_ctrl_queue_mA)
        xQueueSend(pid_ctrl_queue_mA, &buf, portMAX_DELAY);
}

void pid_ctrl_init(void)
{
    pid_ctrl_config_t pid_cfg = {
        .init_param = {
            .kp           = -10.0f,
            .ki           = -1.0f,
            .kd           = 0.0f,
            .max_output   = FS_MAX,
            .min_output   = FS_MIN,
            .max_integral = 30000.0f,
            .min_integral = -30000.0f,
            .cal_type     = PID_CAL_TYPE_POSITIONAL,
        }
    };
    pid_new_control_block(&pid_cfg, &pid_handle);
    pid_ctrl_queue_mA = xQueueCreate(6, sizeof(uint32_t));
    xTaskCreate(PID_ctrl_routine, "PID", 2048, NULL, 15, NULL);
}
