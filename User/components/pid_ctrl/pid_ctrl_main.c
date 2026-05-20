#include "pid_ctrl_internal.h"
#include "FreeRTOS.h"
#include <math.h>
#include "hrtim.h"
#include "main.h"
#include "task.h"
#include "queue.h"
#include "kalman.h"
#include "PID.h"

/* ================================================================
 *  硬件与控制参数宏定义
 * ================================================================ */
#define HRTIM_CLK_HZ        5.44e9f
#define TRANSFORMER_N       3.33333f    /* 匝比 10:3 ≈ 3.33 */

/* 频率调节范围 */
#define FS_MAX              200000.0f   /* 频率上限（最小功率点）*/
#define FS_MIN               95000.0f   /* 频率下限（最大功率点，须 > fr）*/

/* 移相角限制 (弧度) */
#define THETA_MAX           1.50f       /* 约 86 度 */
#define THETA_MIN           0.01f       /* 趋近 0 度 */
#define THETA_FF_BASE       0.60f       /* M=1 时的基础前馈角度 (约 35 度)，保证功率基数 */

#define SWITCH_DELAY_CYCLES    50
#define HRTIM_ALL_OUTPUTS (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                           HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
                           HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | \
                           HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2)

/* ================================================================
 *  全局变量
 * ================================================================ */
extern QueueHandle_t adc_queue;
static pid_ctrl_block_handle_t pid_handle = NULL;
QueueHandle_t pid_ctrl_queue_mA = NULL;

static uint32_t now_low_voltage_mV = 0, now_high_voltage_mV = 0;
static int32_t now_high_current_mA = 0, now_low_current_mA = 0;
static uint16_t mode = MODE_SLEEP;
static uint16_t submode = 0;

int32_t get_pid_value(uint8_t index)
{
    switch(index)
    {
        case 0:
            return (int32_t)now_high_voltage_mV;
        case 1:
            return now_high_current_mA;
        case 2:
            return (int32_t)now_low_voltage_mV;
        case 3:
            return now_low_current_mA;
        case 4:
            return mode | (submode << 8);
        default:
            return 0;
    }
}

/* ================================================================
 *  ADC 数据处理（Kalman 滤波，保持原版）
 * ================================================================ */
static void adc_data_process(uint32_t *data_buf)
{
    static kalman_1d_state_t kf_high_voltage;
    static kalman_1d_state_t kf_high_current;
    static kalman_1d_state_t kf_low_voltage;
    static kalman_1d_state_t kf_low_current;
    static uint8_t is_kf_initialized = 0;
    uint16_t len = ADC_BUFFER_LENGTH / 2;

    uint32_t v_sum[2] = {0};
    int32_t i_sum[2] = {0};

    for(uint32_t i = 0; i < len; i += 2)
    {
        v_sum[0] += data_buf[i] >> 16 & 0x0FFF;
        i_sum[0] += (int32_t)(data_buf[i] & 0x0FFF) - 2047;
        v_sum[1] += data_buf[i + 1] >> 16 & 0xFFFF;
        i_sum[1] += (int32_t)(data_buf[i + 1] & 0xFFFF) - 2047;
    }

    float v_avg_high = (float)v_sum[0] / ((float)len / 2.0f);
    float v_avg_low = (float)v_sum[1] / ((float)len / 2.0f);
    float i_avg_high = (float)i_sum[0] / ((float)len / 2.0f);
    float i_avg_low = (float)i_sum[1] / ((float)len / 2.0f);

    /* 消除传感器零漂小偏置 */
    if(i_avg_high <= 0.0f && i_avg_high >= -10.0f)
        i_avg_high = 0.0f;
    if(i_avg_low <= 0.0f && i_avg_low >= -10.0f)
        i_avg_low = 0.0f;

    float raw_high_voltage_mV = v_avg_high * (3000.0f / 4095.0f) * 6.666666f;
    float raw_high_current_mA = (i_avg_high * (3000.0f / 4095.0f) / 8.2f * 20.0f);
    float raw_low_voltage_mV = v_avg_low * (3000.0f / 4095.0f) * 6.666666f;
    float raw_low_current_mA = (i_avg_low * (3000.0f / 4095.0f) / 8.2f * 20.0f);

    if(!is_kf_initialized)
    {
        kalman_1d_init(&kf_high_voltage, raw_high_voltage_mV, 10.0f, 0.5f, 50.0f);
        kalman_1d_init(&kf_high_current, raw_high_current_mA, 1.0f, 0.01f, 1.0f);
        kalman_1d_init(&kf_low_voltage, raw_low_voltage_mV, 10.0f, 0.5f, 50.0f);
        kalman_1d_init(&kf_low_current, raw_low_current_mA, 1.0f, 0.01f, 1.0f);
        is_kf_initialized = 1;
    }

    now_high_voltage_mV = (uint32_t)kalman_1d_update(&kf_high_voltage, raw_high_voltage_mV);
    now_high_current_mA = (int32_t)kalman_1d_update(&kf_high_current, raw_high_current_mA);
    now_low_voltage_mV = (uint32_t)kalman_1d_update(&kf_low_voltage, raw_low_voltage_mV);
    now_low_current_mA = (int32_t)kalman_1d_update(&kf_low_current, raw_low_current_mA);
}

/* ================================================================
 *  原子化 HRTIM 时序更新函数
 * ================================================================ */
void HRTIM_Update_Timing(float fs_hz, float alpha_p_rad, float alpha_s_rad, float theta_rad, power_dir_t dir)
{
    /* 频率硬限幅 */
    if(fs_hz > 250000.0f)
        fs_hz = 250000.0f;
    if(fs_hz < 80000.0f)
        fs_hz = 80000.0f;

    uint32_t period = (uint32_t)(HRTIM_CLK_HZ / fs_hz);
    uint32_t half_p = period / 2;
    float rad_to_ticks = (float)period / (2.0f * 3.14159265f);

    uint32_t alpha_p_t = (uint32_t)(alpha_p_rad * rad_to_ticks);
    uint32_t alpha_s_t = (uint32_t)(alpha_s_rad * rad_to_ticks);
    uint32_t theta_t = (uint32_t)(theta_rad * rad_to_ticks);

    uint32_t cmp1 = 0;
    uint32_t cmp3 = (cmp1 + half_p + alpha_p_t) % period;
    uint32_t cmp2 = (dir == DIR_FORWARD) ? (cmp1 + theta_t) % period : (cmp1 + period - theta_t) % period;
    uint32_t cmp4 = (cmp2 + half_p + alpha_s_t) % period;

    /* 硬件极值保护 */
    if(cmp2 < 96)
        cmp2 = 96;
    if(cmp4 < 96)
        cmp4 = 96;
    if(cmp2 >= period)
        cmp2 = period - 1;
    if(cmp3 >= period)
        cmp3 = period - 1;
    if(cmp4 >= period)
        cmp4 = period - 1;

    /* 影子寄存器写入 */
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, period);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, period);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, period);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, period);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, period);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, half_p);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, half_p);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, half_p);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, half_p);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, cmp1);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, cmp2);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, cmp3);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, cmp4);
}

/* ================================================================
 *  PID 参数定义 (增量式，正增益)
 *  正增益逻辑：error > 0 -> pid_output 增大 -> 增加传输功率
 * ================================================================ */
static const pid_ctrl_parameter_t HYBRID_PID_PARAM = {
    .kp           = 0.00005f,   /* 针对 0.0~1.0 虚拟量程标定 */
    .ki           = 0.00002f,
    .kd           = 0.000001f,
    .max_output   = 1.5f,       /* 允许进入升相区 */
    .min_output   = -0.5f,      /* 允许进入降相区 */
    .max_integral = 50.0f,
    .min_integral = -50.0f,
    .cal_type     = PID_CAL_TYPE_INCREMENTAL,
};

/* ================================================================
 *  PID 控制任务
 * ================================================================ */
static void PID_ctrl_routine(void *pvParameters)
{
    uint8_t log_tick = 0;
    uint32_t *buf_ptr = NULL;

    static uint16_t target_mA = 0;
    static uint16_t target_mA_buffer = 0;
    static uint32_t queue_buf = 0;
    static uint32_t mode_tick = 0;

    static mode_switch_state_t sw_state = SWITCH_STATE_IDLE;
    static uint16_t sw_timer = 0;
    static uint16_t target_mode_req = MODE_SLEEP;

    /* 控制核心变量 */
    static float pid_output = 0.0f; /* 统一控制油门 [-0.5, 1.5] */
    float error_mA = 0.0f;
    float alpha_p = 0.0f, alpha_s = 0.0f;
    float final_theta = 0.0f, final_fs = FS_MAX;
    power_dir_t current_dir = DIR_FORWARD;

    while(1)
    {
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) != pdTRUE)
            continue;

        /* 1. 处理指令队列 */
        if(xQueueReceive(pid_ctrl_queue_mA, &queue_buf, 0) == pdPASS)
        {
            uint16_t recv_mode = (uint16_t)(queue_buf >> 16);
            uint16_t recv_target = (uint16_t)(queue_buf & 0xFFFF);

            if(recv_mode != mode && sw_state == SWITCH_STATE_IDLE)
            {
                target_mode_req = recv_mode;
                target_mA_buffer = recv_target;
                mode = MODE_SLEEP;
                sw_state = SWITCH_STATE_SLEEP_TRANSITION;
                sw_timer = 0;
                mode_tick = 0;
                pid_output = 0.0f;
                pid_reset_ctrl_block(pid_handle);
            }
            else if(sw_state == SWITCH_STATE_IDLE)
            {
                target_mA_buffer = recv_target;
            }
        }

        adc_data_process(buf_ptr);

        /* 2. 模式切换计时 */
        if(sw_state == SWITCH_STATE_SLEEP_TRANSITION)
        {
            if(++sw_timer >= SWITCH_DELAY_CYCLES)
            {
                mode = target_mode_req;
                target_mA = target_mA_buffer;
                sw_state = SWITCH_STATE_IDLE;
                mode_tick = 0;
            }
        }

        /* 3. 计算电压增益 M 与 桥内移相 alpha (效率环) */
        float v_high = (float)(now_high_voltage_mV < 8000 ? 8000 : now_high_voltage_mV);
        float v_low = (float)(now_low_voltage_mV < 2500 ? 2500 : now_low_voltage_mV);
        float M = TRANSFORMER_N * v_low / v_high;

        if(M <= 1.0f)
        {
            if(M > 0.9f)
                M = 0.9f;
            alpha_s = 0.0f;
            /* 桥内移相 alpha_p 负责匹配压差以消除环流: cos(alpha) = M */
            alpha_p = acosf(fminf(1.0f, M));
        }
        else
        {
            if(M < 1.1f)
                M = 1.1f;
            alpha_p = 0.0f;
            alpha_s = acosf(fminf(1.0f, 1.0f / M));
        }

        /* 4. 计算桥间移相前馈 theta_ff (基础挡位) */
        float theta_ff = 0.0f;
        if(M <= 1.0f)
            theta_ff = asinf(sqrtf(fmaxf(0.0f, 1.0f - M)));
        else
            theta_ff = asinf(sqrtf(fmaxf(0.0f, 1.0f - 1.0f / M)));

        /* 关键：给前馈角度一个“底薪”，防止 M=1 时角度过小导致 3A 输不出 */
        theta_ff = fmaxf(theta_ff, THETA_FF_BASE);

        /* 5. 模式特定逻辑 */
        switch(mode)
        {
            case MODE_SLEEP:
                if(mode_tick == 0)
                {
                    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    HRTIM_Update_Timing(FS_MAX, 0, 0, 0, DIR_FORWARD);
                    mode_tick++;
                }
                continue;

            case MODE_1TO2:
                current_dir = DIR_FORWARD;
                target_mA = (now_low_voltage_mV < 3000) ? 500 : target_mA_buffer;
                if(target_mA > 3000)
                    target_mA = 3000;
                if(target_mA < 500)
                    target_mA = 500;
                error_mA = (float)target_mA - (float)now_low_current_mA;
                if(mode_tick == 0)
                {
                    pid_update_parameters(pid_handle, &HYBRID_PID_PARAM);
                    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    mode_tick++;
                }
                break;

            case MODE_2TO1:
                current_dir = DIR_REVERSE;
                target_mA = (now_high_voltage_mV < 10000) ? 100 : target_mA_buffer;
                if(target_mA < 100)
                    target_mA = 100;
                if(target_mA > 1000)
                    target_mA = 1000;

                /* 反向：error = 目标 - |实际| */
                error_mA = (float)target_mA + (float)now_high_current_mA;
                if(mode_tick == 0)
                {
                    pid_update_parameters(pid_handle, &HYBRID_PID_PARAM);
                    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    mode_tick++;
                }
                break;

            case MODE_AUTO:
                target_mA = target_mA_buffer;
                /* 滞回逻辑 */
                if(now_high_voltage_mV <= 11000)
                {
                    current_dir = DIR_REVERSE;
                    error_mA = (float)target_mA + (float)now_high_current_mA;
                }
                else if(now_high_voltage_mV >= 11500)
                {
                    current_dir = DIR_FORWARD;
                    error_mA = (float)target_mA - (float)now_low_current_mA;
                }
                if(mode_tick == 0)
                {
                    pid_update_parameters(pid_handle, &HYBRID_PID_PARAM);
                    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    mode_tick++;
                }
                break;
            default:
                continue;
        }

        /* 6. 统一油门 PID 计算与三段映射 */
        pid_compute(pid_handle, error_mA, &pid_output);

        /*
         * 映射逻辑设计：
         * [0.0, 1.0] 段：调频区。theta 固定为 theta_ff，fs 从 FS_MAX 降到 FS_MIN
         * [-0.5, 0.0] 段：降相区（极轻载）。fs 固定为 FS_MAX，theta 从 theta_ff 线性降向 0
         * [1.0, 1.5]  段：升相区（超重载）。fs 固定为 FS_MIN，theta 从 theta_ff 线性升向 THETA_MAX
         */
        if(pid_output < 0.0f)
        {
            final_fs = FS_MAX;
            /* 在 0.0 到 -0.5 之间，角度从 theta_ff 降到 0 */
            final_theta = theta_ff * (1.0f + pid_output * 2.0f);
        }
        else if(pid_output <= 1.0f)
        {
            final_theta = theta_ff;
            /* 在 0.0 到 1.0 之间，频率从 FS_MAX 降到 FS_MIN */
            final_fs = FS_MAX - (pid_output * (FS_MAX - FS_MIN));
        }
        else
        {
            final_fs = FS_MIN;
            /* 在 1.0 到 1.5 之间，角度从 theta_ff 升向上限 */
            final_theta = theta_ff + (pid_output - 1.0f) * 2.0f * (THETA_MAX - theta_ff);
        }

        /* 安全二次限幅 */
        if(final_theta < THETA_MIN)
            final_theta = THETA_MIN;
        if(final_theta > THETA_MAX)
            final_theta = THETA_MAX;

        /* 7. 更新硬件 */
        HRTIM_Update_Timing(final_fs, alpha_p, alpha_s, final_theta, current_dir);

        /* 8. 调试打印 */
        if(++log_tick >= 100)
        {
            LOGI("PID", "M=%.2f ap=%.2f th=%.2f fs=%.0f p_out=%.2f err=%.0f",
                 M, alpha_p+alpha_s, final_theta, final_fs, pid_output, error_mA);
            log_tick = 0;
        }
    }
}

/* ================================================================
 *  外部接口
 * ================================================================ */
void pid_set_current(uint32_t mA)
{
    uint32_t buf = ((uint32_t)mode << 16) | (mA & 0xFFFF);
    if(pid_ctrl_queue_mA != NULL)
        xQueueSend(pid_ctrl_queue_mA, &buf, portMAX_DELAY);
}

void pid_ctrl_init(void)
{
    pid_ctrl_config_t cfg = {
        .init_param = HYBRID_PID_PARAM,
    };
    pid_new_control_block(&cfg, &pid_handle);
    pid_ctrl_queue_mA = xQueueCreate(6, sizeof(uint32_t));
    xTaskCreate(PID_ctrl_routine, "PID", 2048, NULL, 15, NULL);
}
