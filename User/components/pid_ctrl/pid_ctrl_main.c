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
QueueHandle_t pid_ctrl_queue_mA = NULL;
static uint32_t now_low_voltage_mV = 0, now_high_voltage_mV = 0;
static int32_t now_high_current_mA = 0, now_low_current_mA = 0;
static uint16_t mode = MODE_SLEEP;
static uint16_t submode = 0;

/* ================================================================
 *  公共状态读取接口
 * ================================================================ */
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
 *  硬件参数宏定义
 * ================================================================ */
#define HRTIM_CLK_HZ        5.44e9f

/*
 * 频率范围：须始终工作在谐振频率以上（过谐振区），保证 ZVS。
 * 与谐振频率 fr 的关系：FS_MIN > fr（此处 fr ≈ 91 kHz）
 * 频率越低 → Xp.u 越小 → 电流越大（满载）
 * 频率越高 → Xp.u 越大 → 电流越小（空载/轻载）
 */
#define FS_MAX              200000.0f   /* 最高频率 → 最小电流（轻载/初始化）*/
#define FS_MIN               90000.0f   /* 最低频率 → 最大电流（须 > fr）    */

#define TRANSFORMER_N         3.33333f  /* 变压器匝数比 n = N1/N2             */
#define SWITCH_DELAY_CYCLES      50     /* 模式切换时 SLEEP 停留周期数         */

/*
 * M 裁剪范围（参考 Yaqoob 2019 Table I）
 * M 在 [0.95, 1.05] 区间内角度退化，裁剪后维持最小可用角度
 */
#define M_BUCK_CLIP   0.9f   /* M ≤ 1 时的上限裁剪 */
#define M_BOOST_CLIP  1.1f   /* M > 1 时的下限裁剪 */
#define M_ABS_MIN     0.20f
#define M_ABS_MAX     2.00f

#define HRTIM_ALL_OUTPUTS (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                           HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
                           HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | \
                           HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2)

/* ================================================================
 *  HRTIM 时序更新（保持原版，无改动）
 * ================================================================ */
void HRTIM_Update_Timing(float fs_hz, float alpha_p_rad, float alpha_s_rad,
                         float theta_rad, power_dir_t dir)
{
    if(fs_hz > FS_MAX)
        fs_hz = FS_MAX;
    if(fs_hz < FS_MIN)
        fs_hz = FS_MIN;

    uint32_t new_period_ticks = (uint32_t)(HRTIM_CLK_HZ / fs_hz);
    uint32_t half_period = new_period_ticks / 2;
    float rad_to_ticks = (float)new_period_ticks / (2.0f * 3.14159265f);

    uint32_t alpha_p_t = (uint32_t)(alpha_p_rad * rad_to_ticks);
    uint32_t alpha_s_t = (uint32_t)(alpha_s_rad * rad_to_ticks);
    uint32_t theta_t = (uint32_t)(theta_rad * rad_to_ticks);

    uint32_t cmp1 = 0;
    uint32_t cmp3 = (cmp1 + half_period + alpha_p_t) % new_period_ticks;
    uint32_t cmp2 = (dir == DIR_FORWARD)
                        ? (cmp1 + theta_t) % new_period_ticks
                        : (cmp1 + new_period_ticks - theta_t) % new_period_ticks;
    uint32_t cmp4 = (cmp2 + half_period + alpha_s_t) % new_period_ticks;

    if(cmp2 < 96)
        cmp2 = 96;
    if(cmp4 < 96)
        cmp4 = 96;
    if(cmp2 >= new_period_ticks)
        cmp2 = new_period_ticks - 1;
    if(cmp3 >= new_period_ticks)
        cmp3 = new_period_ticks - 1;
    if(cmp4 >= new_period_ticks)
        cmp4 = new_period_ticks - 1;

    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, new_period_ticks);
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, new_period_ticks);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, half_period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, half_period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, half_period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, half_period);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, cmp1);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, cmp2);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, cmp3);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, cmp4);
}

/* ================================================================
 *  4-DOF 最优角度计算
 *
 *  依据 Yaqoob 2019 eq.(20)：
 *    M ≤ 1 → αp = θ = arcsin(√(1-M)),   αs = 0
 *    M > 1 → αs = θ = arcsin(√(1-1/M)), αp = 0
 *
 *  注意：θ 与 αp/αs 完全由 M 决定，与频率无关。
 *        频率 fs 是独立的功率调节量（由 PID 控制）。
 * ================================================================ */
static void calc_4dof_angles(float M, float *alpha_p, float *alpha_s, float *theta)
{
    /* 全局限幅 */
    if(M < M_ABS_MIN)
        M = M_ABS_MIN;
    if(M > M_ABS_MAX)
        M = M_ABS_MAX;

    if(M <= 1.0f)
    {
        /*
         * 当 M 接近 1 时 sqrt(1-M)→0，θ→0，软开关条件退化。
         * 按论文 Table I 裁剪到 0.95，维持最小可用角度。
         */
        if(M > M_BUCK_CLIP)
            M = M_BUCK_CLIP;

        float ang = asinf(sqrtf(1.0f - M));
        *theta = ang;
        *alpha_p = ang;
        *alpha_s = 0.0f;
    }
    else
    {
        /* 同理，M 接近 1 时裁剪到 1.05 */
        if(M < M_BOOST_CLIP)
            M = M_BOOST_CLIP;

        float ang = asinf(sqrtf(1.0f - 1.0f / M));
        *theta = ang;
        *alpha_s = ang;
        *alpha_p = 0.0f;
    }
}

/* ================================================================
 *  PID 参数（调频模式，增量式，负增益）
 *
 *  负增益原因：
 *    fs ↑  →  Xp.u ↑  →  电流 ↓
 *    误差 > 0（需要更多电流）→ PID 输出负增量 → fs ↓ → 电流 ↑
 * ================================================================ */
static const pid_ctrl_parameter_t FM_PID_PARAM = {
    .kp           = -5.0f,
    .ki           = -1.5f,
    .kd           = -0.1f,
    .max_output   = FS_MAX,
    .min_output   = FS_MIN,
    .max_integral = 30000.0f,
    .min_integral = -30000.0f,
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

    /* 控制量 */
    static float fs_output = FS_MAX;
    float error_mA = 0.0f;
    float alpha_p = 0.0f;
    float alpha_s = 0.0f;
    float theta = 0.0f;
    power_dir_t current_dir = DIR_FORWARD;

    while(1)
    {
        if(xQueueReceive(adc_queue, &buf_ptr, portMAX_DELAY) != pdTRUE)
            continue;

        /* ------------------------------------------------------------
         * 1. 处理上层命令（模式切换 / 目标电流更新）
         * ------------------------------------------------------------ */
        if(xQueueReceive(pid_ctrl_queue_mA, &queue_buf, 0) == pdPASS)
        {
            uint16_t recv_mode = (uint16_t)(queue_buf >> 16);
            uint16_t recv_target = (uint16_t)(queue_buf & 0xFFFF);

            if(recv_mode != mode && sw_state == SWITCH_STATE_IDLE)
            {
                /* 需要切换模式：先拉停输出，经 SLEEP 过渡再启动 */
                target_mode_req = recv_mode;
                target_mA_buffer = recv_target;
                mode = MODE_SLEEP;
                sw_state = SWITCH_STATE_SLEEP_TRANSITION;
                sw_timer = 0;
                mode_tick = 0;
                fs_output = FS_MAX;          /* 复位到最小功率频率 */
                pid_reset_ctrl_block(pid_handle);
            }
            else if(sw_state == SWITCH_STATE_IDLE)
            {
                target_mA_buffer = recv_target;
                target_mA = recv_target;     /* 立即生效 */
            }
        }

        /* ------------------------------------------------------------
         * 2. ADC 数据处理
         * ------------------------------------------------------------ */
        adc_data_process(buf_ptr);

        /* ------------------------------------------------------------
         * 3. 模式切换过渡计时
         * ------------------------------------------------------------ */
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

        /* ------------------------------------------------------------
         * 4. 计算电压增益 M
         *    下限保护：防止除零和电压过低时 M 失真
         * ------------------------------------------------------------ */
        float v_high = (float)(now_high_voltage_mV < 8000 ? 8000 : now_high_voltage_mV);
        float v_low = (float)(now_low_voltage_mV < 2500 ? 2500 : now_low_voltage_mV);
        float M = TRANSFORMER_N * v_low / v_high;

        /* ------------------------------------------------------------
         * 5. 各工作模式逻辑
         * ------------------------------------------------------------ */
        switch(mode)
        {
            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
             * MODE_SLEEP：停机状态
             * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            case MODE_SLEEP:
                if(mode_tick == 0)
                {
                    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    HRTIM_Update_Timing(FS_MAX, 0.0f, 0.0f, 0.0f, DIR_FORWARD);
                    mode_tick++;
                }
                continue; /* 不执行后续 PID / HRTIM 更新 */

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
             * MODE_1TO2：高侧→低侧，全程恒流
             *   外部电子负载 CV 钳位 Vb，无需控制恒压
             *   传感器约定：now_low_current_mA > 0 表示电流流入低侧（充电）
             * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            case MODE_1TO2:
                current_dir = DIR_FORWARD;

                /*
                 * 低压预充保护：Vb < 3.0 V 时限制为 500 mA，
                 * 电压正常后恢复为用户设定值。
                 * （即使外部电子负载钳压，过低电压下仍需限流保护）
                 */
                target_mA = (now_low_voltage_mV < 3000) ? 500 : target_mA_buffer;
                if(target_mA < 100)
                    target_mA = 100;
                if(target_mA > 3000)
                    target_mA = 3000;

                error_mA = (float)target_mA - (float)now_low_current_mA;

                if(mode_tick == 0)
                {
                    pid_update_parameters(pid_handle, &FM_PID_PARAM);
                    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    mode_tick++;
                }
                break;

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
             * MODE_2TO1：低侧→高侧，全程恒流
             *   外部电子负载 CV 钳位 Va，无需控制恒压
             *   传感器约定：反向充电时 now_high_current_mA < 0
             *     → error = target + now_high_current_mA = target - |I_high|
             * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            case MODE_2TO1:
                current_dir = DIR_REVERSE;

                /* 高压预充保护：Va < 10.0 V 时限制为 100 mA */
                target_mA = (now_high_voltage_mV < 10000) ? 100 : target_mA_buffer;
                if(target_mA < 100)
                    target_mA = 100;
                if(target_mA > 1000)
                    target_mA = 1000;

                error_mA = (float)target_mA + (float)now_high_current_mA;

                if(mode_tick == 0)
                {
                    pid_update_parameters(pid_handle, &FM_PID_PARAM);
                    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    mode_tick++;
                }
                break;

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
             * MODE_AUTO：根据高侧电压自动判断方向（含滞回防抖）
             *   滞回区间 11.0~11.5 V：保持上一次的方向和误差，避免频繁切换
             * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            case MODE_AUTO:
                target_mA = target_mA_buffer;
                if(target_mA < 500)
                    target_mA = 500;

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
                /* 11000~11500 mV：current_dir 和 error_mA 维持上次值（滞回） */

                if(mode_tick == 0)
                {
                    /*
                     * 首次进入 AUTO：根据当前电压确定初始方向，
                     * 避免首拍在滞回区内 current_dir 为 DIR_FORWARD 默认值
                     */
                    if(now_high_voltage_mV < 11250)
                    {
                        current_dir = DIR_REVERSE;
                        error_mA = (float)target_mA + (float)now_high_current_mA;
                    }
                    else
                    {
                        current_dir = DIR_FORWARD;
                        error_mA = (float)target_mA - (float)now_low_current_mA;
                    }
                    pid_update_parameters(pid_handle, &FM_PID_PARAM);
                    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ALL_OUTPUTS);
                    mode_tick++;
                }
                break;

            default:
                continue;
        }

        /* ------------------------------------------------------------
         * 6. PID 计算 → 输出目标频率 fs_output
         * ------------------------------------------------------------ */
        pid_compute(pid_handle, error_mA, &fs_output);

        /* ------------------------------------------------------------
         * 7. 根据 M 计算 4-DOF 最优角度
         *    ★ θ 仅由 M 决定，与 fs 无关 ★
         * ------------------------------------------------------------ */
        calc_4dof_angles(M, &alpha_p, &alpha_s, &theta);

        /* ------------------------------------------------------------
         * 8. 更新 HRTIM 硬件
         * ------------------------------------------------------------ */
        HRTIM_Update_Timing(fs_output, alpha_p, alpha_s, theta, current_dir);

        /* ------------------------------------------------------------
         * 9. 调试日志（每 100 个 ADC 周期打印一次）
         * ------------------------------------------------------------ */
        if(++log_tick >= 100)
        {
            LOGI("PID", "M=%.3f ap=%.3f as=%.3f th=%.3f err=%.1f fs=%.0f", M, alpha_p, alpha_s, theta, error_mA,
                 fs_output);
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
        .init_param = FM_PID_PARAM,
    };
    pid_new_control_block(&cfg, &pid_handle);
    pid_ctrl_queue_mA = xQueueCreate(6, sizeof(uint32_t));
    xTaskCreate(PID_ctrl_routine, "PID", 2048, NULL, 15, NULL);
}
