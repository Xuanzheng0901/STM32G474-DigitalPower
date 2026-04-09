#ifndef G474_1_KALMAN_H
#define G474_1_KALMAN_H

// ============ 1D 卡尔曼滤波器 ============
typedef struct {
    float x; // 系统的状态估计值 (如当前的电压预测值)
    float p; // 估计值的误差协方差
    float q; // 过程噪声协方差 (系统本身的波动)
    float r; // 测量噪声协方差 (ADC采样的噪声)
} kalman_1d_state_t;

void kalman_1d_init(kalman_1d_state_t *kf, float init_x, float init_p, float q, float r);

float kalman_1d_update(kalman_1d_state_t *kf, float measurement);

#endif //G474_1_KALMAN_H
