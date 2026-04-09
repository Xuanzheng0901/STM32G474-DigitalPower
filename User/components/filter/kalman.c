#include "kalman.h"

// 初始化卡尔曼滤波器
void kalman_1d_init(kalman_1d_state_t *kf, float init_x, float init_p, float q, float r)
{
    kf->x = init_x;
    kf->p = init_p;
    kf->q = q;
    kf->r = r;
}

// 卡尔曼滤波更新函数
float kalman_1d_update(kalman_1d_state_t *kf, float measurement)
{
    // 1. 预测阶段 (Predict)
    // x_pre = kf->x (一维系统且无控制输入，假定系统状态不变)
    kf->p = kf->p + kf->q;
    // 2. 更新阶段 (Update)
    // 计算卡尔曼增益 K
    float k = kf->p / (kf->p + kf->r);

    // 计算最优估计值
    kf->x = kf->x + k * (measurement - kf->x);

    // 更新误差协方差
    kf->p = (1.0f - k) * kf->p;

    return kf->x;
}
