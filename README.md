# DAB-SPS — 串联谐振双有源桥单移相控制

基于 STM32G474RET6 + HRTIM 的隔离型双向 DC-DC 变换器，采用**单移相 (Single Phase Shift, SPS)** 控制策略，支持 4 种工作模式下的恒流闭环控制。

## 系统概述

- **拓扑**: 串联谐振双有源桥 (SR-DAB)，变压器匝比 10:3
- **调制**: 所有桥臂 50% 固定占空比，仅调节桥间移相角 θ 控制功率
- **谐振频率**: 85kHz（硬件 LC 谐振）
- **开关频率**: 90kHz ~ 160kHz，随目标电流线性调频
- **控制频率**: 受 ADC DMA 半满/全满中断驱动，与开关频率同频
- **PID**: 增量式，PID 输出直接控制移相角 θ

## 硬件平台

| 组件 | 详情 |
|------|------|
| MCU | STM32G474RET6, 170MHz Cortex-M4 |
| HRTIM | 5 路 Timer (Master + A/B/C/D)，5.44GHz 时钟 |
| ADC | ADC1/2 双 ADC 同步 DMA，ADC1 差分采样电流，ADC2 单端采样电压 |
| 显示 | SH1107 OLED, 128x64, SPI |
| 输入 | 旋转编码器 (TIM2 硬件解码) + 按键 |
| 通信 | UART3 调试控制台 |
| RTOS | FreeRTOS |
| UI | LVGL v9.3.0 |

## HRTIM 移相控制

共 5 个 Timer 产生 8 路 PWM 驱动 4 个桥臂：

```
Timer A (CMP1=0)           ─── 初级侧 H 桥 上管
Timer C (CMP3)             ─── 初级侧 H 桥 下管，滞后 A 180°+αp
Timer B (CMP2)             ─── 次级侧 H 桥 上管，滞后 A 角度 θ
Timer D (CMP4)             ─── 次级侧 H 桥 下管，滞后 B 180°+αs
Master (CMP1~4)            ─── 控制四路 Timer 的同步复位点
```

**SPS 模式下 αp=αs=0**，仅通过调节 θ 改变传输功率：

- θ > 0：正向传输 (端口1 → 端口2)
- θ < 0：反向传输 (端口2 → 端口1)
- θ = 0：无功率传输

## 工作模式

| 模式 | 符号 | 功率方向 | 预充电阶段 | 恒流阶段 |
|------|------|----------|------------|----------|
| SLEEP | `-` | 关闭输出 | — | — |
| 1→2 | `→` | 端口1 → 端口2 | Vlow<3V: 500mA定频160kHz | 用户设定 (500~3000mA)，线性调频 |
| 2→1 | `←` | 端口2 → 端口1 | Vhigh<10V: 100mA定频135kHz | 用户设定 (100~1000mA)，线性调频 |
| AUTO | `↔` | 双向自动 | — | Vhigh<11V 反向充电 / Vhigh>11.5V 正向放电 |

**调频策略**: 目标电流越小 → 频率越高 → 功率越小，实现轻载效率优化。

## 控制流程

```
ADC半满/全满中断
  → xQueueSendFromISR(半块数据指针)
  → PID任务唤醒
    → adc_data_process()      # 4路卡尔曼滤波
    → 模式状态机               # 处理模式切换/预充电
    → 电压增益 M 计算          # M = n·Vlow/Vhigh
    → PID计算误差电流          # error_mA = target - actual
    → pid_compute()            # 增量式 PID → theta_output
    → HRTIM_Update_Timing()    # 写入5路Timer的Period/CMP
```

### 模式切换安全机制

- 切换模式时先进入 SLEEP（关闭所有 PWM 输出）
- 持续 50 个控制周期后正式切入目标模式
- 切换时重置 PID 积分器和移相角

## OLED 显示界面

```
┌─────────────────────┐
│          端口1 端口2│
│ 电压/V    xx.xx xx.xx│
│ 电流/A    xx.xx xx.xx│
│ 功率/W    xx.xx xx.xx│
│ 模式     -  →  ←  ↔ │  ← 4个按钮，编码器切换
│ 状态      预充电     │  ← 动态显示
│ 设定电流/A  0.0      │  ← 编码器调节 (0~3.1A)
└─────────────────────┘
```

5 种状态文本：`-` / `预充电` / `可调恒流` / `反向充电` / `正向放电`

## 项目结构

```
User/
├── components/
│   ├── ADC/            # ADC DMA 双缓冲采样 (ADC.c)
│   ├── console/        # UART 控制台 (console.c)
│   ├── display/        # SH1107 驱动 + LVGL 移植
│   ├── filter/         # 卡尔曼滤波 (kalman.c) + RC滤波 (RC.c)
│   ├── LOG/            # 分级日志库
│   ├── pid_ctrl/       # PID 算法库 + DAB-SPS 主控逻辑
│   │   ├── PID.h               # 公共接口
│   │   ├── pid_ctrl.c          # PID 核心算法 (增量式/位置式)
│   │   ├── pid_ctrl_internal.h # 内部头文件
│   │   └── pid_ctrl_main.c     # 控制主循环 (ADC处理+模式状态机+HRTIM更新)
│   └── UI/             # LVGL 界面 (UI.c)
└── main/
    ├── main.c          # 入口 (任务创建与硬件启动)
    ├── IO.c            # _write 重定向到 UART3
    └── fusion_pixel_12.c  # 12px 像素字体
```

## 构建

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| HRTIM_CLK_HZ | 5.44e9 | 170MHz × PLL×32 |
| FS_MIN | 95000 Hz | 满载最低频率 |
| FS_MAX | 135000 Hz | 轻载最高频率 |
| TRANSFORMER_N | 3.33333 | 变压器匝比 10:3 |
| THETA_MAX | ~60° | 移相角上限 |
| SWITCH_DELAY | 50 cycles | 模式切换安全延时 |
| PID 类型 | 增量式 | Kp=0.0015, Ki=0.00001, Kd=0.0001 |

---

*题目来源: B题 串联谐振有源双桥试验装置*
