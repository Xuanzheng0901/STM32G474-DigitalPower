# PFC — 功率因数校正变换器

基于 STM32G474RET6 的单相 Boost PFC（功率因数校正）数字电源，实现 **平均电流模式控制**。

## 控制架构

```
                 ┌─ 电压外环 (FreeRTOS Task, ~kHz) ─┐
AC输入 ─→ ADC3注入组 ─→ vac_shape (馒头波归一化)       │
                 │                          × ─→ target_il ─→ 电流内环 ─→ PWM
Bus电压 ←─ ADC1 DMA ←─ 卡尔曼滤波 ←────────┘          │
                 └─ 电流内环 (HRTIM Repetition ISR) ───┘
```

核心思想：
1. **电压外环**：通过 PID 调节母线电压，输出电流幅值指令 `Global_Current_Ref_Amplitude`
2. **电流内环**：在 HRTIM 每个 PWM 周期中断中，将电网电压形状 (馒头波) × 电流幅值指令 → 目标瞬时电流，再做电流闭环

## 技术特性

| 项目 | 说明 |
|------|------|
| **拓扑** | Boost PFC |
| **控制策略** | 平均电流模式 (Average Current Mode) |
| **电流内环** | HRTIM Master Repetition 中断，频率 = PWM 频率 |
| **电压外环** | FreeRTOS 任务，频率取决于 ADC DMA 周期 |
| **AC 采样** | ADC3/4 注入组同步采样 (HRTIM 触发) |
| **DC 采样** | ADC1/2 DMA 双缓冲 + 卡尔曼滤波 |
| **调试** | GPIO PA2/PC1 翻转观测计算耗时 |

## 硬件平台

- MCU: STM32G474RET6 (170MHz Cortex-M4)
- HRTIM: Master Repetition 中断驱动电流环，Timer A 输出 PWM
- ADC: ADC1/2 DMA + ADC3/4 注入组
- OLED: SSD1306/SH1107 (SPI)
- 编码器: 旋转编码器
- RTOS: FreeRTOS + LVGL

## 项目结构

```
User/
├── components/
│   ├── ADC/          # ADC 采样 (DMA + 注入组)
│   ├── display/      # OLED 驱动 + LVGL
│   ├── filter/       # 卡尔曼滤波
│   ├── LOG/          # 日志库
│   ├── pid_ctrl/     # 双环 PID + PFC 控制逻辑
│   └── UI/           # LVGL 界面
└── main/             # 入口
```

## 构建

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```
