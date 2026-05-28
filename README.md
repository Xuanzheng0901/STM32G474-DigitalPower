# dcac — DC-AC 逆变器

基于 STM32G474RET6 的单相 DC-AC 逆变器，采用 **SPWM (正弦脉宽调制)** 实现直流到交流的变换。

## 控制架构

```
DC输入 ─→ 全桥逆变 (Timer A/B SPWM) ─→ AC输出
              ↑                            ↓
      正弦表查表 (400点)          ADC RMS采样 (方差法)
              ↑                            ↓
      调制比 PID调节 ←── 卡尔曼滤波 ←── Vrms/Irms 计算
```

核心流程：
1. **SPWM 生成**：HRTIM Master Repetition 中断中，每个 PWM 周期从 400 点正弦表查表，更新 Timer A/B 的 CMP1/CMP3 寄存器
2. **调制比调节**：电压 PID 外环输出调制比 `factor`，通过 `set_mod_ratio_by_factor()` 预计算全表
3. **AC 采样**：使用方差法 (`SumSq - Sum²/N`) 直接计算 RMS 有效值，避免浮点逐点运算

## 技术特性

| 项目 | 说明 |
|------|------|
| **拓扑** | 全桥 DC-AC 逆变器 |
| **调制方式** | SPWM，400 点正弦表 |
| **载波频率** | PWM 频率 (HRTIM Timer A/B) |
| **输出频率** | 载波频率 / 400 |
| **控制环** | 电压 PID 外环调节调制比 |
| **AC 测量** | 方差法 RMS 有效值 (无直流分量影响) |
| **滤波** | 卡尔曼滤波平滑 RMS 结果 |

## SPWM 实现

- Timer A: 上桥臂 SPWM，占空比 = Period/2 ± duty_A
- Timer B: 下桥臂 SPWM，占空比 = Period/2 ± (Period - duty_A)，与 A 互补
- 调制比 factor ∈ [0, 0.96]，通过 PID 闭环自动调节

## 硬件平台

- MCU: STM32G474RET6 (170MHz Cortex-M4)
- HRTIM: Master Repetition 中断 + Timer A/B 输出 SPWM
- ADC: ADC1/2 DMA 双缓冲 AC 采样
- 调试: GPIO PC0/PC1 翻转观测时序

## 项目结构

```
User/
├── components/
│   ├── ADC/          # ADC AC 采样
│   ├── display/      # OLED 驱动 + LVGL
│   ├── filter/       # 卡尔曼滤波 (RMS后级平滑)
│   ├── LOG/          # 日志库
│   ├── pid_ctrl/     # PID 算法 + SPWM 控制逻辑
│   └── UI/           # LVGL 界面
└── main/             # 入口
```

## 构建

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```
