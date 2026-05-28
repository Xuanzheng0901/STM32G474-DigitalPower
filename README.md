# Buck-Boost 数字电源平台

基于 STM32G474RET6 的通用数字电源基础平台，实现 Buck-Boost 变换器的 **CV/CC 双环 PID 控制**。作为后续各电源项目的公共基础设施。

## 控制架构

```
ADC采样 → 卡尔曼滤波 → 电压环 PID (外环)
                    → 电流环 PID (内环)
                    → CV/CC 比较取较小值 → PWM 占空比更新
```

- **拓扑**: Buck-Boost 变换器
- **控制**: 电压外环 + 电流内环，增量式 PID
- **CV/CC**: 比较电压环和电流环输出，取较小值实现自动恒压/恒流切换
- **PWM 输出**: HRTIM Timer E，频率由 CubeMX 配置
- **滤波器**: 一维卡尔曼滤波（电压/电流各一路）

## 公共基础设施

本分支提供了所有电源项目共用的基础框架：

| 模块 | 说明 |
|------|------|
| **FreeRTOS** | 多任务实时调度 |
| **LVGL v9.3.0** | 嵌入式图形界面框架 |
| **OLED 驱动** | SSD1306 / SH1107 (SPI)，128x64 |
| **ADC 采样** | ADC1~ADC4，DMA 双缓冲，差分/单端 |
| **卡尔曼滤波** | 一维，可调 Q/R 参数 |
| **PID 算法库** | 增量式/位置式，抗积分饱和，参数可动态更新 |
| **编码器驱动** | 硬件定时器扫描，LVGL 集成 |
| **日志库** | 分级日志 (ERROR/WARN/INFO/DEBUG) |
| **UART 控制台** | stdio 重定向，实时调参 |

## 硬件平台

- MCU: STM32G474RET6 (170MHz Cortex-M4, FPU)
- HRTIM: 5.44GHz 高精度定时器
- OLED: SPI 接口，SH1107/SSD1306
- 编码器: 旋转编码器 + 按键
- 调试串口: UART3

## 项目结构

```
User/
├── components/
│   ├── ADC/              # ADC 多通道 DMA 采样
│   ├── console/          # UART 控制台
│   ├── display/          # OLED 驱动 + LVGL 移植
│   ├── filter/           # 卡尔曼滤波
│   ├── LOG/              # 日志库
│   ├── pid_ctrl/         # PID 算法 + 双环控制逻辑
│   └── UI/               # LVGL 界面
└── main/
    ├── main.c            # 入口
    ├── IO.c              # UART stdio
    └── fusion_pixel_12.c # 像素字体
```

## 构建

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

## 后续分支

- [DAB](../DAB) — 双有源桥三移相控制
- [DAB-SPS](../DAB-SPS) — 双有源桥单移相控制
- [MPPT](../MPPT) — 最大功率点跟踪
- [PFC](../PFC) — 功率因数校正
- [dcac](../dcac) — DC-AC 逆变器
