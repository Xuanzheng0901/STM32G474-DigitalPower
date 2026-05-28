# STM32G474 Power Electronics Projects

基于 STM32G474RET6 的电力电子数字电源项目集合，使用 HRTIM 高精度定时器实现多种电源拓扑的数字控制。

## 硬件平台

- **MCU**: STM32G474RET6 (170MHz Cortex-M4)
- **HRTIM**: 5.44GHz 高精度定时器，支持多路移相 PWM
- **ADC**: 4 路 ADC (ADC1~ADC4)，支持差分/单端采样，过采样
- **显示屏**: SH1107/SSD1306 OLED (SPI)，128x64
- **人机交互**: 旋转编码器 + 按键
- **通信**: UART3 (调试/控制台)
- **RTOS**: FreeRTOS
- **UI 框架**: LVGL v9.3.0

## 项目结构

```
.
├── Core/                   # STM32 HAL 驱动
├── Drivers/                # CMSIS & HAL 库
├── Middlewares/            # FreeRTOS
├── lvgl/                   # LVGL 图形库 (submodule)
├── User/
│   ├── components/
│   │   ├── ADC/            # ADC 多通道采样与数据处理
│   │   ├── console/        # UART 控制台
│   │   ├── display/        # OLED 驱动 (SSD1306/SH1107) + LVGL 移植
│   │   ├── filter/         # 卡尔曼滤波器
│   │   ├── LOG/            # 日志库
│   │   ├── pid_ctrl/       # PID 控制框架 (增量式/位置式)
│   │   └── UI/             # LVGL 界面 (模式切换、参数设定、数据监控)
│   └── main/
│       ├── main.c          # 主入口
│       ├── IO.c            # UART stdio 重定向
│       └── fusion_pixel_12.c  # 像素字体
├── cmake/                  # CMake STM32CubeMX 集成
└── CMakeLists.txt          # 顶层 CMake
```

## 分支与电源项目

> 每个分支对应一种电源拓扑/控制方案，可根据需求切换。

### `master` — 基础平台

项目基础设施分支，包含所有电源项目的公共代码：
- FreeRTOS 多任务框架
- OLED 显示屏驱动 (SSD1306/SH1107) + LVGL 图形界面
- 旋转编码器输入设备
- ADC 多通道采样 (单端/差分)
- 一维卡尔曼滤波器
- PID 控制算法库 (增量式/位置式)
- UART 控制台 + 日志系统

### `DAB-SPS` ★ (当前分支) — 双有源桥单移相控制

**Dual Active Bridge with Single Phase Shift**

基于串联谐振双有源桥 (SR-DAB) 的隔离型双向 DC-DC 变换器，采用单移相 (SPS) 控制策略。

**关键特性：**
- 固定开关频率 (85~160kHz 可变)，仅调节桥间移相角 θ
- 4 种工作模式：睡眠 / 正向充电 (1→2) / 反向放电 (2→1) / 自动双向
- 每个模式支持预充电/恒流子模式切换
- 增量式 PID 电流闭环控制 (500Hz 控制频率)
- 移相角限幅保护 (θ_max = 60°)
- OLED 实时显示：双端口电压/电流/功率、运行状态
- 编码器设定目标电流

**模式说明：**
| 模式 | 说明 | 功率流向 |
|------|------|----------|
| SLEEP | 关闭所有 PWM 输出 | — |
| 1→2 | 端口1 (10~15V) → 端口2 (3~4.2V) 充电 | 正向 |
| 2→1 | 端口2 (3~4.2V) → 端口1 (10~15V) 放电 | 反向 |
| AUTO | 自动维持端口1电压在 11~11.5V 范围内 | 双向 |

### `DAB` — 双有源桥三移相控制

**Dual Active Bridge with Triple Phase Shift**

在 SPS 基础上增加了桥内移相角 αp、αs，实现三自由度 (4-DOF) 的全范围软开关优化。

**与 DAB-SPS 的主要差异：**
- 三移相 (TPS) 控制：αp (初级内移相)、αs (次级内移相)、θ (桥间移相)
- 基于电压增益 M 的自适应 α 计算
- 双环 PID (电压外环 + 电流内环)
- PID 频率 500Hz

### `MPPT` — 最大功率点跟踪

**Maximum Power Point Tracking**

光伏/电池充电应用，实现太阳能电池板的最大功率点跟踪控制。

**关键特性：**
- MPPT 扰动观察法 (P&O)
- 电池电压采样与保护
- Buck-Boost 变换器拓扑
- ADC 采样触发时机优化 (避开 MOS 开关尖峰)
- 外部参考电压

### `PFC` — 功率因数校正

**Power Factor Correction**

单相 AC-DC 功率因数校正变换器。

**关键特性：**
- 交流电压/电流有效值采样与计算
- 电流内环 + 电压外环双环控制
- PFC 控制算法
- HRTIM 中断中翻转 GPIO 观测计算耗时
- ADC3/4 注入组同步采样

### `dcac` — DC-AC 逆变器

**DC to AC Inverter**

直流到交流逆变器，使用 SPWM 调制。

**关键特性：**
- SPWM 正弦脉宽调制
- 正弦表实时占空比计算
- HRTIM Master Repetition 中断更新 Timer 比较值
- AC 有效值采样算法
- 隔直与卡尔曼平滑

## 构建

```bash
# CMake 构建 (需要 ARM GCC 工具链)
cmake -B build -G Ninja
cmake --build build

# 或在 CLion 中直接打开项目
```

## 开发日志

在各分支的提交记录中可查阅完整开发历史。关键里程碑：

| 分支 | 关键节点 |
|------|----------|
| master | 基础平台搭建：OLED 驱动 → LVGL → FreeRTOS → PID 框架 → 日志库 |
| MPPT | MPPT 算法实现 → 控制环收敛 → 电池采样 |
| PFC | AC 采样 → SPWM → 电流内环 → PFC 控制 |
| dcac | SPWM 调制 → HRTIM 中断更新 → AC 有效值采样 |
| DAB | 4-DOF 移相角算法 → 三模式控制 → 双环 PID → 封箱 |
| DAB-SPS | SPS 简化 → PID 调优 → 模式3 完成 → UI 优化 |

---

*题目来源: B题 串联谐振有源双桥试验装置*
