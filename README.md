# MPPT — 最大功率点跟踪光伏充电器

基于 STM32G474RET6 的光伏充电控制器，采用**扰动观察法 (Perturb & Observe, P&O)** 实现太阳能电池板的最大功率点跟踪。

## 控制架构

```
光伏板 ─→ ADC采样 (Vpv, Ipv) ─→ RC滤波 ─→ MPPT P&O算法 ─→ PWM占空比
电池  ─→ ADC采样 (Vbat, Ibat) ─→ 监控显示
```

- **拓扑**: Buck-Boost 变换器
- **MPPT 算法**: 扰动观察法 (P&O)，固定步长占空比扰动
- **采样**: 4 路 ADC（光伏电压/电流 + 电池电压/电流）
- **ADC 触发**: Timer E 的 Compare3 事件，避开 MOS 开关尖峰
- **滤波**: 二阶 IIR (RC) 低通滤波
- **死区**: 功率变化死区阈值 0.05W，避免噪声触发误反转
- **占空比范围**: 500 ~ 54000 (对应 0.9% ~ 99.2%)

## 显示界面

OLED 128x64 实时显示：
- 光伏电压 / 电流 / 功率
- 电池电压 / 电流
- 编码器调节占空比

## 硬件平台

- MCU: STM32G474RET6 (170MHz Cortex-M4)
- HRTIM: Timer E 输出 Buck-Boost PWM
- ADC1/2: 过采样 + 外部参考电压
- OLED: SSD1306 128x64 (SPI)
- 编码器: 硬件定时器扫描

## 项目结构

```
User/
├── components/
│   ├── ADC/          # ADC 多通道采样
│   ├── display/      # SSD1306 驱动 + LVGL 移植
│   ├── LOG/          # 日志库
│   ├── MPPT/         # MPPT P&O 算法
│   ├── stm_ssd1306/  # SSD1306 底层驱动
│   └── UI/           # LVGL 界面 (含动画)
└── main/
    └── main.c        # 入口
```

## 构建

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```
