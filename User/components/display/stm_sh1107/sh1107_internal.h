#ifndef G474_1_SH1107_INTERNAL_H
#define G474_1_SH1107_INTERNAL_H

// 1. 基础显示控制
#define SH1107_CMD_DISP_OFF                  0xAE // 关闭显示 (进入睡眠模式 Sleep Mode)
#define SH1107_CMD_DISP_ON                   0xAF // 开启显示 (正常运行 Normal Operation)
#define SH1107_CMD_ENTIRE_DISP_OFF           0xA4 // 恢复 RAM 内容显示 (默认正常状态)
#define SH1107_CMD_ENTIRE_DISP_ON            0xA5 // 强制全屏点亮 (忽略 RAM 内容，常用于测试)
#define SH1107_CMD_SET_DISPLAY_INVERT_OFF    0xA6 // 正常显示 (RAM: 0=灭, 1=亮)
#define SH1107_CMD_SET_DISPLAY_INVERT_ON     0xA7 // 反色显示 (RAM: 0=亮, 1=灭)
#define SH1107_CMD_SET_DISPLAY_CONTRAST      0x81 // 设置对比度/亮度 (双字节指令: 0x81, 参数0x00~0xFF)

// 2. 寻址与位置设置
#define SH1107_CMD_SET_MEMORY_ADDR_MODE      0x20 // 设置寻址模式 (0x20: 页寻址模式[默认], 0x21: 垂直寻址模式)
#define SH1107_CMD_SET_PAGE_ADDR             0xB0 // 设置页地址基址 (0xB0 ~ 0xBF, 对应 Page 0 ~ 15)
#define SH1107_CMD_SET_COL_LOWER             0x00 // 设置列地址低 4 位 (0x00 ~ 0x0F)
#define SH1107_CMD_SET_COL_HIGHER            0x10 // 设置列地址高 4 位 (0x10 ~ 0x17)
#define SH1107_CMD_SET_DISPLAY_OFFSET        0xD3 // 设置显示垂直偏移 (双字节指令: 0xD3, 参数0x00~0x7F)
#define SH1107_CMD_SET_DISPLAY_START_LINE    0xDC // 设置显示起始行 (双字节指令: 0xDC, 参数0x00~0x7F) ★注意：SSD1306是0x40

// 3. 硬件与扫描方向配置
#define SH1107_CMD_SET_MULTIPLEX             0xA8 // 设置多路复用率/驱动路数 (双字节指令: 0xA8, 参数0x00~0x7F，默认0x7F即128路)
#define SH1107_CMD_SET_SEGMENT_REMAP_NORMAL  0xA0 // 设置列扫描方向: 正常 (SEG0 映射到 RAM 列 0)
#define SH1107_CMD_SET_SEGMENT_REMAP_REVERSE 0xA1 // 设置列扫描方向: 左右反转 (SEG0 映射到 RAM 列 127)
#define SH1107_CMD_SET_COM_SCAN_NORMAL       0xC0 // 设置行扫描方向: 正常 (从 COM0 到 COM[N-1])
#define SH1107_CMD_SET_COM_SCAN_REVERSE      0xC8 // 设置行扫描方向: 上下反转 (从 COM[N-1] 到 COM0)

// 4. 时序与驱动电压配置
#define SH1107_CMD_SET_CLOCK_DIV             0xD5 // 设置显示时钟分频因子/振荡频率 (双字节指令: 0xD5, 参数0x00~0xFF)
#define SH1107_CMD_SET_PRECHARGE_PERIOD      0xD9 // 设置预充电周期 (双字节指令: 0xD9, 参数0x00~0xFF)
#define SH1107_CMD_SET_VCOM_LEVEL            0xDB // 设置 VCOMH 释放电平 (双字节指令: 0xDB, 参数0x00~0x3F)
#define SH1107_CMD_SET_COMPINS               0xDA // 设置 COM 硬件引脚配置 (双字节指令: 0xDA, 常见参数0x12)

// 5. 电源控制
#define SH1107_CMD_DC_DC_CONTROL             0xAD // DC-DC 升压开关控制 (双字节指令: 0xAD, 参数0x8A或0x8B)
#define SH1107_DC_DC_ON                      0x8B // 参数: 开启内部 DC-DC 升压 (常与 0xAD 配合使用)
#define SH1107_DC_DC_OFF                     0x8A // 参数: 关闭内部 DC-DC 升压 (使用外部 VPP 时)

#define SH1107_WIDTH                         128
#define SH1107_HEIGHT                        128
#define SH1107_FULL_FLUSH_BUFFER_SIZE        (SH1107_WIDTH * SH1107_HEIGHT / 8)

#endif //G474_1_SH1107_INTERNAL_H
