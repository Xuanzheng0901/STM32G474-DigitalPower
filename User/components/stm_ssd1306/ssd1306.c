#include <stdint.h>
#include "ssd1306.h"
#include "ssd1306_internal.h"

void ssd1306_init(void)
{
    // 1. 硬件复位
    ssd1306_reset();

    // 2. 关闭屏幕（初始化期间）
    ssd1306_tx_cmd(SSD1306_CMD_DISP_OFF, NULL, 0);

    // 3. 基础配置
    // 设置寻址模式为“页寻址模式”，以匹配 LVGL 缓冲区的转换逻辑
    ssd1306_tx_cmd(SSD1306_CMD_SET_MEMORY_ADDR_MODE, (uint8_t[]){0x00}, 1); //Page Addressing Mode

    // 设置屏幕尺寸和 COM 引脚配置
    ssd1306_tx_cmd(SSD1306_CMD_SET_MULTIPLEX, (uint8_t[]){SSD1306_HEIGHT - 1}, 1);
    ssd1306_tx_cmd(SSD1306_CMD_SET_COMPINS, (uint8_t[]){0x12}, 1); // 适用于 64 高度

    // 4. 显示方向/镜像设置 (通常需要开启以获得正确的显示方向)
    ssd1306_tx_cmd(SSD1306_CMD_MIRROR_X_ON, NULL, 0); // 列地址 127 映射到 0
    ssd1306_tx_cmd(SSD1306_CMD_MIRROR_Y_ON, NULL, 0); // 从 COM[N-1] 到 0

    // 5. 时序和驱动电压设置 (关键配置)
    ssd1306_tx_cmd(SSD1306_CMD_SET_DISP_CLOCK_DIV, (uint8_t[]){0x80},
                   1); // 设置时钟分频/振荡器频率
    ssd1306_tx_cmd(SSD1306_CMD_SET_PRECHARGE, (uint8_t[]){0xF1}, 1); // 设置预充电周期
    ssd1306_tx_cmd(SSD1306_CMD_SET_VCOM_DESELECT, (uint8_t[]){0x30}, 1); // 设置 VCOMH

    // 6. 启用电荷泵 (内部升压电路)
    ssd1306_tx_cmd(SSD1306_CMD_SET_CHARGE_PUMP, (uint8_t[]){0x14}, 1);

    // 7. 设置显示模式
    ssd1306_tx_cmd(SSD1306_CMD_ENTIRE_DISP_OFF, NULL, 0); // 恢复 GDDRAM 显示
    ssd1306_tx_cmd(SSD1306_CMD_INVERT_ON, NULL, 0); // 设置为反色显示(0为点亮)

    // 8. 清屏
    // uint8_t clear_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8] = {0};
    // ssd1306_draw(clear_buffer);

    // 9. 打开屏幕
    ssd1306_tx_cmd(SSD1306_CMD_DISP_ON, NULL, 0);
}

void ssd1306_draw(uint8_t *color_data)
{
    // 1. 设置列地址范围 (从第 0 列到第 127 列)
    ssd1306_tx_cmd(SSD1306_CMD_SET_COLUMN_RANGE, (uint8_t[]){0, SSD1306_WIDTH - 1}, 2);

    // 2. 设置页地址范围 (从第 0 页到第 7 页)
    ssd1306_tx_cmd(SSD1306_CMD_SET_PAGE_RANGE, (uint8_t[]){0, (SSD1306_HEIGHT / 8) - 1}, 2);

    // 3. 通过 SPI 发送整个缓冲区的数据
    // 在此操作之前，DC 引脚必须被设置为 DATA 模式
    ssd1306_tx_data(color_data, FULL_FLUSH_BUFFER_SIZE);
}
