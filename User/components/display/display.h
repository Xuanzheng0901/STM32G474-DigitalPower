#ifndef G474_1_DISPLAY_H
#define G474_1_DISPLAY_H

#include "stdio.h"
#include "main.h"

#define DISPLAY_SH1107   0x01
#define DISPLAY_SSD1306  0x02

#define DISPLAY_USE DISPLAY_SH1107

#if DISPLAY_USE == DISPLAY_SSD1306
#include "ssd1306.h"
#define DISP_HOR_RES    128
#define DISP_VER_RES    64
#define DISP_BUF_SIZE   (DISP_HOR_RES * DISP_VER_RES / 8 + 8)
#define DISPLAY_DRAW(buffer) ssd1306_draw(buffer)
#elif DISPLAY_USE == DISPLAY_SH1107
#include "sh1107.h"
#define DISP_HOR_RES    128
#define DISP_VER_RES    128
#define DISP_BUF_SIZE   (DISP_HOR_RES * DISP_VER_RES / 8 + 8)
#define DISPLAY_DRAW(buffer) sh1107_draw(buffer)
#else
#error "Unsupported DISPLAY_USE value"
#endif

#define display_draw(buffer) DISPLAY_DRAW(buffer)

void display_init(void);

#endif //G474_1_DISPLAY_H
