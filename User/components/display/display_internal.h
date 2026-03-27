#ifndef G474_1_DISPLAY_INTERNAL_H
#define G474_1_DISPLAY_INTERNAL_H

#include "FreeRTOS.h"

typedef enum {
    display_COMMAND = 0,
    display_DATA,
} display_DC;

void display_tx_cmd(uint8_t cmd, const uint8_t *param, size_t param_size);

void display_tx_data(uint8_t *data, size_t size);

#endif //G474_1_DISPLAY_INTERNAL_H
