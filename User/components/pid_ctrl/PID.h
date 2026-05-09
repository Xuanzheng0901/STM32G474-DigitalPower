#ifndef G474_1_PID_H
#define G474_1_PID_H
#include <stdint.h>

typedef enum {
    MODE_SLEEP = 0,
    MODE_1TO2,
    MODE_2TO1,
    MODE_AUTO
} mode_t;

void pid_ctrl_init(void);

float get_pid_value(uint8_t index);

void pid_set_current(uint32_t mA);

#endif //G474_1_PID_H
