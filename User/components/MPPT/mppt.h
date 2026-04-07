#ifndef G474_1_MPPT_H
#define G474_1_MPPT_H

#include <stdint.h>

typedef enum {
    POWER_V,
    POWER_I,
    BAT_V,
    BAT_I
} voltage_data_t;

float get_voltage_value(voltage_data_t data);

void pwm_set_duty(uint32_t pwm_duty);

void MPPT_init(void);

#endif //G474_1_MPPT_H
