#pragma once

#ifndef G474_1_MPPT_INTERNAL_H
#define G474_1_MPPT_INTERNAL_H

#include "stdio.h"
#include "stm32g4xx_hal.h"

typedef enum {
    TIMER_A = 0,
    TIMER_B,
    TIMER_C,
    TIMER_D,
    TIMER_E,
    TIMER_F,
} hrtim_timer_t;


#endif //G474_1_MPPT_INTERNAL_H
