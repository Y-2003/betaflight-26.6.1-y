#pragma once

#include <stdbool.h>

#include "common/time.h"

typedef enum {
    AUTONOMOUS_MODE_IDLE = 0,
    AUTONOMOUS_MODE_ARM_ACQUISITION,
    AUTONOMOUS_MODE_ARMED,
} autonomousModeState_e;

void autonomousModeStartArmAcquisition(void);
void autonomousModeAbort(void);
void autonomousModeUpdate(timeUs_t currentTimeUs);

bool autonomousModeOwnsArming(void);
autonomousModeState_e autonomousModeGetState(void);
