#pragma once

#include <stdbool.h>

#include "common/time.h"

typedef enum {
    AUTONOMOUS_MODE_IDLE = 0,
    AUTONOMOUS_MODE_AUTHORIZED,
    AUTONOMOUS_MODE_RELEASE_WAITING,
    AUTONOMOUS_MODE_ARM_ACQUISITION,
    AUTONOMOUS_MODE_ARMED,
    AUTONOMOUS_MODE_HOVER_ENTRY,
    AUTONOMOUS_MODE_MANUAL_HANDOFF,
} autonomousModeState_e;

bool autonomousModeAuthorize(void);
bool autonomousModeRequestRelease(void);

void autonomousModeStartArmAcquisition(void);
void autonomousModeAbort(void);
void autonomousModeUpdate(timeUs_t currentTimeUs);

bool autonomousModeSuppressesPilotInput(void);
bool autonomousModeOwnsArming(void);
bool autonomousModeRequestsAngle(void);
bool autonomousModeRequestsAltitudeHold(void);
bool autonomousModeRequestsPositionHold(void);
bool autonomousModeSuppressesPilotHoldModes(void);
autonomousModeState_e autonomousModeGetState(void);
