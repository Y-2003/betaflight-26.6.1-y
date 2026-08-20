#include "platform.h"
#ifdef SITL
#include <stdio.h>
#endif

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autonomous_mode.h"

#include "flight/imu.h"

static autonomousModeState_e autonomousModeState = AUTONOMOUS_MODE_IDLE;

void autonomousModeStartArmAcquisition(void)
{
    if (ARMING_FLAG(ARMED) || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        return;
    }

    autonomousModeState = AUTONOMOUS_MODE_ARM_ACQUISITION;

#ifdef SITL
    printf("[AUTO] ARM_ACQUISITION started\n");
#endif
}

void autonomousModeAbort(void)
{
    autonomousModeState = AUTONOMOUS_MODE_IDLE;
}

bool autonomousModeOwnsArming(void)
{
    return autonomousModeState != AUTONOMOUS_MODE_IDLE;
}

autonomousModeState_e autonomousModeGetState(void)
{
    return autonomousModeState;
}

void autonomousModeUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    #ifdef SITL
    static timeUs_t lastAttitudePrintUs = 0;

    if (currentTimeUs - lastAttitudePrintUs >= 1000000) {
        lastAttitudePrintUs = currentTimeUs;

    printf("[ATT] roll_ddeg=%d pitch_ddeg=%d upright=%d armed=%d auto_state=%d boxfailsafe=%d\n",
    attitude.values.roll,
    attitude.values.pitch,
    isUpright() ? 1 : 0,
    ARMING_FLAG(ARMED) ? 1 : 0,
    (int)autonomousModeState,
    IS_RC_MODE_ACTIVE(BOXFAILSAFE) ? 1 : 0);
    }
    #endif

    if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        autonomousModeAbort();
        return;
    }

     switch (autonomousModeState) {
    case AUTONOMOUS_MODE_IDLE:
        return;

    case AUTONOMOUS_MODE_ARM_ACQUISITION:
        if (ARMING_FLAG(ARMED)) {
            autonomousModeState = AUTONOMOUS_MODE_ARMED;
            return;
        }

        if (tryArmAutonomous()) {
            autonomousModeState = AUTONOMOUS_MODE_ARMED;

        #ifdef SITL
            printf("[AUTO] ARM success\n");
            printf("[AUTO] global ANGLE blocker: %s\n",
                (getArmingDisableFlags() & ARMING_DISABLED_ANGLE) ? "SET" : "CLEAR");
        #endif
        }
        return;

    case AUTONOMOUS_MODE_ARMED:
        if (!ARMING_FLAG(ARMED)) {
            autonomousModeState = AUTONOMOUS_MODE_IDLE;
        }
        return;
    }
}
