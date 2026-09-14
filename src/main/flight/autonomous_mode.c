#include "platform.h"
#ifdef SITL
#include <stdio.h>
#endif

#include <math.h>

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autonomous_mode.h"

#include "flight/imu.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define AUTONOMOUS_FREEFALL_ACCEL_THRESHOLD_G 0.30f
#define AUTONOMOUS_FREEFALL_CONFIRM_TIME_US 100000
#define AUTONOMOUS_HOVER_STABLE_TIME_US 500000
#define AUTONOMOUS_HOVER_STABLE_RATE_DPS 30

static autonomousModeState_e autonomousModeState = AUTONOMOUS_MODE_IDLE;

static bool autonomousFreefallCandidateActive = false;
static timeUs_t autonomousFreefallCandidateStartUs = 0;
static bool autonomousHoverStableCandidateActive = false;
static timeUs_t autonomousHoverStableStartUs = 0;

static void autonomousModeResetFreefallDetection(void)
{
    autonomousFreefallCandidateActive = false;
    autonomousFreefallCandidateStartUs = 0;
}

static void autonomousModeResetHoverStability(void)
{
    autonomousHoverStableCandidateActive = false;
    autonomousHoverStableStartUs = 0;
}

bool autonomousModeAuthorize(void)
{
    if (autonomousModeState != AUTONOMOUS_MODE_IDLE
        || ARMING_FLAG(ARMED)
        || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        return false;
    }

    autonomousModeState = AUTONOMOUS_MODE_AUTHORIZED;

#ifdef SITL
    printf("[AUTO] authorized\n");
#endif

    return true;
}

bool autonomousModeRequestRelease(void)
{
    if (autonomousModeState != AUTONOMOUS_MODE_AUTHORIZED
        || ARMING_FLAG(ARMED)
        || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        return false;
    }

    autonomousModeResetFreefallDetection();
    autonomousModeState = AUTONOMOUS_MODE_RELEASE_WAITING;

#ifdef SITL
    printf("[AUTO] release requested\n");
#endif

    return true;
}

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
    autonomousModeResetFreefallDetection();
    autonomousModeResetHoverStability();

    if (ARMING_FLAG(ARMED) && autonomousModeState != AUTONOMOUS_MODE_IDLE) {
        autonomousModeState = AUTONOMOUS_MODE_MANUAL_HANDOFF;
    } else {
        autonomousModeState = AUTONOMOUS_MODE_IDLE;
    }
}

bool autonomousModeOwnsArming(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ARM_ACQUISITION
        || autonomousModeState == AUTONOMOUS_MODE_ARMED
        || autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_MANUAL_HANDOFF;
}

bool autonomousModeSuppressesPilotInput(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_AUTHORIZED
        || autonomousModeState == AUTONOMOUS_MODE_RELEASE_WAITING
        || autonomousModeState == AUTONOMOUS_MODE_ARM_ACQUISITION
        || autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

bool autonomousModeRequestsAngle(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED
        || autonomousModeState == AUTONOMOUS_MODE_MANUAL_HANDOFF;
}

bool autonomousModeRequestsAltitudeHold(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

bool autonomousModeRequestsPositionHold(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

bool autonomousModeSuppressesPilotHoldModes(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED
        || autonomousModeState == AUTONOMOUS_MODE_MANUAL_HANDOFF;
}

autonomousModeState_e autonomousModeGetState(void)
{
    return autonomousModeState;
}

void autonomousModeUpdate(timeUs_t currentTimeUs)
{
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

    printf("[ACC] magnitude_mg=%ld updated=%d adc=(%ld,%ld,%ld)\n",
    lrintf(acc.accMagnitude * 1000.0f),
    acc.isAccelUpdatedAtLeastOnce ? 1 : 0,
    lrintf(acc.accADC.x),
    lrintf(acc.accADC.y),
    lrintf(acc.accADC.z));



    }
    #endif

    if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        autonomousModeAbort();
        return;
    }

     switch (autonomousModeState) {
    case AUTONOMOUS_MODE_IDLE:
        return;

    case AUTONOMOUS_MODE_AUTHORIZED:
        return;

    case AUTONOMOUS_MODE_RELEASE_WAITING:
    if (!acc.isAccelUpdatedAtLeastOnce
        || acc.accMagnitude >= AUTONOMOUS_FREEFALL_ACCEL_THRESHOLD_G) {
        autonomousModeResetFreefallDetection();
        return;
    }

    if (!autonomousFreefallCandidateActive) {
        autonomousFreefallCandidateActive = true;
        autonomousFreefallCandidateStartUs = currentTimeUs;
        return;
    }

    if (cmpTimeUs(currentTimeUs, autonomousFreefallCandidateStartUs)
        >= AUTONOMOUS_FREEFALL_CONFIRM_TIME_US) {
#ifdef SITL
        printf("[AUTO] freefall confirmed magnitude_mg=%ld\n",
            lrintf(acc.accMagnitude * 1000.0f));
#endif
        autonomousModeResetFreefallDetection();
        autonomousModeStartArmAcquisition();
    }
    return;

    case AUTONOMOUS_MODE_ARM_ACQUISITION:
        if (ARMING_FLAG(ARMED)) {
            autonomousModeResetHoverStability();
            autonomousModeState = AUTONOMOUS_MODE_HOVER_ENTRY;
            return;
        }

        if (tryArmAutonomous()) {
            autonomousModeResetHoverStability();
            autonomousModeState = AUTONOMOUS_MODE_HOVER_ENTRY;

        #ifdef SITL
            printf("[AUTO] ARM success\n");
            printf("[AUTO] global ANGLE blocker: %s\n",
                (getArmingDisableFlags() & ARMING_DISABLED_ANGLE) ? "SET" : "CLEAR");
        #endif
        }
        return;

    case AUTONOMOUS_MODE_ARMED:
    case AUTONOMOUS_MODE_HOVER_ENTRY:
        if (!ARMING_FLAG(ARMED)) {
            autonomousModeResetHoverStability();
            autonomousModeState = AUTONOMOUS_MODE_IDLE;
            return;
        }

        const bool hoverStable = gyroAbsRateDps(FD_ROLL) <= AUTONOMOUS_HOVER_STABLE_RATE_DPS
            && gyroAbsRateDps(FD_PITCH) <= AUTONOMOUS_HOVER_STABLE_RATE_DPS
            && gyroAbsRateDps(FD_YAW) <= AUTONOMOUS_HOVER_STABLE_RATE_DPS;
        if (!hoverStable) {
            autonomousModeResetHoverStability();
            return;
        }

        if (!autonomousHoverStableCandidateActive) {
            autonomousHoverStableCandidateActive = true;
            autonomousHoverStableStartUs = currentTimeUs;
            return;
        }

        if (cmpTimeUs(currentTimeUs, autonomousHoverStableStartUs) >= AUTONOMOUS_HOVER_STABLE_TIME_US) {
            autonomousModeResetHoverStability();
            autonomousModeState = AUTONOMOUS_MODE_ARMED;
        }
        return;

    case AUTONOMOUS_MODE_MANUAL_HANDOFF:
        if (!ARMING_FLAG(ARMED)) {
            autonomousModeState = AUTONOMOUS_MODE_IDLE;
        }
        return;
    }
}
