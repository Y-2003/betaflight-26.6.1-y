#include "platform.h"
#ifdef SITL
#include <stdio.h>
#endif

#include <math.h>

#include "build/debug.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "rx/rx.h"

#include "flight/autonomous_mode.h"

#include "flight/autopilot_multirotor.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define AUTONOMOUS_FREEFALL_ACCEL_THRESHOLD_G 0.30f // 自由落体确认阈值，单位：G
#define AUTONOMOUS_FREEFALL_CONFIRM_TIME_US 100000 // 自由落体持续确认时间，单位：us
#define AUTONOMOUS_HOVER_STABLE_TIME_US 500000 // 悬停稳定持续时间，单位：us
#define AUTONOMOUS_HOVER_STABLE_RATE_DPS 30 // 悬停稳定角速度阈值，单位：dps
#define AUTONOMOUS_HOVER_LEVEL_ANGLE_DEG 5.0f // 悬停判定最大横滚/俯仰角，单位：deg
#define AUTONOMOUS_HOVER_ABORT_ANGLE_DEG 65.0f // 已稳定悬停后重新进入调平的倾角，单位：deg
#define AUTONOMOUS_USER1_POSITION_CHANGE_CONFIRM_US 30000 // USER1 拨杆位置变化确认时间，单位：us
#define AUTONOMOUS_USER2_ARM_DELAY_US 2000000 // USER2 触发后进入自主解锁检测前的延时，单位：us
#define AUTONOMOUS_FLIP_THROTTLE_RATIO 0.50f // 翻身油门初始值使用悬停油门余量的比例
#define AUTONOMOUS_ENTRY_THROTTLE_RATIO 0.50f // 入口时使用悬停油门余量的比例（相对 PWM 1000）
#define AUTONOMOUS_FLIP_THROTTLE_MIN_RATIO 0.30f // 翻身闭环油门下限使用悬停油门余量的比例
#define AUTONOMOUS_FLIP_THROTTLE_MAX_RATIO 0.80f // 翻身闭环油门上限使用悬停油门余量的比例
#define AUTONOMOUS_FLIP_RATE_DPS 400.0f // 翻身目标角速度，单位：dps
#define AUTONOMOUS_FLIP_RATE_KP_PWM_PER_DPS 0.15f // 角速度误差到油门修正的比例增益
#define AUTONOMOUS_FLIP_THROTTLE_SLEW_PWM_PER_S 600.0f // 翻身油门最大变化速度，单位：PWM/s
#define AUTONOMOUS_FLIP_START_COS_TILT 0.20f // 进入/结束翻身的 cos 倾角阈值
#define AUTONOMOUS_FLIP_END_COS_TILT 0.90f // 结束翻身时要求倾角小于约26度
#define AUTONOMOUS_FLIP_TIMEOUT_US 1000000 // 翻身最大允许时间，单位：us

static autonomousModeState_e autonomousModeState = AUTONOMOUS_MODE_IDLE;

static bool autonomousUser2SeenLow = false;
static bool autonomousUser2WasActive = false;
static bool autonomousArmWasActive = false;
static bool autonomousUser2CycleRequested = false;
static bool autonomousUser2ArmDelayStarted = false;
static timeUs_t autonomousUser2ArmDelayStartUs = 0;
static autonomousUser1Position_e autonomousUser1InitialPosition = AUTONOMOUS_USER1_POSITION_INVALID;
static autonomousUser1Position_e autonomousUser1PendingPosition = AUTONOMOUS_USER1_POSITION_INVALID;
static timeUs_t autonomousUser1PendingSinceUs = 0;

static bool autonomousFreefallCandidateActive = false;
static timeUs_t autonomousFreefallCandidateStartUs = 0;
static bool autonomousHoverStableCandidateActive = false;
static timeUs_t autonomousHoverStableStartUs = 0;
static flight_dynamics_index_t autonomousFlipAxis = FD_ROLL;
static float autonomousFlipDirection = 1.0f;
static float autonomousFlipRateDps = 0.0f;
static uint16_t autonomousFlipThrottlePwm = 0;
static timeUs_t autonomousFlipStartUs = 0;
static timeUs_t autonomousFlipLastUpdateUs = 0;

static void autonomousModeStartFlip(timeUs_t currentTimeUs);
static void autonomousModeUpdateFlipControl(timeUs_t currentTimeUs);
static void autonomousModeResetFlip(void);

static uint16_t autonomousModeThrottleFromHoverRatio(const float ratio)
{
    const float hoverThrottle = (float)autopilotGetEffectiveHoverThrottlePwm();
    return (uint16_t)lrintf(constrainf(
        (float)PWM_RANGE_MIN
            + (hoverThrottle - (float)PWM_RANGE_MIN) * ratio,
        (float)PWM_RANGE_MIN,
        (float)PWM_RANGE_MAX));
}

// [新增] 功能：清除自由落体候选检测状态。
static void autonomousModeResetFreefallDetection(void)
{
    autonomousFreefallCandidateActive = false;
    autonomousFreefallCandidateStartUs = 0;
}

// [新增] 功能：清除悬停稳定候选检测状态。
static void autonomousModeResetHoverStability(void)
{
    autonomousHoverStableCandidateActive = false;
    autonomousHoverStableStartUs = 0;
}

// [新增] 功能：通过 BOXUSER1 配置取得 AUX 通道并读取三段拨杆位置。
// USER1 仅用于定位原始 RC 通道；LOW/MIDDLE/HIGH 不会直接启用任何飞行模式。
static autonomousUser1Position_e autonomousModeReadUser1Position(void)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->modeId != BOXUSER1 || !IS_RANGE_USABLE(&mac->range)) {
            continue;
        }

        const uint16_t value = constrain(rcData[mac->auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
        if (value >= 1300 && value <= 1700) {
            return AUTONOMOUS_USER1_POSITION_MIDDLE;
        }
        if (value >= 1800) {
            return AUTONOMOUS_USER1_POSITION_HIGH;
        }
        if (value < 1300) {
            return AUTONOMOUS_USER1_POSITION_LOW;
        }
        return AUTONOMOUS_USER1_POSITION_INVALID;
    }

    return AUTONOMOUS_USER1_POSITION_INVALID;
}

// [新增] 功能：复位自主状态机及其检测、翻转和 USER1 拨杆位置状态。
static void autonomousModeResetToIdle(void)
{
    // The active autonomous trigger owns the synthetic lock duration. An
    // abort must not release it while USER2 or USER3 is still high; the
    // corresponding falling level below is the sole release.
    if (!IS_RC_MODE_ACTIVE(BOXUSER2) && !IS_RC_MODE_ACTIVE(BOXUSER3)) {
        failsafeRequestAutonomousLock(false);
    }
    autonomousModeResetFreefallDetection();
    autonomousModeResetHoverStability();
    autonomousModeResetFlip();
    autonomousModeState = AUTONOMOUS_MODE_IDLE;
    autonomousUser2CycleRequested = false;
    autonomousUser2ArmDelayStarted = false;
    autonomousUser2ArmDelayStartUs = 0;
    autonomousUser1InitialPosition = AUTONOMOUS_USER1_POSITION_INVALID;
    autonomousUser1PendingPosition = AUTONOMOUS_USER1_POSITION_INVALID;
    autonomousUser1PendingSinceUs = 0;
}

// [新增] 功能：进入自由落体检测阶段，开始等待自主解锁条件。
void autonomousModeStartAccelerationDetection(void)
{
    if (ARMING_FLAG(ARMED)
        || !autonomousUser2CycleRequested
        || failsafeIsActive()
        || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        return;
    }

    autonomousModeResetFreefallDetection();
    autonomousModeResetHoverStability();
    autonomousModeState = AUTONOMOUS_MODE_ACCEL_WAITING;

#ifdef SITL
    printf("[AUTO] acceleration detection started\n");
#endif
}

// [新增] 功能：中止当前自主流程并恢复空闲状态。
void autonomousModeAbort(void)
{
    autonomousModeResetToIdle();
}

// [新增] 功能：判断 USER2 自主流程是否暂时接管解锁控制。
bool autonomousModeOwnsArming(void)
{
    return autonomousUser2CycleRequested && autonomousModeState != AUTONOMOUS_MODE_IDLE;
}

// [新增] 功能：判断当前状态是否屏蔽飞手姿态输入。
bool autonomousModeSuppressesPilotInput(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ACCEL_WAITING
        || autonomousModeState == AUTONOMOUS_MODE_FLIP
        || autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

// [新增] 功能：判断当前状态是否用自主悬停油门覆盖飞手油门。
bool autonomousModeOverridesThrottle(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ACCEL_WAITING
        || autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

// 返回自主流程当前阶段使用的集体油门。调平入口阶段使用悬停油门的固定比例，
// 避免因不同 ap_hover_throttle 配置导致入口推力失配。
uint16_t autonomousModeGetThrottlePwm(void)
{
    if (autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY) {
        return autonomousModeThrottleFromHoverRatio(AUTONOMOUS_ENTRY_THROTTLE_RATIO);
    }

    return autopilotGetEffectiveHoverThrottlePwm();
}

// [新增] 功能：判断翻转控制是否正在执行。
bool autonomousModeIsFlipActive(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_FLIP;
}

// [新增] 功能：返回自主翻转阶段指定轴的角速度设定值。
float autonomousModeGetFlipRate(int axis)
{
    if (!autonomousModeIsFlipActive() || axis != (int)autonomousFlipAxis) {
        return 0.0f;
    }

    return autonomousFlipDirection * autonomousFlipRateDps;
}

// [新增] 功能：返回自主翻转阶段的油门 PWM 输出。
uint16_t autonomousModeGetFlipThrottlePwm(void)
{
    return autonomousFlipThrottlePwm;
}

// [新增] 功能：判断自主状态机是否请求 ANGLE 模式。
bool autonomousModeRequestsAngle(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ACCEL_WAITING
        || autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

// [新增] 功能：判断自主状态机是否请求高度保持。
bool autonomousModeRequestsAltitudeHold(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

// [新增] 功能：判断自主状态机是否请求位置保持。
bool autonomousModeRequestsPositionHold(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

// [新增] 功能：判断是否暂时屏蔽飞手的高度/位置保持模式请求。
bool autonomousModeSuppressesPilotHoldModes(void)
{
    return autonomousModeState == AUTONOMOUS_MODE_ACCEL_WAITING
        || autonomousModeState == AUTONOMOUS_MODE_FLIP
        || autonomousModeState == AUTONOMOUS_MODE_HOVER_ENTRY
        || autonomousModeState == AUTONOMOUS_MODE_ARMED;
}

// [新增] 功能：返回当前自主状态机状态。
autonomousModeState_e autonomousModeGetState(void)
{
    return autonomousModeState;
}

// [新增] 功能：返回当前 USER1 拨杆位置。
autonomousUser1Position_e autonomousModeGetUser1Position(void)
{
    return autonomousModeReadUser1Position();
}

// [兼容] USER1 仅作为拨杆位置变化/退出输入，不直接请求高度保持。
bool autonomousModeUserRequestsAltitudeHold(void)
{
    return false;
}

// [兼容] USER1 仅作为拨杆位置变化/退出输入，不直接请求位置保持。
bool autonomousModeUserRequestsPositionHold(void)
{
    return false;
}

// [新增] 功能：按周期推进 USER2 自主解锁、自由落体、翻转和悬停状态机。
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

    const bool user2Active = IS_RC_MODE_ACTIVE(BOXUSER2);
    const bool user3Active = IS_RC_MODE_ACTIVE(BOXUSER3);
    const bool armActive = IS_RC_MODE_ACTIVE(BOXARM);
    const bool user2WasSeenLow = autonomousUser2SeenLow;
    const bool user2RisingEdge = user2Active && !autonomousUser2WasActive && user2WasSeenLow;
    // USER2 owns its configured trigger semantics. USER3 is an alternate
    // throw-mode trigger and must not reinterpret a simultaneous USER2 cycle
    // (otherwise USER1 validation is bypassed and the two workflows interfere).
    const bool user3ArmRisingEdge = user3Active && !user2Active && armActive && !autonomousArmWasActive;
    if (!user2Active) {
        autonomousUser2SeenLow = true;
    }
    autonomousUser2WasActive = user2Active;
    autonomousArmWasActive = armActive;

    if (autonomousUser2CycleRequested
        && autonomousModeState != AUTONOMOUS_MODE_IDLE
        && !IS_RC_MODE_ACTIVE(BOXARM)) {
        autonomousModeAbort();
        return;
    }

    if ((user2RisingEdge || user3ArmRisingEdge) && !autonomousUser2CycleRequested
        && autonomousModeState == AUTONOMOUS_MODE_IDLE) {
        const autonomousUser1Position_e user1Position = autonomousModeReadUser1Position();
        // Both USER2 and USER3 use USER1 as the exit/hand-off switch. Keep
        // the trigger invalid until USER1 has a readable initial position so
        // both entry paths have identical exit semantics.
        if (armActive
            && user1Position != AUTONOMOUS_USER1_POSITION_INVALID) {
            failsafeRequestAutonomousLock(true);
            autonomousUser2CycleRequested = true;
            // Consume the low-to-high trigger. A later autonomous cycle must
            // observe a fresh low level before another high level is valid.
            autonomousUser2SeenLow = false;
            autonomousUser2ArmDelayStarted = false;
            autonomousUser2ArmDelayStartUs = 0;
            autonomousUser1InitialPosition = user1Position;
            autonomousUser1PendingPosition = AUTONOMOUS_USER1_POSITION_INVALID;
            autonomousUser1PendingSinceUs = 0;
            autonomousModeState = AUTONOMOUS_MODE_WAITING_FOR_ARM;
        }
    }

    // USER2 is a one-shot trigger and keeps the synthetic RX lock until its
    // level is released. USER3 is a level-held throw enable whose ARM rising
    // edge starts the cycle; release the synthetic lock after that edge so
    // valid RX data can recover and the waiting state can proceed. Keeping
    // USER3 locked here would make failsafeIsReceivingRxData() stay false
    // forever while USER3 remains high.
    if (!user2Active) {
        failsafeRequestAutonomousLock(false);
    }

    DEBUG_SET(DEBUG_FAILSAFE, 4, autonomousModeState);
    DEBUG_SET(DEBUG_FAILSAFE, 5, user2Active);

    if (!ARMING_FLAG(ARMED)
        && autonomousModeState != AUTONOMOUS_MODE_WAITING_FOR_ARM
        && autonomousModeState != AUTONOMOUS_MODE_ACCEL_WAITING
        && autonomousModeState != AUTONOMOUS_MODE_IDLE) {
        autonomousModeResetToIdle();
    }

    if (autonomousUser2CycleRequested
        && autonomousModeState != AUTONOMOUS_MODE_IDLE
        && autonomousUser1InitialPosition != AUTONOMOUS_USER1_POSITION_INVALID) {
        const autonomousUser1Position_e user1Position = autonomousModeReadUser1Position();
        if (user1Position != autonomousUser1InitialPosition) {
            if (user1Position != autonomousUser1PendingPosition) {
                autonomousUser1PendingPosition = user1Position;
                autonomousUser1PendingSinceUs = currentTimeUs;
            } else if (cmpTimeUs(currentTimeUs, autonomousUser1PendingSinceUs)
                >= AUTONOMOUS_USER1_POSITION_CHANGE_CONFIRM_US) {
                autonomousModeAbort();
                return;
            }
        } else {
            autonomousUser1PendingPosition = AUTONOMOUS_USER1_POSITION_INVALID;
            autonomousUser1PendingSinceUs = 0;
        }
    }

    // Failsafe owns the aircraft. Preserve a USER2 cycle while waiting for
    // the active failsafe recovery sequence to finish.
    if (failsafeIsActive() || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        if (autonomousModeState != AUTONOMOUS_MODE_WAITING_FOR_ARM) {
            autonomousModeAbort();
        }
        return;
    }

    switch (autonomousModeState) {
    case AUTONOMOUS_MODE_IDLE:
        return;

    case AUTONOMOUS_MODE_WAITING_FOR_ARM:
        if (!autonomousUser2CycleRequested || !IS_RC_MODE_ACTIVE(BOXARM)) {
            autonomousModeResetToIdle();
            return;
        }

        if (!rxAreFlightChannelsValid()
            || !failsafeIsReceivingRxData()
            || (getArmingDisableFlags() & (ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_RX_FAILSAFE))) {
            return;
        }

        // Start the fixed delay only after failsafe has completed and RX is
        // usable, so the requested pause cannot overlap the disarm sequence.
        if (!autonomousUser2ArmDelayStarted) {
            autonomousUser2ArmDelayStarted = true;
            autonomousUser2ArmDelayStartUs = currentTimeUs;
        }
        if (cmpTimeUs(currentTimeUs, autonomousUser2ArmDelayStartUs) < AUTONOMOUS_USER2_ARM_DELAY_US) {
            return;
        }

        // USER2 is the explicit autonomous request. BOXARM remains a
        // required pilot permission; acceleration must be detected before arming.
        unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        unsetArmingDisabled(ARMING_DISABLED_NOT_DISARMED);
        autonomousModeStartAccelerationDetection();
        return;

    case AUTONOMOUS_MODE_ACCEL_WAITING:
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
        unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        unsetArmingDisabled(ARMING_DISABLED_NOT_DISARMED);
        if (tryArmAutonomous()) {
            autonomousModeResetHoverStability();
            if (sensors(SENSOR_ACC)
                && acc.isAccelUpdatedAtLeastOnce
                && getCosTiltAngle() < AUTONOMOUS_FLIP_START_COS_TILT) {
                autonomousModeStartFlip(currentTimeUs);
                autonomousModeState = AUTONOMOUS_MODE_FLIP;
            } else {
                autonomousModeState = AUTONOMOUS_MODE_HOVER_ENTRY;
            }
        }
    }
    return;

    case AUTONOMOUS_MODE_FLIP:
        if (!ARMING_FLAG(ARMED)) {
            autonomousModeResetToIdle();
            return;
        }

        if (cmpTimeUs(currentTimeUs, autonomousFlipStartUs) >= AUTONOMOUS_FLIP_TIMEOUT_US) {
            disarm(DISARM_REASON_CRASH_PROTECTION);
            autonomousModeResetToIdle();
            return;
        }

        autonomousModeUpdateFlipControl(currentTimeUs);
        // The entry threshold is intentionally permissive, but using it as
        // the exit threshold would stop a 180-degree recovery at about
        // 78.5 degrees of tilt. Continue until the aircraft is close to
        // upright, then let HOVER_ENTRY finish the last few degrees.
        if (getCosTiltAngle() >= AUTONOMOUS_FLIP_END_COS_TILT) {
            autonomousFlipRateDps = 0.0f;
            autonomousModeResetHoverStability();
            autonomousModeState = AUTONOMOUS_MODE_HOVER_ENTRY;
        }
        return;
    case AUTONOMOUS_MODE_ARMED:
    case AUTONOMOUS_MODE_HOVER_ENTRY:
        if (!ARMING_FLAG(ARMED)) {
            autonomousModeResetHoverStability();
            autonomousModeState = AUTONOMOUS_MODE_IDLE;
            return;
        }

        // 姿态数据可能在解锁后才更新；如果此时发现倒扣，补进翻转流程。
        if (sensors(SENSOR_ACC)
            && acc.isAccelUpdatedAtLeastOnce
            && getCosTiltAngle() < AUTONOMOUS_FLIP_START_COS_TILT) {
            autonomousModeStartFlip(currentTimeUs);
            autonomousModeState = AUTONOMOUS_MODE_FLIP;
            return;
        }

        const bool hoverLevel =
            fabsf((float)attitude.values.roll) * 0.1f <= AUTONOMOUS_HOVER_LEVEL_ANGLE_DEG
            && fabsf((float)attitude.values.pitch) * 0.1f <= AUTONOMOUS_HOVER_LEVEL_ANGLE_DEG;

        const bool hoverStable =
            hoverLevel
            && gyroAbsRateDps(FD_ROLL) <= AUTONOMOUS_HOVER_STABLE_RATE_DPS
            && gyroAbsRateDps(FD_PITCH) <= AUTONOMOUS_HOVER_STABLE_RATE_DPS
            && gyroAbsRateDps(FD_YAW) <= AUTONOMOUS_HOVER_STABLE_RATE_DPS;

        if (!hoverStable) {
            autonomousModeResetHoverStability();

            const bool hoverTiltExcessive =
                fabsf((float)attitude.values.roll) * 0.1f > AUTONOMOUS_HOVER_ABORT_ANGLE_DEG
                || fabsf((float)attitude.values.pitch) * 0.1f > AUTONOMOUS_HOVER_ABORT_ANGLE_DEG;

            if (autonomousModeState == AUTONOMOUS_MODE_ARMED && hoverTiltExcessive) {
                autonomousModeState = AUTONOMOUS_MODE_HOVER_ENTRY;
            }

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

    }
}

// [新增] 功能：将翻转轴、方向、角速度和油门控制量恢复为初始值。
static void autonomousModeResetFlip(void)
{
    autonomousFlipAxis = FD_ROLL;
    autonomousFlipDirection = 1.0f;
    autonomousFlipRateDps = 0.0f;
    autonomousFlipThrottlePwm = autonomousModeThrottleFromHoverRatio(AUTONOMOUS_FLIP_THROTTLE_RATIO);
    autonomousFlipStartUs = 0;
    autonomousFlipLastUpdateUs = 0;
}

// [新增] 功能：根据当前姿态选择翻转轴和方向，并初始化翻转控制。
static void autonomousModeStartFlip(timeUs_t currentTimeUs)
{
    const float gravityX = rMat.m[NWU_U][X];
    const float gravityY = rMat.m[NWU_U][Y];

    autonomousFlipStartUs = currentTimeUs;
    autonomousFlipLastUpdateUs = currentTimeUs;
    autonomousFlipRateDps = 0.0f;
    autonomousFlipThrottlePwm = autonomousModeThrottleFromHoverRatio(AUTONOMOUS_FLIP_THROTTLE_RATIO);

    // Select and lock one body axis. The signs match the existing Angle
    // controller's roll/pitch error convention.
    if (fabsf(gravityX) > fabsf(gravityY)) {
        autonomousFlipAxis = FD_PITCH;
        autonomousFlipDirection = gravityX >= 0.0f ? 1.0f : -1.0f;
    } else {
        autonomousFlipAxis = FD_ROLL;
        autonomousFlipDirection = gravityY <= 0.0f ? 1.0f : -1.0f;
    }

    // At exact inversion the horizontal gravity projection is undefined.
    if (fabsf(gravityX) < 0.05f && fabsf(gravityY) < 0.05f) {
        autonomousFlipAxis = FD_ROLL;
        autonomousFlipDirection = 1.0f;
    }

    pidResetIterm();
}

// [新增] 功能：根据倾角和角速度闭环更新翻转角速度及油门。
static void autonomousModeUpdateFlipControl(timeUs_t currentTimeUs)
{
    const timeDelta_t elapsedUs = cmpTimeUs(currentTimeUs, autonomousFlipLastUpdateUs);
    const float dt = constrainf((float)elapsedUs * 1e-6f, 0.0005f, 0.01f);
    const float cosTilt = getCosTiltAngle();
    const float brakingProgress = constrainf(cosTilt / AUTONOMOUS_FLIP_START_COS_TILT, 0.0f, 1.0f);
    const float targetRate = AUTONOMOUS_FLIP_RATE_DPS * (1.0f - brakingProgress);
    const float actualRate = fabsf(gyro.gyroADCf[autonomousFlipAxis]);
    const float flipThrottle = (float)autonomousModeThrottleFromHoverRatio(AUTONOMOUS_FLIP_THROTTLE_RATIO);
    const float flipThrottleMin = (float)autonomousModeThrottleFromHoverRatio(AUTONOMOUS_FLIP_THROTTLE_MIN_RATIO);
    const float flipThrottleMax = (float)autonomousModeThrottleFromHoverRatio(AUTONOMOUS_FLIP_THROTTLE_MAX_RATIO);
    const float rateError = targetRate - actualRate;
    const float targetThrottle = constrainf(
        flipThrottle + rateError * AUTONOMOUS_FLIP_RATE_KP_PWM_PER_DPS,
        flipThrottleMin,
        flipThrottleMax);
    const float maxThrottleChange = AUTONOMOUS_FLIP_THROTTLE_SLEW_PWM_PER_S * dt;

    autonomousFlipThrottlePwm = (uint16_t)lrintf(constrainf(
        targetThrottle,
        (float)autonomousFlipThrottlePwm - maxThrottleChange,
        (float)autonomousFlipThrottlePwm + maxThrottleChange));
    autonomousFlipRateDps = fminf(
        autonomousFlipRateDps + AUTONOMOUS_FLIP_RATE_DPS * dt / 0.12f,
        AUTONOMOUS_FLIP_RATE_DPS);
    autonomousFlipLastUpdateUs = currentTimeUs;
}
