#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef enum {
    AUTONOMOUS_USER1_POSITION_LOW = 0,
    AUTONOMOUS_USER1_POSITION_MIDDLE,
    AUTONOMOUS_USER1_POSITION_HIGH,
    AUTONOMOUS_USER1_POSITION_INVALID,
} autonomousUser1Position_e;

typedef enum {
    AUTONOMOUS_MODE_IDLE = 0,
    AUTONOMOUS_MODE_WAITING_FOR_ARM,
    AUTONOMOUS_MODE_ACCEL_WAITING,
    AUTONOMOUS_MODE_FLIP,
    AUTONOMOUS_MODE_ARMED,
    AUTONOMOUS_MODE_HOVER_ENTRY,
} autonomousModeState_e;

// [新增] 功能：进入自由落体检测阶段。
void autonomousModeStartAccelerationDetection(void);
// [新增] 功能：中止当前自主流程并恢复空闲状态。
void autonomousModeAbort(void);
// [新增] 功能：按周期推进自主飞行状态机。
void autonomousModeUpdate(timeUs_t currentTimeUs);

// [新增] 功能：查询自主流程对飞手输入和控制模式的接管状态。
bool autonomousModeSuppressesPilotInput(void);
bool autonomousModeOwnsArming(void);
bool autonomousModeOverridesThrottle(void);
uint16_t autonomousModeGetThrottlePwm(void);
bool autonomousModeIsFlipActive(void);
// [新增] 功能：读取自主翻转阶段的角速度和油门输出。
float autonomousModeGetFlipRate(int axis);
uint16_t autonomousModeGetFlipThrottlePwm(void);
// [新增] 功能：查询自主流程请求的姿态、位置和高度控制模式。
bool autonomousModeRequestsAngle(void);
bool autonomousModeRequestsAltitudeHold(void);
bool autonomousModeRequestsPositionHold(void);
bool autonomousModeSuppressesPilotHoldModes(void);
// [新增] 功能：读取当前自主状态和 USER1 原始拨杆位置。
autonomousModeState_e autonomousModeGetState(void);
autonomousUser1Position_e autonomousModeGetUser1Position(void);
// [兼容] USER1 不直接请求高度/位置保持；这两个接口始终返回 false。
bool autonomousModeUserRequestsAltitudeHold(void);
bool autonomousModeUserRequestsPositionHold(void);
