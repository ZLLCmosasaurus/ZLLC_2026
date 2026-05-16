#ifndef RM_UI_APP_H
#define RM_UI_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef enum {
    RM_UI_MODE_DISABLE = 0,
    RM_UI_MODE_WORKING,
    RM_UI_MODE_MOVING,
    RM_UI_MODE_UPLIFT,
    RM_UI_MODE_DOWNLIFT,
    RM_UI_MODE_SAVELOAD,
} rm_ui_mode_t;

typedef enum {
    RM_UI_SPEED_SLOW = 0,
    RM_UI_SPEED_AXEL,
} rm_ui_speed_t;

typedef enum {
    RM_UI_WHEEL_OFF = 0,
    RM_UI_WHEEL_ON,
} rm_ui_wheel_t;

typedef enum {
    RM_UI_GRIPPER_CLOSE = 0,
    RM_UI_GRIPPER_OPEN,
} rm_ui_gripper_t;

typedef enum {
    RM_UI_INPUT_JOYSTICK = 0,
    RM_UI_INPUT_KEYBOARD,
} rm_ui_input_t;

typedef struct {
    rm_ui_speed_t speed;
    rm_ui_mode_t mode;
    uint8_t stage;
    rm_ui_gripper_t gripper;
    rm_ui_input_t input;
    uint8_t flags;
} rm_ui_sync_t;

#define RM_UI_SYNC_DLC 8U
#define RM_UI_SYNC_IDX_SPEED 0U
#define RM_UI_SYNC_IDX_MODE 1U
#define RM_UI_SYNC_IDX_STAGE 2U
#define RM_UI_SYNC_IDX_GRIPPER 3U
#define RM_UI_SYNC_IDX_INPUT 4U
#define RM_UI_SYNC_IDX_FLAGS 5U

#define RM_UI_SYNC_FLAG_FULL_REFRESH 0x01U

#if defined(GIMBAL)
void RM_UI_SyncSetSpeed(rm_ui_speed_t speed);
void RM_UI_SyncSetMode(rm_ui_mode_t mode);
void RM_UI_SyncSetStage(uint8_t stage);
void RM_UI_SyncSetGripper(rm_ui_gripper_t gripper);
void RM_UI_SyncSetInput(rm_ui_input_t input);
void RM_UI_SyncRequestFullRefresh(void);
void RM_UI_SyncGet(rm_ui_sync_t *out);
void RM_UI_SyncPack(uint8_t data[RM_UI_SYNC_DLC]);
#endif

#if defined(CHASSIS)
typedef struct {
    uint32_t now_ms;
    uint16_t period_ms;
} rm_ui_poll_t;

void RM_UI_Init(uint16_t self_id);
void RM_UI_RequestFullRefresh(void);
void RM_UI_RefreshStatic(void);
void RM_UI_RequestStaticRefresh(void);
void RM_UI_Flush(void);
void RM_UI_ServiceTx(void);
void RM_UI_OnTxComplete(void);
void RM_UI_Task(const rm_ui_poll_t *poll);
void RM_UI_Poll(const rm_ui_poll_t *poll);
void RM_UI_RequestFlush(void);

void RM_UI_SetLift(float rf_percent, float lf_percent, float rb_percent, float lb_percent);
void RM_UI_SetPower(float power_percent);
void RM_UI_SetOrientation(bool forehead);
void RM_UI_SetWheel(rm_ui_wheel_t wheel);

void RM_UI_SetSpeed(rm_ui_speed_t speed);
void RM_UI_SetMode(rm_ui_mode_t mode);
void RM_UI_SetStage(uint8_t stage);
void RM_UI_SetGripper(rm_ui_gripper_t gripper);
void RM_UI_SetInput(rm_ui_input_t input);

bool RM_UI_SyncUnpack(const uint8_t data[RM_UI_SYNC_DLC], rm_ui_sync_t *out);
void RM_UI_SyncApply(const rm_ui_sync_t *sync);
#endif

/* Compatibility aliases for older names. */
typedef rm_ui_mode_t rm_ui_running_mode_t;
typedef rm_ui_speed_t rm_ui_speed_mode_t;
typedef rm_ui_wheel_t rm_ui_wheel_status_t;
typedef rm_ui_gripper_t rm_ui_gripper_status_t;
typedef rm_ui_input_t rm_ui_controller_type_t;
typedef rm_ui_sync_t rm_ui_remote_state_t;
#if defined(CHASSIS)
typedef rm_ui_poll_t rm_ui_service_context_t;
#endif

#define RM_UI_RUNNING_DISABLE RM_UI_MODE_DISABLE
#define RM_UI_RUNNING_WORKING RM_UI_MODE_WORKING
#define RM_UI_RUNNING_MOVING RM_UI_MODE_MOVING
#define RM_UI_RUNNING_UPLIFT RM_UI_MODE_UPLIFT
#define RM_UI_RUNNING_DOWNLIFT RM_UI_MODE_DOWNLIFT
#define RM_UI_RUNNING_SAVELOAD RM_UI_MODE_SAVELOAD

#define RM_UI_CONTROLLER_JOYSTICK RM_UI_INPUT_JOYSTICK
#define RM_UI_CONTROLLER_KEYBOARD RM_UI_INPUT_KEYBOARD

#define RM_UI_REMOTE_STATE_CAN_DLC RM_UI_SYNC_DLC
#define RM_UI_REMOTE_STATE_IDX_SPEED_MODE RM_UI_SYNC_IDX_SPEED
#define RM_UI_REMOTE_STATE_IDX_RUNNING_MODE RM_UI_SYNC_IDX_MODE
#define RM_UI_REMOTE_STATE_IDX_FSM_STAGE RM_UI_SYNC_IDX_STAGE
#define RM_UI_REMOTE_STATE_IDX_GRIPPER_STATUS RM_UI_SYNC_IDX_GRIPPER
#define RM_UI_REMOTE_STATE_IDX_CONTROLLER_TYPE RM_UI_SYNC_IDX_INPUT
#define RM_UI_REMOTE_STATE_IDX_FLAGS RM_UI_SYNC_IDX_FLAGS

#define RM_UI_RemoteSetSpeedMode RM_UI_SyncSetSpeed
#define RM_UI_RemoteSetRunningMode RM_UI_SyncSetMode
#define RM_UI_RemoteSetFSMStage RM_UI_SyncSetStage
#define RM_UI_RemoteSetGripperStatus RM_UI_SyncSetGripper
#define RM_UI_RemoteSetControllerType RM_UI_SyncSetInput
#define RM_UI_RemoteGetState RM_UI_SyncGet
#define RM_UI_RemotePackState RM_UI_SyncPack

#if defined(GIMBAL)
#define RM_UI_RemoteRequestFullRefresh RM_UI_SyncRequestFullRefresh
#endif

#define RM_UI_Commit RM_UI_Flush
#define RM_UI_RefreshStaticElements RM_UI_RequestFullRefresh
#define RM_UI_RequestStaticElements RM_UI_RequestStaticRefresh
#define RM_UI_RunTxService RM_UI_ServiceTx
#define RM_UI_TxCpltCallback RM_UI_OnTxComplete
#define RM_UI_Schedule RM_UI_Task
#define RM_UI_Service RM_UI_Poll
#define RM_UI_RequestCommit RM_UI_RequestFlush
#define RM_UI_SetUplift RM_UI_SetLift
#define RM_UI_SetWheelStatus RM_UI_SetWheel
#define RM_UI_SetSpeedMode RM_UI_SetSpeed
#define RM_UI_SetRunningMode RM_UI_SetMode
#define RM_UI_SetFSMStage RM_UI_SetStage
#define RM_UI_SetGripperStatus RM_UI_SetGripper
#define RM_UI_SetControllerType RM_UI_SetInput
#define RM_UI_RemoteUnpackState RM_UI_SyncUnpack
#define RM_UI_ApplyRemoteState RM_UI_SyncApply

#ifdef __cplusplus
}
#endif

#endif // RM_UI_APP_H
