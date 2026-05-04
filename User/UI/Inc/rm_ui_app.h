#ifndef RM_UI_APP_H
#define RM_UI_APP_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RM_UI_RUNNING_DISABLE = 0,
    RM_UI_RUNNING_WORKING,
    RM_UI_RUNNING_MOVING,
    RM_UI_RUNNING_UPLIFT,
    RM_UI_RUNNING_DOWNLIFT,
    RM_UI_RUNNING_SAVELOAD,
} rm_ui_running_mode_t;

typedef enum {
    RM_UI_SPEED_SLOW = 0,
    RM_UI_SPEED_AXEL,
} rm_ui_speed_mode_t;

typedef enum {
    RM_UI_WHEEL_OFF = 0,
    RM_UI_WHEEL_ON,
} rm_ui_wheel_status_t;

typedef enum {
    RM_UI_GRIPPER_CLOSE = 0,
    RM_UI_GRIPPER_OPEN,
} rm_ui_gripper_status_t;

typedef enum {
    RM_UI_CONTROLLER_JOYSTICK = 0,
    RM_UI_CONTROLLER_KEYBOARD,
} rm_ui_controller_type_t;

typedef struct {
    rm_ui_speed_mode_t speed_mode;
    rm_ui_running_mode_t running_mode;
    uint8_t fsm_stage;
    rm_ui_gripper_status_t gripper_status;
    rm_ui_controller_type_t controller_type;
} rm_ui_remote_state_t;

#define RM_UI_REMOTE_STATE_CAN_DLC 8U
#define RM_UI_REMOTE_STATE_IDX_SPEED_MODE 0U
#define RM_UI_REMOTE_STATE_IDX_RUNNING_MODE 1U
#define RM_UI_REMOTE_STATE_IDX_FSM_STAGE 2U
#define RM_UI_REMOTE_STATE_IDX_GRIPPER_STATUS 3U
#define RM_UI_REMOTE_STATE_IDX_CONTROLLER_TYPE 4U

#if defined(GIMBAL)
void RM_UI_RemoteSetSpeedMode(rm_ui_speed_mode_t mode);
void RM_UI_RemoteSetRunningMode(rm_ui_running_mode_t mode);
void RM_UI_RemoteSetFSMStage(uint8_t stage);
void RM_UI_RemoteSetGripperStatus(rm_ui_gripper_status_t status);
void RM_UI_RemoteSetControllerType(rm_ui_controller_type_t type);
void RM_UI_RemoteGetState(rm_ui_remote_state_t *out);
void RM_UI_RemotePackState(uint8_t data[RM_UI_REMOTE_STATE_CAN_DLC]);
#endif

#if defined(CHASSIS)
void RM_UI_Init(uint16_t self_id);
void RM_UI_Commit(void);

void RM_UI_SetUplift(float rf_percent, float lf_percent, float rb_percent, float lb_percent);
void RM_UI_SetPower(float power_percent);
void RM_UI_SetWheelStatus(rm_ui_wheel_status_t status);

void RM_UI_SetSpeedMode(rm_ui_speed_mode_t mode);
void RM_UI_SetRunningMode(rm_ui_running_mode_t mode);
void RM_UI_SetFSMStage(uint8_t stage);
void RM_UI_SetGripperStatus(rm_ui_gripper_status_t status);
void RM_UI_SetControllerType(rm_ui_controller_type_t type);

void RM_UI_ApplyRemoteState(const rm_ui_remote_state_t *state);
#endif

#ifdef __cplusplus
}
#endif

#endif // RM_UI_APP_H
