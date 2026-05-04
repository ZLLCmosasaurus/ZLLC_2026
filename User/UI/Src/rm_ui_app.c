#include "rm_ui_app.h"

#include <stddef.h>
#include <string.h>

#if defined(GIMBAL) && defined(CHASSIS)
#error "GIMBAL and CHASSIS cannot be defined at the same time"
#endif

#if !defined(GIMBAL) && !defined(CHASSIS)
#error "Either GIMBAL or CHASSIS must be defined"
#endif

#if defined(CHASSIS)
#include "ui.h"
#endif

#define RM_UI_COLOR_YELLOW 1U
#define RM_UI_COLOR_GREEN 2U
#define RM_UI_COLOR_ORANGE 3U
#define RM_UI_COLOR_RED 4U
#define RM_UI_COLOR_PURPLE 5U
#define RM_UI_COLOR_CYAN 6U

#define RM_UI_TEXT_CAPACITY 30U

#if defined(GIMBAL)

static rm_ui_remote_state_t g_rm_ui_remote_state = {
    .speed_mode = RM_UI_SPEED_SLOW,
    .running_mode = RM_UI_RUNNING_WORKING,
    .fsm_stage = 0,
    .gripper_status = RM_UI_GRIPPER_CLOSE,
    .controller_type = RM_UI_CONTROLLER_JOYSTICK,
};

static rm_ui_speed_mode_t rm_ui_sanitize_speed_mode(const rm_ui_speed_mode_t mode)
{
    return mode == RM_UI_SPEED_AXEL ? RM_UI_SPEED_AXEL : RM_UI_SPEED_SLOW;
}

static rm_ui_running_mode_t rm_ui_sanitize_running_mode(const rm_ui_running_mode_t mode)
{
    switch (mode) {
    case RM_UI_RUNNING_DISABLE:
    case RM_UI_RUNNING_WORKING:
    case RM_UI_RUNNING_MOVING:
    case RM_UI_RUNNING_UPLIFT:
    case RM_UI_RUNNING_DOWNLIFT:
    case RM_UI_RUNNING_SAVELOAD:
        return mode;
    default:
        return RM_UI_RUNNING_WORKING;
    }
}

static rm_ui_gripper_status_t rm_ui_sanitize_gripper_status(const rm_ui_gripper_status_t status)
{
    return status == RM_UI_GRIPPER_OPEN ? RM_UI_GRIPPER_OPEN : RM_UI_GRIPPER_CLOSE;
}

static rm_ui_controller_type_t rm_ui_sanitize_controller_type(const rm_ui_controller_type_t type)
{
    return type == RM_UI_CONTROLLER_KEYBOARD ? RM_UI_CONTROLLER_KEYBOARD : RM_UI_CONTROLLER_JOYSTICK;
}

void RM_UI_RemoteSetSpeedMode(const rm_ui_speed_mode_t mode)
{
    g_rm_ui_remote_state.speed_mode = rm_ui_sanitize_speed_mode(mode);
}

void RM_UI_RemoteSetRunningMode(const rm_ui_running_mode_t mode)
{
    g_rm_ui_remote_state.running_mode = rm_ui_sanitize_running_mode(mode);
}

void RM_UI_RemoteSetFSMStage(const uint8_t stage)
{
    g_rm_ui_remote_state.fsm_stage = stage > 9U ? 9U : stage;
}

void RM_UI_RemoteSetGripperStatus(const rm_ui_gripper_status_t status)
{
    g_rm_ui_remote_state.gripper_status = rm_ui_sanitize_gripper_status(status);
}

void RM_UI_RemoteSetControllerType(const rm_ui_controller_type_t type)
{
    g_rm_ui_remote_state.controller_type = rm_ui_sanitize_controller_type(type);
}

void RM_UI_RemoteGetState(rm_ui_remote_state_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = g_rm_ui_remote_state;
}

void RM_UI_RemotePackState(uint8_t data[RM_UI_REMOTE_STATE_CAN_DLC])
{
    if (data == NULL) {
        return;
    }

    memset(data, 0, RM_UI_REMOTE_STATE_CAN_DLC);
    data[RM_UI_REMOTE_STATE_IDX_SPEED_MODE] = (uint8_t)g_rm_ui_remote_state.speed_mode;
    data[RM_UI_REMOTE_STATE_IDX_RUNNING_MODE] = (uint8_t)g_rm_ui_remote_state.running_mode;
    data[RM_UI_REMOTE_STATE_IDX_FSM_STAGE] = g_rm_ui_remote_state.fsm_stage;
    data[RM_UI_REMOTE_STATE_IDX_GRIPPER_STATUS] = (uint8_t)g_rm_ui_remote_state.gripper_status;
    data[RM_UI_REMOTE_STATE_IDX_CONTROLLER_TYPE] = (uint8_t)g_rm_ui_remote_state.controller_type;
}

#endif

#if defined(CHASSIS)

typedef struct {
    float uplift_rf_percent;
    float uplift_lf_percent;
    float uplift_rb_percent;
    float uplift_lb_percent;
    float power_percent;
    rm_ui_speed_mode_t speed_mode;
    rm_ui_running_mode_t running_mode;
    uint8_t fsm_stage;
    rm_ui_wheel_status_t wheel_status;
    rm_ui_gripper_status_t gripper_status;
    rm_ui_controller_type_t controller_type;
} rm_ui_state_t;

static rm_ui_state_t g_rm_ui_state;

static rm_ui_speed_mode_t rm_ui_sanitize_speed_mode(const rm_ui_speed_mode_t mode)
{
    return mode == RM_UI_SPEED_AXEL ? RM_UI_SPEED_AXEL : RM_UI_SPEED_SLOW;
}

static rm_ui_running_mode_t rm_ui_sanitize_running_mode(const rm_ui_running_mode_t mode)
{
    switch (mode) {
    case RM_UI_RUNNING_DISABLE:
    case RM_UI_RUNNING_WORKING:
    case RM_UI_RUNNING_MOVING:
    case RM_UI_RUNNING_UPLIFT:
    case RM_UI_RUNNING_DOWNLIFT:
    case RM_UI_RUNNING_SAVELOAD:
        return mode;
    default:
        return RM_UI_RUNNING_WORKING;
    }
}

static rm_ui_wheel_status_t rm_ui_sanitize_wheel_status(const rm_ui_wheel_status_t status)
{
    return status == RM_UI_WHEEL_OFF ? RM_UI_WHEEL_OFF : RM_UI_WHEEL_ON;
}

static rm_ui_gripper_status_t rm_ui_sanitize_gripper_status(const rm_ui_gripper_status_t status)
{
    return status == RM_UI_GRIPPER_OPEN ? RM_UI_GRIPPER_OPEN : RM_UI_GRIPPER_CLOSE;
}

static rm_ui_controller_type_t rm_ui_sanitize_controller_type(const rm_ui_controller_type_t type)
{
    return type == RM_UI_CONTROLLER_KEYBOARD ? RM_UI_CONTROLLER_KEYBOARD : RM_UI_CONTROLLER_JOYSTICK;
}

static float rm_ui_clamp_percent_0_100(const float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 100.0f) {
        return 100.0f;
    }
    return value;
}

static float rm_ui_clamp_min_zero(const float value)
{
    return value < 0.0f ? 0.0f : value;
}

static int rm_ui_round_positive(const float value)
{
    if (value <= 0.0f) {
        return 0;
    }
    return (int)(value + 0.5f);
}

static void rm_ui_set_text(ui_interface_string_t *target, const char *text, const uint32_t color)
{
    size_t text_length = 0U;

    if (target == NULL || text == NULL) {
        return;
    }

    memset(target->string, 0, sizeof(target->string));
    strncpy(target->string, text, sizeof(target->string) - 1U);
    text_length = strlen(target->string);

    target->color = color;
    target->str_length = (uint32_t)text_length;
}

static void rm_ui_clear_text(ui_interface_string_t *target)
{
    if (target == NULL) {
        return;
    }

    memset(target->string, 0, sizeof(target->string));
    target->str_length = 0U;
}

static void rm_ui_apply_uplift(ui_interface_rect_t *target, const float percent)
{
    const float clamped_percent = rm_ui_clamp_percent_0_100(percent);
    const float height_f = 10.0f + clamped_percent * 135.0f / 100.0f;
    const int height = rm_ui_round_positive(height_f);

    if (target == NULL) {
        return;
    }

    target->color = RM_UI_COLOR_ORANGE;
    target->end_y = 383U;
    target->start_y = (uint32_t)(383 - height);
}

static void rm_ui_apply_power(const float percent)
{
    const float clamped_percent = rm_ui_clamp_min_zero(percent);
    const float width_f = 3.0f + clamped_percent * 677.0f / 100.0f;
    int end_x = 606 + rm_ui_round_positive(width_f);

    if (end_x > 2047) {
        end_x = 2047;
    }

    ui_g_RobotStatus_Now_Power->color = clamped_percent > 100.0f ? RM_UI_COLOR_RED : RM_UI_COLOR_GREEN;
    ui_g_RobotStatus_Now_Power->start_x = 606U;
    ui_g_RobotStatus_Now_Power->end_x = (uint32_t)end_x;
}

static void rm_ui_apply_speed_mode(const rm_ui_speed_mode_t mode)
{
    ui_g_RobotStatus_SpeedCircle->color =
        mode == RM_UI_SPEED_AXEL ? RM_UI_COLOR_ORANGE : RM_UI_COLOR_GREEN;
}

static void rm_ui_apply_running_mode(const rm_ui_running_mode_t mode)
{
    switch (mode) {
    case RM_UI_RUNNING_DISABLE:
        rm_ui_set_text(ui_g_RobotMode_RunningMode, "DISABLE", RM_UI_COLOR_RED);
        break;
    case RM_UI_RUNNING_MOVING:
        rm_ui_set_text(ui_g_RobotMode_RunningMode, "MOVING", RM_UI_COLOR_ORANGE);
        break;
    case RM_UI_RUNNING_UPLIFT:
        rm_ui_set_text(ui_g_RobotMode_RunningMode, "UPLIFT", RM_UI_COLOR_YELLOW);
        break;
    case RM_UI_RUNNING_DOWNLIFT:
        rm_ui_set_text(ui_g_RobotMode_RunningMode, "DOWNLIFT", RM_UI_COLOR_PURPLE);
        break;
    case RM_UI_RUNNING_SAVELOAD:
        rm_ui_set_text(ui_g_RobotMode_RunningMode, "SAVELOAD", RM_UI_COLOR_CYAN);
        break;
    case RM_UI_RUNNING_WORKING:
    default:
        rm_ui_set_text(ui_g_RobotMode_RunningMode, "WORKING", RM_UI_COLOR_GREEN);
        break;
    }
}

static void rm_ui_apply_fsm_stage(const uint8_t stage)
{
    char text[RM_UI_TEXT_CAPACITY];

    memset(text, 0, sizeof(text));
    text[0] = 'S';
    text[1] = 'T';
    text[2] = 'A';
    text[3] = 'G';
    text[4] = 'E';
    text[5] = ' ';
    text[6] = (char)('0' + (stage > 9U ? 9U : stage));

    rm_ui_set_text(ui_g_RobotStatus_FSM_Status, text, RM_UI_COLOR_ORANGE);
}

static void rm_ui_apply_wheel_status(const rm_ui_wheel_status_t status)
{
    if (status == RM_UI_WHEEL_OFF) {
        rm_ui_set_text(ui_g_RobotStatus_Wheel_Status, "OFF", RM_UI_COLOR_ORANGE);
        return;
    }

    rm_ui_set_text(ui_g_RobotStatus_Wheel_Status, "ON", RM_UI_COLOR_GREEN);
}

static void rm_ui_apply_gripper_status(const rm_ui_gripper_status_t status)
{
    if (status == RM_UI_GRIPPER_OPEN) {
        rm_ui_set_text(ui_g_RobotStatus_Gripper_Status, "OPEN", RM_UI_COLOR_GREEN);
        return;
    }

    rm_ui_set_text(ui_g_RobotStatus_Gripper_Status, "CLOSE", RM_UI_COLOR_ORANGE);
}

static void rm_ui_apply_controller_type(const rm_ui_controller_type_t type)
{
    if (type == RM_UI_CONTROLLER_KEYBOARD) {
        rm_ui_set_text(ui_g_RobotMode_ControllerType, "KEYBOARD", RM_UI_COLOR_GREEN);
        return;
    }

    rm_ui_set_text(ui_g_RobotMode_ControllerType, "JOYSTICK", RM_UI_COLOR_YELLOW);
}

static void rm_ui_set_default_state(void)
{
    memset(&g_rm_ui_state, 0, sizeof(g_rm_ui_state));

    g_rm_ui_state.speed_mode = RM_UI_SPEED_SLOW;
    g_rm_ui_state.running_mode = RM_UI_RUNNING_WORKING;
    g_rm_ui_state.fsm_stage = 0U;
    g_rm_ui_state.wheel_status = RM_UI_WHEEL_ON;
    g_rm_ui_state.gripper_status = RM_UI_GRIPPER_CLOSE;
    g_rm_ui_state.controller_type = RM_UI_CONTROLLER_JOYSTICK;
}

void RM_UI_Init(const uint16_t self_id)
{
    ui_self_id = self_id;
    ui_init_g();

    rm_ui_set_default_state();
    rm_ui_clear_text(ui_g_RobotStatus_Orientation);
}

void RM_UI_Commit(void)
{
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_RF, g_rm_ui_state.uplift_rf_percent);
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_LF, g_rm_ui_state.uplift_lf_percent);
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_RB, g_rm_ui_state.uplift_rb_percent);
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_LB, g_rm_ui_state.uplift_lb_percent);

    rm_ui_apply_power(g_rm_ui_state.power_percent);
    rm_ui_apply_speed_mode(g_rm_ui_state.speed_mode);
    rm_ui_apply_running_mode(g_rm_ui_state.running_mode);
    rm_ui_apply_fsm_stage(g_rm_ui_state.fsm_stage);
    rm_ui_apply_wheel_status(g_rm_ui_state.wheel_status);
    rm_ui_apply_gripper_status(g_rm_ui_state.gripper_status);
    rm_ui_apply_controller_type(g_rm_ui_state.controller_type);

    ui_update_g();
}

void RM_UI_SetUplift(const float rf_percent, const float lf_percent, const float rb_percent, const float lb_percent)
{
    g_rm_ui_state.uplift_rf_percent = rm_ui_clamp_percent_0_100(rf_percent);
    g_rm_ui_state.uplift_lf_percent = rm_ui_clamp_percent_0_100(lf_percent);
    g_rm_ui_state.uplift_rb_percent = rm_ui_clamp_percent_0_100(rb_percent);
    g_rm_ui_state.uplift_lb_percent = rm_ui_clamp_percent_0_100(lb_percent);
}

void RM_UI_SetPower(const float power_percent)
{
    g_rm_ui_state.power_percent = rm_ui_clamp_min_zero(power_percent);
}

void RM_UI_SetWheelStatus(const rm_ui_wheel_status_t status)
{
    g_rm_ui_state.wheel_status = rm_ui_sanitize_wheel_status(status);
}

void RM_UI_SetSpeedMode(const rm_ui_speed_mode_t mode)
{
    g_rm_ui_state.speed_mode = rm_ui_sanitize_speed_mode(mode);
}

void RM_UI_SetRunningMode(const rm_ui_running_mode_t mode)
{
    g_rm_ui_state.running_mode = rm_ui_sanitize_running_mode(mode);
}

void RM_UI_SetFSMStage(const uint8_t stage)
{
    g_rm_ui_state.fsm_stage = stage > 9U ? 9U : stage;
}

void RM_UI_SetGripperStatus(const rm_ui_gripper_status_t status)
{
    g_rm_ui_state.gripper_status = rm_ui_sanitize_gripper_status(status);
}

void RM_UI_SetControllerType(const rm_ui_controller_type_t type)
{
    g_rm_ui_state.controller_type = rm_ui_sanitize_controller_type(type);
}

void RM_UI_ApplyRemoteState(const rm_ui_remote_state_t *state)
{
    if (state == NULL) {
        return;
    }

    g_rm_ui_state.speed_mode = rm_ui_sanitize_speed_mode(state->speed_mode);
    g_rm_ui_state.running_mode = rm_ui_sanitize_running_mode(state->running_mode);
    g_rm_ui_state.fsm_stage = state->fsm_stage > 9U ? 9U : state->fsm_stage;
    g_rm_ui_state.gripper_status = rm_ui_sanitize_gripper_status(state->gripper_status);
    g_rm_ui_state.controller_type = rm_ui_sanitize_controller_type(state->controller_type);
}

#endif
