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
#define RM_UI_COLOR_WHITE 8U

#define RM_UI_TEXT_CAPACITY 30U

#if defined(CHASSIS)
#define RM_UI_TOTAL_FIGURES ((int)(sizeof(ui_g_dirty_figure) / sizeof(ui_g_dirty_figure[0])))
#define RM_UI_TOTAL_STRINGS ((int)(sizeof(ui_g_dirty_string) / sizeof(ui_g_dirty_string[0])))
#define RM_UI_INIT_REDRAW_ROUNDS 3U
#define RM_UI_INIT_PACKETS_PER_TICK 3U
#define RM_UI_FIGURES_PER_PACKET 7U
#define RM_UI_OPERATE_ADD 1U
#define RM_UI_OPERATE_MODIFY 2U
#define RM_UI_ENTER_CRITICAL(primask) do { (primask) = __get_PRIMASK(); __disable_irq(); } while (0)
#define RM_UI_EXIT_CRITICAL(primask) __set_PRIMASK(primask)
#endif

#if defined(GIMBAL)

static rm_ui_sync_t g_rm_ui_remote_state = {
    .speed = RM_UI_SPEED_SLOW,
    .mode = RM_UI_MODE_WORKING,
    .stage = 0,
    .gripper = RM_UI_GRIPPER_CLOSE,
    .input = RM_UI_INPUT_JOYSTICK,
    .flags = 0U,
};

static rm_ui_speed_t rm_ui_sanitize_speed(const rm_ui_speed_t mode)
{
    return mode == RM_UI_SPEED_AXEL ? RM_UI_SPEED_AXEL : RM_UI_SPEED_SLOW;
}

static rm_ui_mode_t rm_ui_sanitize_mode(const rm_ui_mode_t mode)
{
    switch (mode) {
    case RM_UI_MODE_DISABLE:
    case RM_UI_MODE_WORKING:
    case RM_UI_MODE_MOVING:
    case RM_UI_MODE_UPLIFT:
    case RM_UI_MODE_DOWNLIFT:
    case RM_UI_MODE_SAVELOAD:
        return mode;
    default:
        return RM_UI_MODE_WORKING;
    }
}

static rm_ui_gripper_t rm_ui_sanitize_gripper(const rm_ui_gripper_t status)
{
    return status == RM_UI_GRIPPER_OPEN ? RM_UI_GRIPPER_OPEN : RM_UI_GRIPPER_CLOSE;
}

static rm_ui_input_t rm_ui_sanitize_input(const rm_ui_input_t type)
{
    return type == RM_UI_INPUT_KEYBOARD ? RM_UI_INPUT_KEYBOARD : RM_UI_INPUT_JOYSTICK;
}

void RM_UI_SyncSetSpeed(const rm_ui_speed_t mode)
{
    g_rm_ui_remote_state.speed = rm_ui_sanitize_speed(mode);
}

void RM_UI_SyncSetMode(const rm_ui_mode_t mode)
{
    g_rm_ui_remote_state.mode = rm_ui_sanitize_mode(mode);
}

void RM_UI_SyncSetStage(const uint8_t stage)
{
    g_rm_ui_remote_state.stage = stage > 9U ? 9U : stage;
}

void RM_UI_SyncSetGripper(const rm_ui_gripper_t status)
{
    g_rm_ui_remote_state.gripper = rm_ui_sanitize_gripper(status);
}

void RM_UI_SyncSetInput(const rm_ui_input_t type)
{
    g_rm_ui_remote_state.input = rm_ui_sanitize_input(type);
}

void RM_UI_SyncRequestFullRefresh(void)
{
    g_rm_ui_remote_state.flags |= RM_UI_SYNC_FLAG_FULL_REFRESH;
}

void RM_UI_SyncGet(rm_ui_sync_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = g_rm_ui_remote_state;
}

void RM_UI_SyncPack(uint8_t data[RM_UI_SYNC_DLC])
{
    if (data == NULL) {
        return;
    }

    memset(data, 0, RM_UI_SYNC_DLC);
    data[RM_UI_SYNC_IDX_SPEED] = (uint8_t)g_rm_ui_remote_state.speed;
    data[RM_UI_SYNC_IDX_MODE] = (uint8_t)g_rm_ui_remote_state.mode;
    data[RM_UI_SYNC_IDX_STAGE] = g_rm_ui_remote_state.stage;
    data[RM_UI_SYNC_IDX_GRIPPER] = (uint8_t)g_rm_ui_remote_state.gripper;
    data[RM_UI_SYNC_IDX_INPUT] = (uint8_t)g_rm_ui_remote_state.input;
    data[RM_UI_SYNC_IDX_FLAGS] = g_rm_ui_remote_state.flags;
    g_rm_ui_remote_state.flags = 0U;
}

#endif

#if defined(CHASSIS)

typedef struct {
    float uplift_rf_percent;
    float uplift_lf_percent;
    float uplift_rb_percent;
    float uplift_lb_percent;
    float power_percent;
    bool orientation_forehead;
    rm_ui_speed_t speed;
    rm_ui_mode_t mode;
    uint8_t stage;
    rm_ui_wheel_t wheel;
    rm_ui_gripper_t gripper;
    rm_ui_input_t input;
} rm_ui_state_t;

typedef struct {
    uint8_t active;
    uint8_t rounds_left;
    uint8_t figure_cursor;
    uint8_t string_cursor;
} rm_ui_init_tx_t;

typedef struct {
    uint8_t active;
    uint8_t figure_cursor;
    uint8_t string_cursor;
} rm_ui_update_tx_t;

static rm_ui_state_t g_rm_ui_state;
static rm_ui_init_tx_t g_rm_ui_init_tx;
static rm_ui_update_tx_t g_rm_ui_update_tx;
static uint8_t g_rm_ui_flush_pending = 0;
static uint32_t g_rm_ui_last_flush_ms = 0U;
static uint8_t g_rm_ui_pending_figure[RM_UI_TOTAL_FIGURES];
static uint8_t g_rm_ui_pending_string[RM_UI_TOTAL_STRINGS];
static uint8_t g_rm_ui_update_figure[RM_UI_TOTAL_FIGURES];
static uint8_t g_rm_ui_update_string[RM_UI_TOTAL_STRINGS];

static const uint8_t g_rm_ui_dynamic_figures[] = {
    4U,  /* SpeedCircle */
    6U,  /* Now_Power */
    7U,  /* Now_RF */
    8U,  /* Now_LF */
    9U,  /* Now_RB */
    10U, /* Now_LB */
    11U, /* WoringRect */
    12U, /* MovingRect */
    13U, /* UpliftRect */
    14U, /* DownliftRect */
    15U, /* SaveloadRect */
    16U, /* FSM_Number */
};

static const uint8_t g_rm_ui_dynamic_strings[] = {
    0U,  /* Orientation */
    2U,  /* FSM_Status */
    4U,  /* Wheel_Status */
    6U,  /* Gripper_Status */
    7U,  /* WoringMode */
    8U,  /* MovingMode */
    9U,  /* UpliftMode */
    10U, /* DownliftMode */
    11U, /* SaveloadMode */
};

static const uint8_t g_rm_ui_lift_figures[] = {
    7U,  /* Now_RF */
    8U,  /* Now_LF */
    9U,  /* Now_RB */
    10U, /* Now_LB */
};

static const uint8_t g_rm_ui_power_figures[] = {
    6U, /* Now_Power */
};

static const uint8_t g_rm_ui_orientation_strings[] = {
    0U, /* Orientation */
};

static const uint8_t g_rm_ui_wheel_strings[] = {
    4U, /* Wheel_Status */
};

static const uint8_t g_rm_ui_speed_figures[] = {
    4U, /* SpeedCircle */
};

static const uint8_t g_rm_ui_mode_figures[] = {
    11U, /* WoringRect */
    12U, /* MovingRect */
    13U, /* UpliftRect */
    14U, /* DownliftRect */
    15U, /* SaveloadRect */
};

static const uint8_t g_rm_ui_mode_strings[] = {
    7U,  /* WoringMode */
    8U,  /* MovingMode */
    9U,  /* UpliftMode */
    10U, /* DownliftMode */
    11U, /* SaveloadMode */
};

static const uint8_t g_rm_ui_stage_figures[] = {
    16U, /* FSM_Number */
};

static const uint8_t g_rm_ui_stage_strings[] = {
    2U, /* FSM_Status */
};

static const uint8_t g_rm_ui_gripper_strings[] = {
    6U, /* Gripper_Status */
};

static const uint8_t g_rm_ui_remote_figures[] = {
    4U,  /* SpeedCircle */
    11U, /* WoringRect */
    12U, /* MovingRect */
    13U, /* UpliftRect */
    14U, /* DownliftRect */
    15U, /* SaveloadRect */
    16U, /* FSM_Number */
};

static const uint8_t g_rm_ui_remote_strings[] = {
    2U,  /* FSM_Status */
    6U,  /* Gripper_Status */
    7U,  /* WoringMode */
    8U,  /* MovingMode */
    9U,  /* UpliftMode */
    10U, /* DownliftMode */
    11U, /* SaveloadMode */
};

static rm_ui_speed_t rm_ui_sanitize_speed(const rm_ui_speed_t speed)
{
    return speed == RM_UI_SPEED_AXEL ? RM_UI_SPEED_AXEL : RM_UI_SPEED_SLOW;
}

static rm_ui_mode_t rm_ui_sanitize_mode(const rm_ui_mode_t mode)
{
    switch (mode) {
    case RM_UI_MODE_DISABLE:
    case RM_UI_MODE_WORKING:
    case RM_UI_MODE_MOVING:
    case RM_UI_MODE_UPLIFT:
    case RM_UI_MODE_DOWNLIFT:
    case RM_UI_MODE_SAVELOAD:
        return mode;
    default:
        return RM_UI_MODE_WORKING;
    }
}

static rm_ui_wheel_t rm_ui_sanitize_wheel(const rm_ui_wheel_t wheel)
{
    return wheel == RM_UI_WHEEL_OFF ? RM_UI_WHEEL_OFF : RM_UI_WHEEL_ON;
}

static rm_ui_gripper_t rm_ui_sanitize_gripper(const rm_ui_gripper_t gripper)
{
    return gripper == RM_UI_GRIPPER_OPEN ? RM_UI_GRIPPER_OPEN : RM_UI_GRIPPER_CLOSE;
}

static rm_ui_input_t rm_ui_sanitize_input(const rm_ui_input_t input)
{
    return input == RM_UI_INPUT_KEYBOARD ? RM_UI_INPUT_KEYBOARD : RM_UI_INPUT_JOYSTICK;
}

static bool rm_ui_remote_state_is_valid(const rm_ui_sync_t *state)
{
    if (state == NULL) {
        return false;
    }

    if (state->speed != RM_UI_SPEED_SLOW && state->speed != RM_UI_SPEED_AXEL) {
        return false;
    }

    switch (state->mode) {
    case RM_UI_MODE_DISABLE:
    case RM_UI_MODE_WORKING:
    case RM_UI_MODE_MOVING:
    case RM_UI_MODE_UPLIFT:
    case RM_UI_MODE_DOWNLIFT:
    case RM_UI_MODE_SAVELOAD:
        break;
    default:
        return false;
    }

    if (state->stage > 9U) {
        return false;
    }

    if (state->gripper != RM_UI_GRIPPER_CLOSE && state->gripper != RM_UI_GRIPPER_OPEN) {
        return false;
    }

    if (state->input != RM_UI_INPUT_JOYSTICK &&
        state->input != RM_UI_INPUT_KEYBOARD) {
        return false;
    }

    return true;
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

static void rm_ui_clear_dirty(void)
{
    size_t i = 0U;

    for (i = 0U; i < (size_t)RM_UI_TOTAL_FIGURES; i++) {
        ui_g_dirty_figure[i] = 0U;
    }
    for (i = 0U; i < (size_t)RM_UI_TOTAL_STRINGS; i++) {
        ui_g_dirty_string[i] = 0U;
    }
}

static void rm_ui_mark_pending_update(const uint8_t *figure_indices,
                                      const size_t figure_count,
                                      const uint8_t *string_indices,
                                      const size_t string_count)
{
    uint32_t primask = 0U;
    size_t i = 0U;

    RM_UI_ENTER_CRITICAL(primask);
    for (i = 0U; i < figure_count; i++) {
        if (figure_indices != NULL && figure_indices[i] < (uint8_t)RM_UI_TOTAL_FIGURES) {
            g_rm_ui_pending_figure[figure_indices[i]] = 1U;
        }
    }

    for (i = 0U; i < string_count; i++) {
        if (string_indices != NULL && string_indices[i] < (uint8_t)RM_UI_TOTAL_STRINGS) {
            g_rm_ui_pending_string[string_indices[i]] = 1U;
        }
    }

    g_rm_ui_flush_pending = 1U;
    RM_UI_EXIT_CRITICAL(primask);
}

static void rm_ui_mark_dynamic_dirty(void)
{
    rm_ui_mark_pending_update(g_rm_ui_dynamic_figures,
                              sizeof(g_rm_ui_dynamic_figures) / sizeof(g_rm_ui_dynamic_figures[0]),
                              g_rm_ui_dynamic_strings,
                              sizeof(g_rm_ui_dynamic_strings) / sizeof(g_rm_ui_dynamic_strings[0]));
}

static uint8_t rm_ui_pending_update_exists(void)
{
    uint32_t primask = 0U;
    size_t i = 0U;
    uint8_t exists = 0U;

    RM_UI_ENTER_CRITICAL(primask);
    for (i = 0U; i < (size_t)RM_UI_TOTAL_FIGURES; i++) {
        if (g_rm_ui_pending_figure[i] != 0U) {
            exists = 1U;
            break;
        }
    }

    if (exists == 0U) {
        for (i = 0U; i < (size_t)RM_UI_TOTAL_STRINGS; i++) {
            if (g_rm_ui_pending_string[i] != 0U) {
                exists = 1U;
                break;
            }
        }
    }

    RM_UI_EXIT_CRITICAL(primask);
    return exists;
}

static void rm_ui_reset_update_tx(void)
{
    uint32_t primask = 0U;

    RM_UI_ENTER_CRITICAL(primask);
    memset(g_rm_ui_pending_figure, 0, sizeof(g_rm_ui_pending_figure));
    memset(g_rm_ui_pending_string, 0, sizeof(g_rm_ui_pending_string));
    memset(g_rm_ui_update_figure, 0, sizeof(g_rm_ui_update_figure));
    memset(g_rm_ui_update_string, 0, sizeof(g_rm_ui_update_string));
    memset(&g_rm_ui_update_tx, 0, sizeof(g_rm_ui_update_tx));
    g_rm_ui_flush_pending = 0U;
    RM_UI_EXIT_CRITICAL(primask);
}

static void rm_ui_start_update_tx(void)
{
    uint32_t primask = 0U;

    RM_UI_ENTER_CRITICAL(primask);
    memcpy(g_rm_ui_update_figure, g_rm_ui_pending_figure, sizeof(g_rm_ui_update_figure));
    memcpy(g_rm_ui_update_string, g_rm_ui_pending_string, sizeof(g_rm_ui_update_string));
    memset(g_rm_ui_pending_figure, 0, sizeof(g_rm_ui_pending_figure));
    memset(g_rm_ui_pending_string, 0, sizeof(g_rm_ui_pending_string));
    g_rm_ui_update_tx.active = 1U;
    g_rm_ui_update_tx.figure_cursor = 0U;
    g_rm_ui_update_tx.string_cursor = 0U;
    g_rm_ui_flush_pending = 0U;
    RM_UI_EXIT_CRITICAL(primask);
}

static void rm_ui_set_all_operate_type(const uint8_t operate_type)
{
    int i = 0;

    for (i = 0; i < RM_UI_TOTAL_FIGURES; i++) {
        ui_g_now_figures[i].operate_type = operate_type;
    }
    for (i = 0; i < RM_UI_TOTAL_STRINGS; i++) {
        ui_g_now_strings[i].operate_type = operate_type;
    }
}

static void rm_ui_switch_all_to_modify(void)
{
    rm_ui_set_all_operate_type(RM_UI_OPERATE_MODIFY);
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

static void rm_ui_set_mode_rects(const uint32_t working,
                                 const uint32_t moving,
                                 const uint32_t uplift,
                                 const uint32_t downlift,
                                 const uint32_t saveload)
{
    ui_g_RunningRect_WoringRect->color = working;
    ui_g_RunningRect_MovingRect->color = moving;
    ui_g_RunningRect_UpliftRect->color = uplift;
    ui_g_RunningRect_DownliftRect->color = downlift;
    ui_g_RunningRect_SaveloadRect->color = saveload;
}

static void rm_ui_apply_speed(const rm_ui_speed_t mode)
{
    ui_g_RobotStatus_SpeedCircle->color =
        mode == RM_UI_SPEED_AXEL ? RM_UI_COLOR_ORANGE : RM_UI_COLOR_GREEN;
}

static void rm_ui_apply_mode(const rm_ui_mode_t mode)
{
    rm_ui_set_mode_rects(RM_UI_COLOR_ORANGE, RM_UI_COLOR_ORANGE, RM_UI_COLOR_ORANGE,
                         RM_UI_COLOR_ORANGE, RM_UI_COLOR_ORANGE);
    rm_ui_set_text(ui_g_RunningMode_WoringMode, "Z", RM_UI_COLOR_ORANGE);
    rm_ui_set_text(ui_g_RunningMode_MovingMode, "P", RM_UI_COLOR_ORANGE);
    rm_ui_set_text(ui_g_RunningMode_UpliftMode, "S", RM_UI_COLOR_ORANGE);
    rm_ui_set_text(ui_g_RunningMode_DownliftMode, "X", RM_UI_COLOR_ORANGE);
    rm_ui_set_text(ui_g_RunningMode_SaveloadMode, "Q", RM_UI_COLOR_ORANGE);

    switch (mode) {
    case RM_UI_MODE_DISABLE:
        break;
    case RM_UI_MODE_MOVING:
        ui_g_RunningRect_MovingRect->color = RM_UI_COLOR_GREEN;
        rm_ui_set_text(ui_g_RunningMode_MovingMode, "P", RM_UI_COLOR_GREEN);
        break;
    case RM_UI_MODE_UPLIFT:
        ui_g_RunningRect_UpliftRect->color = RM_UI_COLOR_GREEN;
        rm_ui_set_text(ui_g_RunningMode_UpliftMode, "S", RM_UI_COLOR_GREEN);
        break;
    case RM_UI_MODE_DOWNLIFT:
        ui_g_RunningRect_DownliftRect->color = RM_UI_COLOR_GREEN;
        rm_ui_set_text(ui_g_RunningMode_DownliftMode, "X", RM_UI_COLOR_GREEN);
        break;
    case RM_UI_MODE_SAVELOAD:
        ui_g_RunningRect_SaveloadRect->color = RM_UI_COLOR_GREEN;
        rm_ui_set_text(ui_g_RunningMode_SaveloadMode, "Q", RM_UI_COLOR_GREEN);
        break;
    case RM_UI_MODE_WORKING:
    default:
        ui_g_RunningRect_WoringRect->color = RM_UI_COLOR_GREEN;
        rm_ui_set_text(ui_g_RunningMode_WoringMode, "Z", RM_UI_COLOR_GREEN);
        break;
    }
}

static void rm_ui_apply_orientation(const bool forehead)
{
    if (forehead) {
        rm_ui_set_text(ui_g_RobotStatus_Orientation, "FOREHEAD", RM_UI_COLOR_GREEN);
        return;
    }

    rm_ui_set_text(ui_g_RobotStatus_Orientation, "REARBACK", RM_UI_COLOR_ORANGE);
}

static void rm_ui_apply_stage(const uint8_t stage)
{
    char text[RM_UI_TEXT_CAPACITY];
    const uint8_t clamped_stage = stage > 9U ? 9U : stage;

    memset(text, 0, sizeof(text));
    text[0] = 'S';
    text[1] = 'T';
    text[2] = 'A';
    text[3] = 'G';
    text[4] = 'E';
    text[5] = ' ';
    text[6] = ' ';

    rm_ui_set_text(ui_g_RobotStatus_FSM_Status, text, RM_UI_COLOR_YELLOW);
    ui_g_Now_FSM_FSM_Number->color = RM_UI_COLOR_YELLOW;
    ui_g_Now_FSM_FSM_Number->number = clamped_stage;
}

static void rm_ui_apply_wheel(const rm_ui_wheel_t status)
{
    if (status == RM_UI_WHEEL_OFF) {
        rm_ui_set_text(ui_g_RobotStatus_Wheel_Status, "OFF", RM_UI_COLOR_ORANGE);
        return;
    }

    rm_ui_set_text(ui_g_RobotStatus_Wheel_Status, "ON", RM_UI_COLOR_GREEN);
}

static void rm_ui_apply_gripper(const rm_ui_gripper_t status)
{
    if (status == RM_UI_GRIPPER_OPEN) {
        rm_ui_set_text(ui_g_RobotStatus_Gripper_Status, "OPEN", RM_UI_COLOR_GREEN);
        return;
    }

    rm_ui_set_text(ui_g_RobotStatus_Gripper_Status, "CLOSE", RM_UI_COLOR_ORANGE);
}

static void rm_ui_apply_input(const rm_ui_input_t type)
{
    (void)type;
}

static void rm_ui_set_default_state(void)
{
    memset(&g_rm_ui_state, 0, sizeof(g_rm_ui_state));

    g_rm_ui_state.orientation_forehead = true;
    g_rm_ui_state.speed = RM_UI_SPEED_SLOW;
    g_rm_ui_state.mode = RM_UI_MODE_WORKING;
    g_rm_ui_state.stage = 0U;
    g_rm_ui_state.wheel = RM_UI_WHEEL_ON;
    g_rm_ui_state.gripper = RM_UI_GRIPPER_CLOSE;
    g_rm_ui_state.input = RM_UI_INPUT_JOYSTICK;
}

static void rm_ui_apply_state_to_objects(void)
{
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_RF, g_rm_ui_state.uplift_rf_percent);
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_LF, g_rm_ui_state.uplift_lf_percent);
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_RB, g_rm_ui_state.uplift_rb_percent);
    rm_ui_apply_uplift(ui_g_RobotStatus_Now_LB, g_rm_ui_state.uplift_lb_percent);

    rm_ui_apply_power(g_rm_ui_state.power_percent);
    rm_ui_apply_orientation(g_rm_ui_state.orientation_forehead);
    rm_ui_apply_speed(g_rm_ui_state.speed);
    rm_ui_apply_mode(g_rm_ui_state.mode);
    rm_ui_apply_stage(g_rm_ui_state.stage);
    rm_ui_apply_wheel(g_rm_ui_state.wheel);
    rm_ui_apply_gripper(g_rm_ui_state.gripper);
    rm_ui_apply_input(g_rm_ui_state.input);
}

static uint8_t rm_ui_tx_queue_has_room(void)
{
    return ui_tx_queue_count() < UI_TX_QUEUE_DEPTH ? 1U : 0U;
}

static void rm_ui_mark_next_figure_packet(void)
{
    uint8_t count = 0U;

    rm_ui_clear_dirty();

    while (g_rm_ui_init_tx.figure_cursor < (uint8_t)RM_UI_TOTAL_FIGURES &&
           count < RM_UI_FIGURES_PER_PACKET) {
        ui_g_dirty_figure[g_rm_ui_init_tx.figure_cursor] = 1U;
        g_rm_ui_init_tx.figure_cursor++;
        count++;
    }
}

static void rm_ui_mark_next_string_packet(void)
{
    rm_ui_clear_dirty();

    if (g_rm_ui_init_tx.string_cursor < (uint8_t)RM_UI_TOTAL_STRINGS) {
        ui_g_dirty_string[g_rm_ui_init_tx.string_cursor] = 1U;
        g_rm_ui_init_tx.string_cursor++;
    }
}

static void rm_ui_start_init_redraw(const uint8_t rounds)
{
    rm_ui_reset_update_tx();
    g_rm_ui_init_tx.active = 1U;
    g_rm_ui_init_tx.rounds_left = rounds == 0U ? 1U : rounds;
    g_rm_ui_init_tx.figure_cursor = 0U;
    g_rm_ui_init_tx.string_cursor = 0U;
}

static void rm_ui_finish_init_redraw(void)
{
    const uint8_t pending_flush = g_rm_ui_flush_pending;

    g_rm_ui_init_tx.active = 0U;
    g_rm_ui_init_tx.rounds_left = 0U;
    g_rm_ui_init_tx.figure_cursor = 0U;
    g_rm_ui_init_tx.string_cursor = 0U;

    rm_ui_apply_state_to_objects();
    rm_ui_switch_all_to_modify();

    /*
     * ADD redraw uses manual dirty chunks. Sync exported last-state after it,
     * then force one MODIFY pass if setters changed state during the redraw.
     */
    ui_sync_g();
    if (pending_flush) {
        if (!rm_ui_pending_update_exists()) {
            rm_ui_mark_dynamic_dirty();
        }
        g_rm_ui_flush_pending = 1U;
    }
}

static uint8_t rm_ui_service_init_redraw(void)
{
    uint8_t packets_this_tick = 0U;

    if (!g_rm_ui_init_tx.active) {
        return 0U;
    }

    rm_ui_apply_state_to_objects();
    rm_ui_set_all_operate_type(RM_UI_OPERATE_ADD);

    while (packets_this_tick < RM_UI_INIT_PACKETS_PER_TICK && rm_ui_tx_queue_has_room()) {
        if (g_rm_ui_init_tx.figure_cursor < (uint8_t)RM_UI_TOTAL_FIGURES) {
            rm_ui_mark_next_figure_packet();
        } else if (g_rm_ui_init_tx.string_cursor < (uint8_t)RM_UI_TOTAL_STRINGS) {
            rm_ui_mark_next_string_packet();
        } else {
            if (g_rm_ui_init_tx.rounds_left > 0U) {
                g_rm_ui_init_tx.rounds_left--;
            }

            if (g_rm_ui_init_tx.rounds_left == 0U) {
                rm_ui_finish_init_redraw();
                return 1U;
            }

            g_rm_ui_init_tx.figure_cursor = 0U;
            g_rm_ui_init_tx.string_cursor = 0U;
            return 1U;
        }

        ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string,
                         RM_UI_TOTAL_FIGURES, RM_UI_TOTAL_STRINGS);
        packets_this_tick++;
    }

    rm_ui_clear_dirty();
    return 1U;
}

static uint8_t rm_ui_mark_next_update_figure_packet(void)
{
    uint8_t count = 0U;

    rm_ui_clear_dirty();

    while (g_rm_ui_update_tx.figure_cursor < (uint8_t)RM_UI_TOTAL_FIGURES &&
           count < RM_UI_FIGURES_PER_PACKET) {
        if (g_rm_ui_update_figure[g_rm_ui_update_tx.figure_cursor] != 0U) {
            ui_g_dirty_figure[g_rm_ui_update_tx.figure_cursor] = 1U;
            count++;
        }
        g_rm_ui_update_tx.figure_cursor++;
    }

    return count > 0U ? 1U : 0U;
}

static uint8_t rm_ui_mark_next_update_string_packet(void)
{
    rm_ui_clear_dirty();

    while (g_rm_ui_update_tx.string_cursor < (uint8_t)RM_UI_TOTAL_STRINGS) {
        if (g_rm_ui_update_string[g_rm_ui_update_tx.string_cursor] != 0U) {
            ui_g_dirty_string[g_rm_ui_update_tx.string_cursor] = 1U;
            g_rm_ui_update_tx.string_cursor++;
            return 1U;
        }
        g_rm_ui_update_tx.string_cursor++;
    }

    return 0U;
}

static void rm_ui_finish_update_tx(void)
{
    memset(g_rm_ui_update_figure, 0, sizeof(g_rm_ui_update_figure));
    memset(g_rm_ui_update_string, 0, sizeof(g_rm_ui_update_string));
    memset(&g_rm_ui_update_tx, 0, sizeof(g_rm_ui_update_tx));

    ui_sync_g();
    if (rm_ui_pending_update_exists()) {
        rm_ui_start_update_tx();
    }
}

static uint8_t rm_ui_service_update_tx(void)
{
    uint8_t packets_this_tick = 0U;

    if (!g_rm_ui_update_tx.active) {
        return 0U;
    }

    rm_ui_apply_state_to_objects();
    rm_ui_switch_all_to_modify();

    while (packets_this_tick < RM_UI_INIT_PACKETS_PER_TICK && rm_ui_tx_queue_has_room()) {
        uint8_t marked = 0U;

        if (g_rm_ui_update_tx.figure_cursor < (uint8_t)RM_UI_TOTAL_FIGURES) {
            marked = rm_ui_mark_next_update_figure_packet();
            if (marked == 0U) {
                continue;
            }
        } else if (g_rm_ui_update_tx.string_cursor < (uint8_t)RM_UI_TOTAL_STRINGS) {
            marked = rm_ui_mark_next_update_string_packet();
            if (marked == 0U) {
                continue;
            }
        } else {
            rm_ui_finish_update_tx();
            return 1U;
        }

        ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string,
                         RM_UI_TOTAL_FIGURES, RM_UI_TOTAL_STRINGS);
        packets_this_tick++;
    }

    rm_ui_clear_dirty();
    return 1U;
}

void RM_UI_Init(const uint16_t self_id)
{
    ui_self_id = self_id;
    ui_prepare_g();

    rm_ui_set_default_state();
    rm_ui_apply_state_to_objects();
    rm_ui_clear_dirty();
    rm_ui_reset_update_tx();
    rm_ui_start_init_redraw(RM_UI_INIT_REDRAW_ROUNDS);
    g_rm_ui_last_flush_ms = 0U;
}

void RM_UI_RequestFullRefresh(void)
{
    rm_ui_start_init_redraw(RM_UI_INIT_REDRAW_ROUNDS);
}

void RM_UI_RefreshStatic(void)
{
    RM_UI_RequestFullRefresh();
}

void RM_UI_RequestStaticRefresh(void)
{
    RM_UI_RequestFullRefresh();
}

void RM_UI_Flush(void)
{
    if (g_rm_ui_init_tx.active) {
        return;
    }

    if (!g_rm_ui_update_tx.active) {
        if (!rm_ui_pending_update_exists()) {
            return;
        }
        rm_ui_start_update_tx();
    }

    (void)rm_ui_service_update_tx();
    ui_tx_service();
}

void RM_UI_ServiceTx(void)
{
    ui_tx_service();
}

void RM_UI_OnTxComplete(void)
{
    ui_tx_on_dma_complete();
    ui_tx_service();
}

void RM_UI_RequestFlush(void)
{
    rm_ui_mark_dynamic_dirty();
}

void RM_UI_Task(const rm_ui_poll_t *context)
{
    uint16_t period_ms = 50U;

    RM_UI_ServiceTx();

    if (context == NULL) {
        return;
    }

    if (context->period_ms > 0U) {
        period_ms = context->period_ms;
    }

    if ((uint32_t)(context->now_ms - g_rm_ui_last_flush_ms) < period_ms) {
        return;
    }

    if (rm_ui_service_init_redraw()) {
        RM_UI_ServiceTx();
        g_rm_ui_last_flush_ms = context->now_ms;
        return;
    }

    if (g_rm_ui_update_tx.active || rm_ui_pending_update_exists() || g_rm_ui_flush_pending) {
        if (!g_rm_ui_update_tx.active) {
            if (!rm_ui_pending_update_exists()) {
                rm_ui_mark_dynamic_dirty();
            }
            rm_ui_start_update_tx();
        }

        (void)rm_ui_service_update_tx();
        RM_UI_ServiceTx();
        g_rm_ui_last_flush_ms = context->now_ms;
    }
}

void RM_UI_Poll(const rm_ui_poll_t *context)
{
    RM_UI_Task(context);
}

void RM_UI_SetLift(const float rf_percent, const float lf_percent, const float rb_percent, const float lb_percent)
{
    g_rm_ui_state.uplift_rf_percent = rm_ui_clamp_percent_0_100(rf_percent);
    g_rm_ui_state.uplift_lf_percent = rm_ui_clamp_percent_0_100(lf_percent);
    g_rm_ui_state.uplift_rb_percent = rm_ui_clamp_percent_0_100(rb_percent);
    g_rm_ui_state.uplift_lb_percent = rm_ui_clamp_percent_0_100(lb_percent);
    rm_ui_mark_pending_update(g_rm_ui_lift_figures,
                              sizeof(g_rm_ui_lift_figures) / sizeof(g_rm_ui_lift_figures[0]),
                              NULL, 0U);
}

void RM_UI_SetPower(const float power_percent)
{
    g_rm_ui_state.power_percent = rm_ui_clamp_min_zero(power_percent);
    rm_ui_mark_pending_update(g_rm_ui_power_figures,
                              sizeof(g_rm_ui_power_figures) / sizeof(g_rm_ui_power_figures[0]),
                              NULL, 0U);
}

void RM_UI_SetOrientation(const bool forehead)
{
    g_rm_ui_state.orientation_forehead = forehead;
    rm_ui_mark_pending_update(NULL, 0U,
                              g_rm_ui_orientation_strings,
                              sizeof(g_rm_ui_orientation_strings) / sizeof(g_rm_ui_orientation_strings[0]));
}

void RM_UI_SetWheel(const rm_ui_wheel_t status)
{
    g_rm_ui_state.wheel = rm_ui_sanitize_wheel(status);
    rm_ui_mark_pending_update(NULL, 0U,
                              g_rm_ui_wheel_strings,
                              sizeof(g_rm_ui_wheel_strings) / sizeof(g_rm_ui_wheel_strings[0]));
}

void RM_UI_SetSpeed(const rm_ui_speed_t mode)
{
    g_rm_ui_state.speed = rm_ui_sanitize_speed(mode);
    rm_ui_mark_pending_update(g_rm_ui_speed_figures,
                              sizeof(g_rm_ui_speed_figures) / sizeof(g_rm_ui_speed_figures[0]),
                              NULL, 0U);
}

void RM_UI_SetMode(const rm_ui_mode_t mode)
{
    g_rm_ui_state.mode = rm_ui_sanitize_mode(mode);
    rm_ui_mark_pending_update(g_rm_ui_mode_figures,
                              sizeof(g_rm_ui_mode_figures) / sizeof(g_rm_ui_mode_figures[0]),
                              g_rm_ui_mode_strings,
                              sizeof(g_rm_ui_mode_strings) / sizeof(g_rm_ui_mode_strings[0]));
}

void RM_UI_SetStage(const uint8_t stage)
{
    g_rm_ui_state.stage = stage > 9U ? 9U : stage;
    rm_ui_mark_pending_update(g_rm_ui_stage_figures,
                              sizeof(g_rm_ui_stage_figures) / sizeof(g_rm_ui_stage_figures[0]),
                              g_rm_ui_stage_strings,
                              sizeof(g_rm_ui_stage_strings) / sizeof(g_rm_ui_stage_strings[0]));
}

void RM_UI_SetGripper(const rm_ui_gripper_t status)
{
    g_rm_ui_state.gripper = rm_ui_sanitize_gripper(status);
    rm_ui_mark_pending_update(NULL, 0U,
                              g_rm_ui_gripper_strings,
                              sizeof(g_rm_ui_gripper_strings) / sizeof(g_rm_ui_gripper_strings[0]));
}

void RM_UI_SetInput(const rm_ui_input_t type)
{
    g_rm_ui_state.input = rm_ui_sanitize_input(type);
}

bool RM_UI_SyncUnpack(const uint8_t data[RM_UI_SYNC_DLC], rm_ui_sync_t *out)
{
    rm_ui_sync_t decoded_state;

    if (data == NULL || out == NULL) {
        return false;
    }

    decoded_state.speed = (rm_ui_speed_t)data[RM_UI_SYNC_IDX_SPEED];
    decoded_state.mode = (rm_ui_mode_t)data[RM_UI_SYNC_IDX_MODE];
    decoded_state.stage = data[RM_UI_SYNC_IDX_STAGE];
    decoded_state.gripper = (rm_ui_gripper_t)data[RM_UI_SYNC_IDX_GRIPPER];
    decoded_state.input = (rm_ui_input_t)data[RM_UI_SYNC_IDX_INPUT];
    decoded_state.flags = data[RM_UI_SYNC_IDX_FLAGS];

    if (!rm_ui_remote_state_is_valid(&decoded_state)) {
        return false;
    }

    *out = decoded_state;
    return true;
}

void RM_UI_SyncApply(const rm_ui_sync_t *state)
{
    if (state == NULL) {
        return;
    }

    g_rm_ui_state.speed = rm_ui_sanitize_speed(state->speed);
    g_rm_ui_state.mode = rm_ui_sanitize_mode(state->mode);
    g_rm_ui_state.stage = state->stage > 9U ? 9U : state->stage;
    g_rm_ui_state.gripper = rm_ui_sanitize_gripper(state->gripper);
    g_rm_ui_state.input = rm_ui_sanitize_input(state->input);
    rm_ui_mark_pending_update(g_rm_ui_remote_figures,
                              sizeof(g_rm_ui_remote_figures) / sizeof(g_rm_ui_remote_figures[0]),
                              g_rm_ui_remote_strings,
                              sizeof(g_rm_ui_remote_strings) / sizeof(g_rm_ui_remote_strings[0]));

    if ((state->flags & RM_UI_SYNC_FLAG_FULL_REFRESH) != 0U) {
        RM_UI_RequestFullRefresh();
    }
}

#endif
