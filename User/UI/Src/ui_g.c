//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 17
#define TOTAL_STRING 12

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

static void ui_setup_g(void) {
    ui_g_Lift_Rect_Uplift_RF->figure_type = 1;
    ui_g_Lift_Rect_Uplift_RF->operate_type = 1;
    ui_g_Lift_Rect_Uplift_RF->layer = 0;
    ui_g_Lift_Rect_Uplift_RF->color = 8;
    ui_g_Lift_Rect_Uplift_RF->start_x = 291;
    ui_g_Lift_Rect_Uplift_RF->start_y = 171;
    ui_g_Lift_Rect_Uplift_RF->width = 3;
    ui_g_Lift_Rect_Uplift_RF->end_x = 319;
    ui_g_Lift_Rect_Uplift_RF->end_y = 319;

    ui_g_Lift_Rect_Uplift_LB->figure_type = 1;
    ui_g_Lift_Rect_Uplift_LB->operate_type = 1;
    ui_g_Lift_Rect_Uplift_LB->layer = 0;
    ui_g_Lift_Rect_Uplift_LB->color = 8;
    ui_g_Lift_Rect_Uplift_LB->start_x = 356;
    ui_g_Lift_Rect_Uplift_LB->start_y = 171;
    ui_g_Lift_Rect_Uplift_LB->width = 3;
    ui_g_Lift_Rect_Uplift_LB->end_x = 384;
    ui_g_Lift_Rect_Uplift_LB->end_y = 319;

    ui_g_Left_Rect_Uplift_RB->figure_type = 1;
    ui_g_Left_Rect_Uplift_RB->operate_type = 1;
    ui_g_Left_Rect_Uplift_RB->layer = 0;
    ui_g_Left_Rect_Uplift_RB->color = 8;
    ui_g_Left_Rect_Uplift_RB->start_x = 414;
    ui_g_Left_Rect_Uplift_RB->start_y = 171;
    ui_g_Left_Rect_Uplift_RB->width = 3;
    ui_g_Left_Rect_Uplift_RB->end_x = 442;
    ui_g_Left_Rect_Uplift_RB->end_y = 319;

    ui_g_RobotStatus_PowerBar->figure_type = 1;
    ui_g_RobotStatus_PowerBar->operate_type = 1;
    ui_g_RobotStatus_PowerBar->layer = 0;
    ui_g_RobotStatus_PowerBar->color = 8;
    ui_g_RobotStatus_PowerBar->start_x = 602;
    ui_g_RobotStatus_PowerBar->start_y = 90;
    ui_g_RobotStatus_PowerBar->width = 3;
    ui_g_RobotStatus_PowerBar->end_x = 1329;
    ui_g_RobotStatus_PowerBar->end_y = 140;

    ui_g_RobotStatus_SpeedCircle->figure_type = 2;
    ui_g_RobotStatus_SpeedCircle->operate_type = 1;
    ui_g_RobotStatus_SpeedCircle->layer = 0;
    ui_g_RobotStatus_SpeedCircle->color = 2;
    ui_g_RobotStatus_SpeedCircle->start_x = 958;
    ui_g_RobotStatus_SpeedCircle->start_y = 539;
    ui_g_RobotStatus_SpeedCircle->width = 3;
    ui_g_RobotStatus_SpeedCircle->r = 80;

    ui_g_Lift_Rect_Uplift_LF->figure_type = 1;
    ui_g_Lift_Rect_Uplift_LF->operate_type = 1;
    ui_g_Lift_Rect_Uplift_LF->layer = 0;
    ui_g_Lift_Rect_Uplift_LF->color = 8;
    ui_g_Lift_Rect_Uplift_LF->start_x = 233;
    ui_g_Lift_Rect_Uplift_LF->start_y = 171;
    ui_g_Lift_Rect_Uplift_LF->width = 3;
    ui_g_Lift_Rect_Uplift_LF->end_x = 261;
    ui_g_Lift_Rect_Uplift_LF->end_y = 319;

    ui_g_RobotStatus_Now_Power->figure_type = 1;
    ui_g_RobotStatus_Now_Power->operate_type = 1;
    ui_g_RobotStatus_Now_Power->layer = 0;
    ui_g_RobotStatus_Now_Power->color = 2;
    ui_g_RobotStatus_Now_Power->start_x = 606;
    ui_g_RobotStatus_Now_Power->start_y = 94;
    ui_g_RobotStatus_Now_Power->width = 40;
    ui_g_RobotStatus_Now_Power->end_x = 1286;
    ui_g_RobotStatus_Now_Power->end_y = 100;

    ui_g_RobotStatus_Now_RF->figure_type = 1;
    ui_g_RobotStatus_Now_RF->operate_type = 1;
    ui_g_RobotStatus_Now_RF->layer = 0;
    ui_g_RobotStatus_Now_RF->color = 3;
    ui_g_RobotStatus_Now_RF->start_x = 235;
    ui_g_RobotStatus_Now_RF->start_y = 162;
    ui_g_RobotStatus_Now_RF->width = 25;
    ui_g_RobotStatus_Now_RF->end_x = 235;
    ui_g_RobotStatus_Now_RF->end_y = 307;

    ui_g_RobotStatus_Now_LF->figure_type = 1;
    ui_g_RobotStatus_Now_LF->operate_type = 1;
    ui_g_RobotStatus_Now_LF->layer = 0;
    ui_g_RobotStatus_Now_LF->color = 3;
    ui_g_RobotStatus_Now_LF->start_x = 294;
    ui_g_RobotStatus_Now_LF->start_y = 162;
    ui_g_RobotStatus_Now_LF->width = 25;
    ui_g_RobotStatus_Now_LF->end_x = 294;
    ui_g_RobotStatus_Now_LF->end_y = 307;

    ui_g_RobotStatus_Now_RB->figure_type = 1;
    ui_g_RobotStatus_Now_RB->operate_type = 1;
    ui_g_RobotStatus_Now_RB->layer = 0;
    ui_g_RobotStatus_Now_RB->color = 3;
    ui_g_RobotStatus_Now_RB->start_x = 359;
    ui_g_RobotStatus_Now_RB->start_y = 162;
    ui_g_RobotStatus_Now_RB->width = 25;
    ui_g_RobotStatus_Now_RB->end_x = 359;
    ui_g_RobotStatus_Now_RB->end_y = 307;

    ui_g_RobotStatus_Now_LB->figure_type = 1;
    ui_g_RobotStatus_Now_LB->operate_type = 1;
    ui_g_RobotStatus_Now_LB->layer = 0;
    ui_g_RobotStatus_Now_LB->color = 3;
    ui_g_RobotStatus_Now_LB->start_x = 418;
    ui_g_RobotStatus_Now_LB->start_y = 162;
    ui_g_RobotStatus_Now_LB->width = 25;
    ui_g_RobotStatus_Now_LB->end_x = 418;
    ui_g_RobotStatus_Now_LB->end_y = 307;

    ui_g_RunningRect_WoringRect->figure_type = 1;
    ui_g_RunningRect_WoringRect->operate_type = 1;
    ui_g_RunningRect_WoringRect->layer = 0;
    ui_g_RunningRect_WoringRect->color = 3;
    ui_g_RunningRect_WoringRect->start_x = 793;
    ui_g_RunningRect_WoringRect->start_y = 253;
    ui_g_RunningRect_WoringRect->width = 3;
    ui_g_RunningRect_WoringRect->end_x = 843;
    ui_g_RunningRect_WoringRect->end_y = 303;

    ui_g_RunningRect_MovingRect->figure_type = 1;
    ui_g_RunningRect_MovingRect->operate_type = 1;
    ui_g_RunningRect_MovingRect->layer = 0;
    ui_g_RunningRect_MovingRect->color = 3;
    ui_g_RunningRect_MovingRect->start_x = 866;
    ui_g_RunningRect_MovingRect->start_y = 252;
    ui_g_RunningRect_MovingRect->width = 3;
    ui_g_RunningRect_MovingRect->end_x = 916;
    ui_g_RunningRect_MovingRect->end_y = 302;

    ui_g_RunningRect_UpliftRect->figure_type = 1;
    ui_g_RunningRect_UpliftRect->operate_type = 1;
    ui_g_RunningRect_UpliftRect->layer = 0;
    ui_g_RunningRect_UpliftRect->color = 3;
    ui_g_RunningRect_UpliftRect->start_x = 936;
    ui_g_RunningRect_UpliftRect->start_y = 252;
    ui_g_RunningRect_UpliftRect->width = 3;
    ui_g_RunningRect_UpliftRect->end_x = 986;
    ui_g_RunningRect_UpliftRect->end_y = 302;

    ui_g_RunningRect_DownliftRect->figure_type = 1;
    ui_g_RunningRect_DownliftRect->operate_type = 1;
    ui_g_RunningRect_DownliftRect->layer = 0;
    ui_g_RunningRect_DownliftRect->color = 3;
    ui_g_RunningRect_DownliftRect->start_x = 1004;
    ui_g_RunningRect_DownliftRect->start_y = 252;
    ui_g_RunningRect_DownliftRect->width = 3;
    ui_g_RunningRect_DownliftRect->end_x = 1054;
    ui_g_RunningRect_DownliftRect->end_y = 302;

    ui_g_RunningRect_SaveloadRect->figure_type = 1;
    ui_g_RunningRect_SaveloadRect->operate_type = 1;
    ui_g_RunningRect_SaveloadRect->layer = 0;
    ui_g_RunningRect_SaveloadRect->color = 3;
    ui_g_RunningRect_SaveloadRect->start_x = 1072;
    ui_g_RunningRect_SaveloadRect->start_y = 252;
    ui_g_RunningRect_SaveloadRect->width = 3;
    ui_g_RunningRect_SaveloadRect->end_x = 1122;
    ui_g_RunningRect_SaveloadRect->end_y = 302;

    ui_g_Now_FSM_FSM_Number->figure_type = 6;
    ui_g_Now_FSM_FSM_Number->operate_type = 1;
    ui_g_Now_FSM_FSM_Number->layer = 0;
    ui_g_Now_FSM_FSM_Number->color = 1;
    ui_g_Now_FSM_FSM_Number->start_x = 1004;
    ui_g_Now_FSM_FSM_Number->start_y = 218;
    ui_g_Now_FSM_FSM_Number->width = 2;
    ui_g_Now_FSM_FSM_Number->font_size = 20;
    ui_g_Now_FSM_FSM_Number->number = 0;

    ui_g_RobotStatus_Orientation->figure_type = 7;
    ui_g_RobotStatus_Orientation->operate_type = 1;
    ui_g_RobotStatus_Orientation->layer = 0;
    ui_g_RobotStatus_Orientation->color = 2;
    ui_g_RobotStatus_Orientation->start_x = 876;
    ui_g_RobotStatus_Orientation->start_y = 882;
    ui_g_RobotStatus_Orientation->width = 2;
    ui_g_RobotStatus_Orientation->font_size = 20;
    ui_g_RobotStatus_Orientation->str_length = 8;
    strcpy(ui_g_RobotStatus_Orientation->string, "FOREHEAD");

    ui_g_RobotStatus_none->figure_type = 7;
    ui_g_RobotStatus_none->operate_type = 1;
    ui_g_RobotStatus_none->layer = 0;
    ui_g_RobotStatus_none->color = 1;
    ui_g_RobotStatus_none->start_x = 812;
    ui_g_RobotStatus_none->start_y = 882;
    ui_g_RobotStatus_none->width = 2;
    ui_g_RobotStatus_none->font_size = 20;
    ui_g_RobotStatus_none->str_length = 0;
    strcpy(ui_g_RobotStatus_none->string, "");

    ui_g_RobotStatus_FSM_Status->figure_type = 7;
    ui_g_RobotStatus_FSM_Status->operate_type = 1;
    ui_g_RobotStatus_FSM_Status->layer = 0;
    ui_g_RobotStatus_FSM_Status->color = 1;
    ui_g_RobotStatus_FSM_Status->start_x = 895;
    ui_g_RobotStatus_FSM_Status->start_y = 218;
    ui_g_RobotStatus_FSM_Status->width = 2;
    ui_g_RobotStatus_FSM_Status->font_size = 20;
    ui_g_RobotStatus_FSM_Status->str_length = 7;
    strcpy(ui_g_RobotStatus_FSM_Status->string, "STAGE  ");

    ui_g_Tips_Wheels->figure_type = 7;
    ui_g_Tips_Wheels->operate_type = 1;
    ui_g_Tips_Wheels->layer = 0;
    ui_g_Tips_Wheels->color = 6;
    ui_g_Tips_Wheels->start_x = 1456;
    ui_g_Tips_Wheels->start_y = 343;
    ui_g_Tips_Wheels->width = 3;
    ui_g_Tips_Wheels->font_size = 25;
    ui_g_Tips_Wheels->str_length = 7;
    strcpy(ui_g_Tips_Wheels->string, "WHEELS:");

    ui_g_RobotStatus_Wheel_Status->figure_type = 7;
    ui_g_RobotStatus_Wheel_Status->operate_type = 1;
    ui_g_RobotStatus_Wheel_Status->layer = 0;
    ui_g_RobotStatus_Wheel_Status->color = 2;
    ui_g_RobotStatus_Wheel_Status->start_x = 1644;
    ui_g_RobotStatus_Wheel_Status->start_y = 343;
    ui_g_RobotStatus_Wheel_Status->width = 3;
    ui_g_RobotStatus_Wheel_Status->font_size = 25;
    ui_g_RobotStatus_Wheel_Status->str_length = 2;
    strcpy(ui_g_RobotStatus_Wheel_Status->string, "ON");

    ui_g_Tips_Gripper->figure_type = 7;
    ui_g_Tips_Gripper->operate_type = 1;
    ui_g_Tips_Gripper->layer = 0;
    ui_g_Tips_Gripper->color = 6;
    ui_g_Tips_Gripper->start_x = 1455;
    ui_g_Tips_Gripper->start_y = 401;
    ui_g_Tips_Gripper->width = 3;
    ui_g_Tips_Gripper->font_size = 25;
    ui_g_Tips_Gripper->str_length = 5;
    strcpy(ui_g_Tips_Gripper->string, "GRIP:");

    ui_g_RobotStatus_Gripper_Status->figure_type = 7;
    ui_g_RobotStatus_Gripper_Status->operate_type = 1;
    ui_g_RobotStatus_Gripper_Status->layer = 0;
    ui_g_RobotStatus_Gripper_Status->color = 2;
    ui_g_RobotStatus_Gripper_Status->start_x = 1608;
    ui_g_RobotStatus_Gripper_Status->start_y = 401;
    ui_g_RobotStatus_Gripper_Status->width = 3;
    ui_g_RobotStatus_Gripper_Status->font_size = 25;
    ui_g_RobotStatus_Gripper_Status->str_length = 5;
    strcpy(ui_g_RobotStatus_Gripper_Status->string, "CLOSE");

    ui_g_RunningMode_WoringMode->figure_type = 7;
    ui_g_RunningMode_WoringMode->operate_type = 1;
    ui_g_RunningMode_WoringMode->layer = 0;
    ui_g_RunningMode_WoringMode->color = 3;
    ui_g_RunningMode_WoringMode->start_x = 803;
    ui_g_RunningMode_WoringMode->start_y = 292;
    ui_g_RunningMode_WoringMode->width = 3;
    ui_g_RunningMode_WoringMode->font_size = 30;
    ui_g_RunningMode_WoringMode->str_length = 1;
    strcpy(ui_g_RunningMode_WoringMode->string, "Z");

    ui_g_RunningMode_MovingMode->figure_type = 7;
    ui_g_RunningMode_MovingMode->operate_type = 1;
    ui_g_RunningMode_MovingMode->layer = 0;
    ui_g_RunningMode_MovingMode->color = 3;
    ui_g_RunningMode_MovingMode->start_x = 876;
    ui_g_RunningMode_MovingMode->start_y = 292;
    ui_g_RunningMode_MovingMode->width = 3;
    ui_g_RunningMode_MovingMode->font_size = 30;
    ui_g_RunningMode_MovingMode->str_length = 1;
    strcpy(ui_g_RunningMode_MovingMode->string, "P");

    ui_g_RunningMode_UpliftMode->figure_type = 7;
    ui_g_RunningMode_UpliftMode->operate_type = 1;
    ui_g_RunningMode_UpliftMode->layer = 0;
    ui_g_RunningMode_UpliftMode->color = 3;
    ui_g_RunningMode_UpliftMode->start_x = 946;
    ui_g_RunningMode_UpliftMode->start_y = 293;
    ui_g_RunningMode_UpliftMode->width = 3;
    ui_g_RunningMode_UpliftMode->font_size = 30;
    ui_g_RunningMode_UpliftMode->str_length = 1;
    strcpy(ui_g_RunningMode_UpliftMode->string, "S");

    ui_g_RunningMode_DownliftMode->figure_type = 7;
    ui_g_RunningMode_DownliftMode->operate_type = 1;
    ui_g_RunningMode_DownliftMode->layer = 0;
    ui_g_RunningMode_DownliftMode->color = 3;
    ui_g_RunningMode_DownliftMode->start_x = 1014;
    ui_g_RunningMode_DownliftMode->start_y = 293;
    ui_g_RunningMode_DownliftMode->width = 3;
    ui_g_RunningMode_DownliftMode->font_size = 30;
    ui_g_RunningMode_DownliftMode->str_length = 1;
    strcpy(ui_g_RunningMode_DownliftMode->string, "X");

    ui_g_RunningMode_SaveloadMode->figure_type = 7;
    ui_g_RunningMode_SaveloadMode->operate_type = 1;
    ui_g_RunningMode_SaveloadMode->layer = 0;
    ui_g_RunningMode_SaveloadMode->color = 3;
    ui_g_RunningMode_SaveloadMode->start_x = 1082;
    ui_g_RunningMode_SaveloadMode->start_y = 293;
    ui_g_RunningMode_SaveloadMode->width = 3;
    ui_g_RunningMode_SaveloadMode->font_size = 30;
    ui_g_RunningMode_SaveloadMode->str_length = 1;
    strcpy(ui_g_RunningMode_SaveloadMode->string, "Q");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

}

void ui_prepare_g(void) {
    ui_setup_g();
}

void ui_sync_g(void) {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_last_figures[i] = ui_g_now_figures[i];
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_last_strings[i] = ui_g_now_strings[i];
    }
#endif
    memset(ui_g_dirty_figure, 0, sizeof(ui_g_dirty_figure));
    memset(ui_g_dirty_string, 0, sizeof(ui_g_dirty_string));
}

void ui_init_g() {
    ui_setup_g();
    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }

    ui_sync_g();
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
