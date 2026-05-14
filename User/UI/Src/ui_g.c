//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 33
#define TOTAL_STRING 9

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
    ui_g_Lift_Rect_Uplift_RF->start_x = 290;
    ui_g_Lift_Rect_Uplift_RF->start_y = 247;
    ui_g_Lift_Rect_Uplift_RF->width = 3;
    ui_g_Lift_Rect_Uplift_RF->end_x = 318;
    ui_g_Lift_Rect_Uplift_RF->end_y = 395;

    ui_g_Lift_Rect_Uplift_LB->figure_type = 1;
    ui_g_Lift_Rect_Uplift_LB->operate_type = 1;
    ui_g_Lift_Rect_Uplift_LB->layer = 0;
    ui_g_Lift_Rect_Uplift_LB->color = 8;
    ui_g_Lift_Rect_Uplift_LB->start_x = 347;
    ui_g_Lift_Rect_Uplift_LB->start_y = 247;
    ui_g_Lift_Rect_Uplift_LB->width = 3;
    ui_g_Lift_Rect_Uplift_LB->end_x = 375;
    ui_g_Lift_Rect_Uplift_LB->end_y = 395;

    ui_g_Left_Rect_Uplift_RB->figure_type = 1;
    ui_g_Left_Rect_Uplift_RB->operate_type = 1;
    ui_g_Left_Rect_Uplift_RB->layer = 0;
    ui_g_Left_Rect_Uplift_RB->color = 8;
    ui_g_Left_Rect_Uplift_RB->start_x = 401;
    ui_g_Left_Rect_Uplift_RB->start_y = 247;
    ui_g_Left_Rect_Uplift_RB->width = 3;
    ui_g_Left_Rect_Uplift_RB->end_x = 429;
    ui_g_Left_Rect_Uplift_RB->end_y = 395;

    ui_g_RobotStatus_PowerBar->figure_type = 1;
    ui_g_RobotStatus_PowerBar->operate_type = 1;
    ui_g_RobotStatus_PowerBar->layer = 0;
    ui_g_RobotStatus_PowerBar->color = 8;
    ui_g_RobotStatus_PowerBar->start_x = 602;
    ui_g_RobotStatus_PowerBar->start_y = 98;
    ui_g_RobotStatus_PowerBar->width = 3;
    ui_g_RobotStatus_PowerBar->end_x = 1329;
    ui_g_RobotStatus_PowerBar->end_y = 148;

    ui_g_Joint_Status_J1Min->figure_type = 0;
    ui_g_Joint_Status_J1Min->operate_type = 1;
    ui_g_Joint_Status_J1Min->layer = 0;
    ui_g_Joint_Status_J1Min->color = 8;
    ui_g_Joint_Status_J1Min->start_x = 1799;
    ui_g_Joint_Status_J1Min->start_y = 651;
    ui_g_Joint_Status_J1Min->width = 3;
    ui_g_Joint_Status_J1Min->end_x = 1816;
    ui_g_Joint_Status_J1Min->end_y = 668;

    ui_g_Joint_Status_J1Max->figure_type = 0;
    ui_g_Joint_Status_J1Max->operate_type = 1;
    ui_g_Joint_Status_J1Max->layer = 0;
    ui_g_Joint_Status_J1Max->color = 8;
    ui_g_Joint_Status_J1Max->start_x = 1800;
    ui_g_Joint_Status_J1Max->start_y = 651;
    ui_g_Joint_Status_J1Max->width = 3;
    ui_g_Joint_Status_J1Max->end_x = 1772;
    ui_g_Joint_Status_J1Max->end_y = 651;

    ui_g_Joint_Status_J0Min->figure_type = 0;
    ui_g_Joint_Status_J0Min->operate_type = 1;
    ui_g_Joint_Status_J0Min->layer = 0;
    ui_g_Joint_Status_J0Min->color = 8;
    ui_g_Joint_Status_J0Min->start_x = 1799;
    ui_g_Joint_Status_J0Min->start_y = 717;
    ui_g_Joint_Status_J0Min->width = 3;
    ui_g_Joint_Status_J0Min->end_x = 1816;
    ui_g_Joint_Status_J0Min->end_y = 700;

    ui_g_RobotStatus_SpeedCircle->figure_type = 2;
    ui_g_RobotStatus_SpeedCircle->operate_type = 1;
    ui_g_RobotStatus_SpeedCircle->layer = 0;
    ui_g_RobotStatus_SpeedCircle->color = 2;
    ui_g_RobotStatus_SpeedCircle->start_x = 958;
    ui_g_RobotStatus_SpeedCircle->start_y = 539;
    ui_g_RobotStatus_SpeedCircle->width = 3;
    ui_g_RobotStatus_SpeedCircle->r = 80;

    ui_g_Joint_Status_J0Max->figure_type = 0;
    ui_g_Joint_Status_J0Max->operate_type = 1;
    ui_g_Joint_Status_J0Max->layer = 0;
    ui_g_Joint_Status_J0Max->color = 8;
    ui_g_Joint_Status_J0Max->start_x = 1799;
    ui_g_Joint_Status_J0Max->start_y = 717;
    ui_g_Joint_Status_J0Max->width = 3;
    ui_g_Joint_Status_J0Max->end_x = 1822;
    ui_g_Joint_Status_J0Max->end_y = 725;

    ui_g_Joint_Status_J2Min->figure_type = 0;
    ui_g_Joint_Status_J2Min->operate_type = 1;
    ui_g_Joint_Status_J2Min->layer = 0;
    ui_g_Joint_Status_J2Min->color = 8;
    ui_g_Joint_Status_J2Min->start_x = 1799;
    ui_g_Joint_Status_J2Min->start_y = 584;
    ui_g_Joint_Status_J2Min->width = 3;
    ui_g_Joint_Status_J2Min->end_x = 1820;
    ui_g_Joint_Status_J2Min->end_y = 576;

    ui_g_Joint_Status_J2Max->figure_type = 0;
    ui_g_Joint_Status_J2Max->operate_type = 1;
    ui_g_Joint_Status_J2Max->layer = 0;
    ui_g_Joint_Status_J2Max->color = 8;
    ui_g_Joint_Status_J2Max->start_x = 1797;
    ui_g_Joint_Status_J2Max->start_y = 584;
    ui_g_Joint_Status_J2Max->width = 3;
    ui_g_Joint_Status_J2Max->end_x = 1774;
    ui_g_Joint_Status_J2Max->end_y = 576;

    ui_g_Joint_Status_J3Max->figure_type = 0;
    ui_g_Joint_Status_J3Max->operate_type = 1;
    ui_g_Joint_Status_J3Max->layer = 0;
    ui_g_Joint_Status_J3Max->color = 8;
    ui_g_Joint_Status_J3Max->start_x = 1798;
    ui_g_Joint_Status_J3Max->start_y = 516;
    ui_g_Joint_Status_J3Max->width = 3;
    ui_g_Joint_Status_J3Max->end_x = 1802;
    ui_g_Joint_Status_J3Max->end_y = 491;

    ui_g_Ungroup_J3Min->figure_type = 0;
    ui_g_Ungroup_J3Min->operate_type = 1;
    ui_g_Ungroup_J3Min->layer = 0;
    ui_g_Ungroup_J3Min->color = 8;
    ui_g_Ungroup_J3Min->start_x = 1797;
    ui_g_Ungroup_J3Min->start_y = 516;
    ui_g_Ungroup_J3Min->width = 3;
    ui_g_Ungroup_J3Min->end_x = 1793;
    ui_g_Ungroup_J3Min->end_y = 491;

    ui_g_Joint_Status_J4Min->figure_type = 0;
    ui_g_Joint_Status_J4Min->operate_type = 1;
    ui_g_Joint_Status_J4Min->layer = 0;
    ui_g_Joint_Status_J4Min->color = 8;
    ui_g_Joint_Status_J4Min->start_x = 1798;
    ui_g_Joint_Status_J4Min->start_y = 445;
    ui_g_Joint_Status_J4Min->width = 3;
    ui_g_Joint_Status_J4Min->end_x = 1798;
    ui_g_Joint_Status_J4Min->end_y = 422;

    ui_g_Joint_Status_J4Max->figure_type = 0;
    ui_g_Joint_Status_J4Max->operate_type = 1;
    ui_g_Joint_Status_J4Max->layer = 0;
    ui_g_Joint_Status_J4Max->color = 8;
    ui_g_Joint_Status_J4Max->start_x = 1798;
    ui_g_Joint_Status_J4Max->start_y = 445;
    ui_g_Joint_Status_J4Max->width = 3;
    ui_g_Joint_Status_J4Max->end_x = 1798;
    ui_g_Joint_Status_J4Max->end_y = 473;

    ui_g_Joint_Now_J0_Now->figure_type = 0;
    ui_g_Joint_Now_J0_Now->operate_type = 1;
    ui_g_Joint_Now_J0_Now->layer = 0;
    ui_g_Joint_Now_J0_Now->color = 6;
    ui_g_Joint_Now_J0_Now->start_x = 1798;
    ui_g_Joint_Now_J0_Now->start_y = 717;
    ui_g_Joint_Now_J0_Now->width = 3;
    ui_g_Joint_Now_J0_Now->end_x = 1823;
    ui_g_Joint_Now_J0_Now->end_y = 717;

    ui_g_Joint_Circle_J1->figure_type = 2;
    ui_g_Joint_Circle_J1->operate_type = 1;
    ui_g_Joint_Circle_J1->layer = 0;
    ui_g_Joint_Circle_J1->color = 8;
    ui_g_Joint_Circle_J1->start_x = 1797;
    ui_g_Joint_Circle_J1->start_y = 651;
    ui_g_Joint_Circle_J1->width = 3;
    ui_g_Joint_Circle_J1->r = 25;

    ui_g_Joint_Now_J1_Now->figure_type = 0;
    ui_g_Joint_Now_J1_Now->operate_type = 1;
    ui_g_Joint_Now_J1_Now->layer = 0;
    ui_g_Joint_Now_J1_Now->color = 6;
    ui_g_Joint_Now_J1_Now->start_x = 1789;
    ui_g_Joint_Now_J1_Now->start_y = 674;
    ui_g_Joint_Now_J1_Now->width = 3;
    ui_g_Joint_Now_J1_Now->end_x = 1797;
    ui_g_Joint_Now_J1_Now->end_y = 651;

    ui_g_Joint_Now_J2_Now->figure_type = 0;
    ui_g_Joint_Now_J2_Now->operate_type = 1;
    ui_g_Joint_Now_J2_Now->layer = 0;
    ui_g_Joint_Now_J2_Now->color = 6;
    ui_g_Joint_Now_J2_Now->start_x = 1798;
    ui_g_Joint_Now_J2_Now->start_y = 585;
    ui_g_Joint_Now_J2_Now->width = 3;
    ui_g_Joint_Now_J2_Now->end_x = 1798;
    ui_g_Joint_Now_J2_Now->end_y = 610;

    ui_g_Joint_Now_J3_Now->figure_type = 0;
    ui_g_Joint_Now_J3_Now->operate_type = 1;
    ui_g_Joint_Now_J3_Now->layer = 0;
    ui_g_Joint_Now_J3_Now->color = 6;
    ui_g_Joint_Now_J3_Now->start_x = 1798;
    ui_g_Joint_Now_J3_Now->start_y = 518;
    ui_g_Joint_Now_J3_Now->width = 3;
    ui_g_Joint_Now_J3_Now->end_x = 1798;
    ui_g_Joint_Now_J3_Now->end_y = 543;

    ui_g_Joint_Now_J4_Now->figure_type = 0;
    ui_g_Joint_Now_J4_Now->operate_type = 1;
    ui_g_Joint_Now_J4_Now->layer = 0;
    ui_g_Joint_Now_J4_Now->color = 6;
    ui_g_Joint_Now_J4_Now->start_x = 1798;
    ui_g_Joint_Now_J4_Now->start_y = 447;
    ui_g_Joint_Now_J4_Now->width = 3;
    ui_g_Joint_Now_J4_Now->end_x = 1823;
    ui_g_Joint_Now_J4_Now->end_y = 447;

    ui_g_Joint_Now_J5_Now->figure_type = 0;
    ui_g_Joint_Now_J5_Now->operate_type = 1;
    ui_g_Joint_Now_J5_Now->layer = 0;
    ui_g_Joint_Now_J5_Now->color = 6;
    ui_g_Joint_Now_J5_Now->start_x = 1798;
    ui_g_Joint_Now_J5_Now->start_y = 380;
    ui_g_Joint_Now_J5_Now->width = 3;
    ui_g_Joint_Now_J5_Now->end_x = 1798;
    ui_g_Joint_Now_J5_Now->end_y = 405;

    ui_g_Lift_Rect_Uplift_LF->figure_type = 1;
    ui_g_Lift_Rect_Uplift_LF->operate_type = 1;
    ui_g_Lift_Rect_Uplift_LF->layer = 0;
    ui_g_Lift_Rect_Uplift_LF->color = 8;
    ui_g_Lift_Rect_Uplift_LF->start_x = 234;
    ui_g_Lift_Rect_Uplift_LF->start_y = 247;
    ui_g_Lift_Rect_Uplift_LF->width = 3;
    ui_g_Lift_Rect_Uplift_LF->end_x = 262;
    ui_g_Lift_Rect_Uplift_LF->end_y = 395;

    ui_g_RobotStatus_Now_Power->figure_type = 1;
    ui_g_RobotStatus_Now_Power->operate_type = 1;
    ui_g_RobotStatus_Now_Power->layer = 0;
    ui_g_RobotStatus_Now_Power->color = 2;
    ui_g_RobotStatus_Now_Power->start_x = 606;
    ui_g_RobotStatus_Now_Power->start_y = 102;
    ui_g_RobotStatus_Now_Power->width = 40;
    ui_g_RobotStatus_Now_Power->end_x = 1286;
    ui_g_RobotStatus_Now_Power->end_y = 108;

    ui_g_RobotStatus_Now_RF->figure_type = 1;
    ui_g_RobotStatus_Now_RF->operate_type = 1;
    ui_g_RobotStatus_Now_RF->layer = 0;
    ui_g_RobotStatus_Now_RF->color = 3;
    ui_g_RobotStatus_Now_RF->start_x = 238;
    ui_g_RobotStatus_Now_RF->start_y = 238;
    ui_g_RobotStatus_Now_RF->width = 25;
    ui_g_RobotStatus_Now_RF->end_x = 238;
    ui_g_RobotStatus_Now_RF->end_y = 383;

    ui_g_RobotStatus_Now_LF->figure_type = 1;
    ui_g_RobotStatus_Now_LF->operate_type = 1;
    ui_g_RobotStatus_Now_LF->layer = 0;
    ui_g_RobotStatus_Now_LF->color = 3;
    ui_g_RobotStatus_Now_LF->start_x = 292;
    ui_g_RobotStatus_Now_LF->start_y = 238;
    ui_g_RobotStatus_Now_LF->width = 25;
    ui_g_RobotStatus_Now_LF->end_x = 292;
    ui_g_RobotStatus_Now_LF->end_y = 383;

    ui_g_RobotStatus_Now_RB->figure_type = 1;
    ui_g_RobotStatus_Now_RB->operate_type = 1;
    ui_g_RobotStatus_Now_RB->layer = 0;
    ui_g_RobotStatus_Now_RB->color = 3;
    ui_g_RobotStatus_Now_RB->start_x = 350;
    ui_g_RobotStatus_Now_RB->start_y = 238;
    ui_g_RobotStatus_Now_RB->width = 25;
    ui_g_RobotStatus_Now_RB->end_x = 350;
    ui_g_RobotStatus_Now_RB->end_y = 383;

    ui_g_RobotStatus_Now_LB->figure_type = 1;
    ui_g_RobotStatus_Now_LB->operate_type = 1;
    ui_g_RobotStatus_Now_LB->layer = 0;
    ui_g_RobotStatus_Now_LB->color = 3;
    ui_g_RobotStatus_Now_LB->start_x = 405;
    ui_g_RobotStatus_Now_LB->start_y = 238;
    ui_g_RobotStatus_Now_LB->width = 25;
    ui_g_RobotStatus_Now_LB->end_x = 405;
    ui_g_RobotStatus_Now_LB->end_y = 383;

    ui_g_Joint_Circle_J0->figure_type = 2;
    ui_g_Joint_Circle_J0->operate_type = 1;
    ui_g_Joint_Circle_J0->layer = 0;
    ui_g_Joint_Circle_J0->color = 8;
    ui_g_Joint_Circle_J0->start_x = 1797;
    ui_g_Joint_Circle_J0->start_y = 717;
    ui_g_Joint_Circle_J0->width = 3;
    ui_g_Joint_Circle_J0->r = 25;

    ui_g_Joint_Circle_J2->figure_type = 2;
    ui_g_Joint_Circle_J2->operate_type = 1;
    ui_g_Joint_Circle_J2->layer = 0;
    ui_g_Joint_Circle_J2->color = 8;
    ui_g_Joint_Circle_J2->start_x = 1797;
    ui_g_Joint_Circle_J2->start_y = 585;
    ui_g_Joint_Circle_J2->width = 3;
    ui_g_Joint_Circle_J2->r = 25;

    ui_g_Joint_Circle_J3->figure_type = 2;
    ui_g_Joint_Circle_J3->operate_type = 1;
    ui_g_Joint_Circle_J3->layer = 0;
    ui_g_Joint_Circle_J3->color = 8;
    ui_g_Joint_Circle_J3->start_x = 1797;
    ui_g_Joint_Circle_J3->start_y = 516;
    ui_g_Joint_Circle_J3->width = 3;
    ui_g_Joint_Circle_J3->r = 25;

    ui_g_Joint_Circle_J4->figure_type = 2;
    ui_g_Joint_Circle_J4->operate_type = 1;
    ui_g_Joint_Circle_J4->layer = 0;
    ui_g_Joint_Circle_J4->color = 8;
    ui_g_Joint_Circle_J4->start_x = 1797;
    ui_g_Joint_Circle_J4->start_y = 447;
    ui_g_Joint_Circle_J4->width = 3;
    ui_g_Joint_Circle_J4->r = 25;

    ui_g_Joint_Circle_J5->figure_type = 2;
    ui_g_Joint_Circle_J5->operate_type = 1;
    ui_g_Joint_Circle_J5->layer = 0;
    ui_g_Joint_Circle_J5->color = 8;
    ui_g_Joint_Circle_J5->start_x = 1797;
    ui_g_Joint_Circle_J5->start_y = 378;
    ui_g_Joint_Circle_J5->width = 3;
    ui_g_Joint_Circle_J5->r = 25;

    ui_g_RobotMode_RunningMode->figure_type = 7;
    ui_g_RobotMode_RunningMode->operate_type = 1;
    ui_g_RobotMode_RunningMode->layer = 0;
    ui_g_RobotMode_RunningMode->color = 2;
    ui_g_RobotMode_RunningMode->start_x = 847;
    ui_g_RobotMode_RunningMode->start_y = 280;
    ui_g_RobotMode_RunningMode->width = 3;
    ui_g_RobotMode_RunningMode->font_size = 35;
    ui_g_RobotMode_RunningMode->str_length = 7;
    strcpy(ui_g_RobotMode_RunningMode->string, "WORKING");

    ui_g_RobotMode_ControllerType->figure_type = 7;
    ui_g_RobotMode_ControllerType->operate_type = 1;
    ui_g_RobotMode_ControllerType->layer = 0;
    ui_g_RobotMode_ControllerType->color = 1;
    ui_g_RobotMode_ControllerType->start_x = 217;
    ui_g_RobotMode_ControllerType->start_y = 211;
    ui_g_RobotMode_ControllerType->width = 3;
    ui_g_RobotMode_ControllerType->font_size = 30;
    ui_g_RobotMode_ControllerType->str_length = 8;
    strcpy(ui_g_RobotMode_ControllerType->string, "JOYSTICK");

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
    ui_g_RobotStatus_FSM_Status->color = 3;
    ui_g_RobotStatus_FSM_Status->start_x = 895;
    ui_g_RobotStatus_FSM_Status->start_y = 225;
    ui_g_RobotStatus_FSM_Status->width = 2;
    ui_g_RobotStatus_FSM_Status->font_size = 20;
    ui_g_RobotStatus_FSM_Status->str_length = 7;
    strcpy(ui_g_RobotStatus_FSM_Status->string, "STAGE 0");

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
    ui_g_RobotStatus_Gripper_Status->str_length = 2;
    strcpy(ui_g_RobotStatus_Gripper_Status->string, "ON");

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
