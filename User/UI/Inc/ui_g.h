//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[33];
extern uint8_t ui_g_dirty_figure[33];
extern ui_interface_string_t ui_g_now_strings[9];
extern uint8_t ui_g_dirty_string[9];

extern uint8_t ui_g_max_send_count[42];

#define ui_g_Lift_Rect_Uplift_RF ((ui_interface_rect_t*)&(ui_g_now_figures[0]))
#define ui_g_Lift_Rect_Uplift_LB ((ui_interface_rect_t*)&(ui_g_now_figures[1]))
#define ui_g_Left_Rect_Uplift_RB ((ui_interface_rect_t*)&(ui_g_now_figures[2]))
#define ui_g_RobotStatus_PowerBar ((ui_interface_rect_t*)&(ui_g_now_figures[3]))
#define ui_g_Joint_Status_J1Min ((ui_interface_line_t*)&(ui_g_now_figures[4]))
#define ui_g_Joint_Status_J1Max ((ui_interface_line_t*)&(ui_g_now_figures[5]))
#define ui_g_Joint_Status_J0Min ((ui_interface_line_t*)&(ui_g_now_figures[6]))
#define ui_g_RobotStatus_SpeedCircle ((ui_interface_round_t*)&(ui_g_now_figures[7]))
#define ui_g_Joint_Status_J0Max ((ui_interface_line_t*)&(ui_g_now_figures[8]))
#define ui_g_Joint_Status_J2Min ((ui_interface_line_t*)&(ui_g_now_figures[9]))
#define ui_g_Joint_Status_J2Max ((ui_interface_line_t*)&(ui_g_now_figures[10]))
#define ui_g_Joint_Status_J3Max ((ui_interface_line_t*)&(ui_g_now_figures[11]))
#define ui_g_Ungroup_J3Min ((ui_interface_line_t*)&(ui_g_now_figures[12]))
#define ui_g_Joint_Status_J4Min ((ui_interface_line_t*)&(ui_g_now_figures[13]))
#define ui_g_Joint_Status_J4Max ((ui_interface_line_t*)&(ui_g_now_figures[14]))
#define ui_g_Joint_Now_J0_Now ((ui_interface_line_t*)&(ui_g_now_figures[15]))
#define ui_g_Joint_Circle_J1 ((ui_interface_round_t*)&(ui_g_now_figures[16]))
#define ui_g_Joint_Now_J1_Now ((ui_interface_line_t*)&(ui_g_now_figures[17]))
#define ui_g_Joint_Now_J2_Now ((ui_interface_line_t*)&(ui_g_now_figures[18]))
#define ui_g_Joint_Now_J3_Now ((ui_interface_line_t*)&(ui_g_now_figures[19]))
#define ui_g_Joint_Now_J4_Now ((ui_interface_line_t*)&(ui_g_now_figures[20]))
#define ui_g_Joint_Now_J5_Now ((ui_interface_line_t*)&(ui_g_now_figures[21]))
#define ui_g_Lift_Rect_Uplift_LF ((ui_interface_rect_t*)&(ui_g_now_figures[22]))
#define ui_g_RobotStatus_Now_Power ((ui_interface_rect_t*)&(ui_g_now_figures[23]))
#define ui_g_RobotStatus_Now_RF ((ui_interface_rect_t*)&(ui_g_now_figures[24]))
#define ui_g_RobotStatus_Now_LF ((ui_interface_rect_t*)&(ui_g_now_figures[25]))
#define ui_g_RobotStatus_Now_RB ((ui_interface_rect_t*)&(ui_g_now_figures[26]))
#define ui_g_RobotStatus_Now_LB ((ui_interface_rect_t*)&(ui_g_now_figures[27]))
#define ui_g_Joint_Circle_J0 ((ui_interface_round_t*)&(ui_g_now_figures[28]))
#define ui_g_Joint_Circle_J2 ((ui_interface_round_t*)&(ui_g_now_figures[29]))
#define ui_g_Joint_Circle_J3 ((ui_interface_round_t*)&(ui_g_now_figures[30]))
#define ui_g_Joint_Circle_J4 ((ui_interface_round_t*)&(ui_g_now_figures[31]))
#define ui_g_Joint_Circle_J5 ((ui_interface_round_t*)&(ui_g_now_figures[32]))

#define ui_g_RobotMode_RunningMode (&(ui_g_now_strings[0]))
#define ui_g_RobotMode_ControllerType (&(ui_g_now_strings[1]))
#define ui_g_RobotStatus_Orientation (&(ui_g_now_strings[2]))
#define ui_g_RobotStatus_none (&(ui_g_now_strings[3]))
#define ui_g_RobotStatus_FSM_Status (&(ui_g_now_strings[4]))
#define ui_g_Tips_Wheels (&(ui_g_now_strings[5]))
#define ui_g_RobotStatus_Wheel_Status (&(ui_g_now_strings[6]))
#define ui_g_Tips_Gripper (&(ui_g_now_strings[7]))
#define ui_g_RobotStatus_Gripper_Status (&(ui_g_now_strings[8]))

#define ui_g_Lift_Rect_Uplift_RF_max_send_count (ui_g_max_send_count[0])
#define ui_g_Lift_Rect_Uplift_LB_max_send_count (ui_g_max_send_count[1])
#define ui_g_Left_Rect_Uplift_RB_max_send_count (ui_g_max_send_count[2])
#define ui_g_RobotStatus_PowerBar_max_send_count (ui_g_max_send_count[3])
#define ui_g_Joint_Status_J1Min_max_send_count (ui_g_max_send_count[4])
#define ui_g_Joint_Status_J1Max_max_send_count (ui_g_max_send_count[5])
#define ui_g_Joint_Status_J0Min_max_send_count (ui_g_max_send_count[6])
#define ui_g_RobotStatus_SpeedCircle_max_send_count (ui_g_max_send_count[7])
#define ui_g_Joint_Status_J0Max_max_send_count (ui_g_max_send_count[8])
#define ui_g_Joint_Status_J2Min_max_send_count (ui_g_max_send_count[9])
#define ui_g_Joint_Status_J2Max_max_send_count (ui_g_max_send_count[10])
#define ui_g_Joint_Status_J3Max_max_send_count (ui_g_max_send_count[11])
#define ui_g_Ungroup_J3Min_max_send_count (ui_g_max_send_count[12])
#define ui_g_Joint_Status_J4Min_max_send_count (ui_g_max_send_count[13])
#define ui_g_Joint_Status_J4Max_max_send_count (ui_g_max_send_count[14])
#define ui_g_Joint_Now_J0_Now_max_send_count (ui_g_max_send_count[15])
#define ui_g_Joint_Circle_J1_max_send_count (ui_g_max_send_count[16])
#define ui_g_Joint_Now_J1_Now_max_send_count (ui_g_max_send_count[17])
#define ui_g_Joint_Now_J2_Now_max_send_count (ui_g_max_send_count[18])
#define ui_g_Joint_Now_J3_Now_max_send_count (ui_g_max_send_count[19])
#define ui_g_Joint_Now_J4_Now_max_send_count (ui_g_max_send_count[20])
#define ui_g_Joint_Now_J5_Now_max_send_count (ui_g_max_send_count[21])
#define ui_g_Lift_Rect_Uplift_LF_max_send_count (ui_g_max_send_count[22])
#define ui_g_RobotStatus_Now_Power_max_send_count (ui_g_max_send_count[23])
#define ui_g_RobotStatus_Now_RF_max_send_count (ui_g_max_send_count[24])
#define ui_g_RobotStatus_Now_LF_max_send_count (ui_g_max_send_count[25])
#define ui_g_RobotStatus_Now_RB_max_send_count (ui_g_max_send_count[26])
#define ui_g_RobotStatus_Now_LB_max_send_count (ui_g_max_send_count[27])
#define ui_g_Joint_Circle_J0_max_send_count (ui_g_max_send_count[28])
#define ui_g_Joint_Circle_J2_max_send_count (ui_g_max_send_count[29])
#define ui_g_Joint_Circle_J3_max_send_count (ui_g_max_send_count[30])
#define ui_g_Joint_Circle_J4_max_send_count (ui_g_max_send_count[31])
#define ui_g_Joint_Circle_J5_max_send_count (ui_g_max_send_count[32])

#define ui_g_RobotMode_RunningMode_max_send_count (ui_g_max_send_count[33])
#define ui_g_RobotMode_ControllerType_max_send_count (ui_g_max_send_count[34])
#define ui_g_RobotStatus_Orientation_max_send_count (ui_g_max_send_count[35])
#define ui_g_RobotStatus_none_max_send_count (ui_g_max_send_count[36])
#define ui_g_RobotStatus_FSM_Status_max_send_count (ui_g_max_send_count[37])
#define ui_g_Tips_Wheels_max_send_count (ui_g_max_send_count[38])
#define ui_g_RobotStatus_Wheel_Status_max_send_count (ui_g_max_send_count[39])
#define ui_g_Tips_Gripper_max_send_count (ui_g_max_send_count[40])
#define ui_g_RobotStatus_Gripper_Status_max_send_count (ui_g_max_send_count[41])

#ifdef MANUAL_DIRTY
#define ui_g_Lift_Rect_Uplift_RF_dirty (ui_g_dirty_figure[0])
#define ui_g_Lift_Rect_Uplift_LB_dirty (ui_g_dirty_figure[1])
#define ui_g_Left_Rect_Uplift_RB_dirty (ui_g_dirty_figure[2])
#define ui_g_RobotStatus_PowerBar_dirty (ui_g_dirty_figure[3])
#define ui_g_Joint_Status_J1Min_dirty (ui_g_dirty_figure[4])
#define ui_g_Joint_Status_J1Max_dirty (ui_g_dirty_figure[5])
#define ui_g_Joint_Status_J0Min_dirty (ui_g_dirty_figure[6])
#define ui_g_RobotStatus_SpeedCircle_dirty (ui_g_dirty_figure[7])
#define ui_g_Joint_Status_J0Max_dirty (ui_g_dirty_figure[8])
#define ui_g_Joint_Status_J2Min_dirty (ui_g_dirty_figure[9])
#define ui_g_Joint_Status_J2Max_dirty (ui_g_dirty_figure[10])
#define ui_g_Joint_Status_J3Max_dirty (ui_g_dirty_figure[11])
#define ui_g_Ungroup_J3Min_dirty (ui_g_dirty_figure[12])
#define ui_g_Joint_Status_J4Min_dirty (ui_g_dirty_figure[13])
#define ui_g_Joint_Status_J4Max_dirty (ui_g_dirty_figure[14])
#define ui_g_Joint_Now_J0_Now_dirty (ui_g_dirty_figure[15])
#define ui_g_Joint_Circle_J1_dirty (ui_g_dirty_figure[16])
#define ui_g_Joint_Now_J1_Now_dirty (ui_g_dirty_figure[17])
#define ui_g_Joint_Now_J2_Now_dirty (ui_g_dirty_figure[18])
#define ui_g_Joint_Now_J3_Now_dirty (ui_g_dirty_figure[19])
#define ui_g_Joint_Now_J4_Now_dirty (ui_g_dirty_figure[20])
#define ui_g_Joint_Now_J5_Now_dirty (ui_g_dirty_figure[21])
#define ui_g_Lift_Rect_Uplift_LF_dirty (ui_g_dirty_figure[22])
#define ui_g_RobotStatus_Now_Power_dirty (ui_g_dirty_figure[23])
#define ui_g_RobotStatus_Now_RF_dirty (ui_g_dirty_figure[24])
#define ui_g_RobotStatus_Now_LF_dirty (ui_g_dirty_figure[25])
#define ui_g_RobotStatus_Now_RB_dirty (ui_g_dirty_figure[26])
#define ui_g_RobotStatus_Now_LB_dirty (ui_g_dirty_figure[27])
#define ui_g_Joint_Circle_J0_dirty (ui_g_dirty_figure[28])
#define ui_g_Joint_Circle_J2_dirty (ui_g_dirty_figure[29])
#define ui_g_Joint_Circle_J3_dirty (ui_g_dirty_figure[30])
#define ui_g_Joint_Circle_J4_dirty (ui_g_dirty_figure[31])
#define ui_g_Joint_Circle_J5_dirty (ui_g_dirty_figure[32])

#define ui_g_RobotMode_RunningMode_dirty (ui_g_dirty_string[0])
#define ui_g_RobotMode_ControllerType_dirty (ui_g_dirty_string[1])
#define ui_g_RobotStatus_Orientation_dirty (ui_g_dirty_string[2])
#define ui_g_RobotStatus_none_dirty (ui_g_dirty_string[3])
#define ui_g_RobotStatus_FSM_Status_dirty (ui_g_dirty_string[4])
#define ui_g_Tips_Wheels_dirty (ui_g_dirty_string[5])
#define ui_g_RobotStatus_Wheel_Status_dirty (ui_g_dirty_string[6])
#define ui_g_Tips_Gripper_dirty (ui_g_dirty_string[7])
#define ui_g_RobotStatus_Gripper_Status_dirty (ui_g_dirty_string[8])
#endif

void ui_init_g();
void ui_prepare_g();
void ui_sync_g();
void ui_update_g();

#endif // UI_g_H
