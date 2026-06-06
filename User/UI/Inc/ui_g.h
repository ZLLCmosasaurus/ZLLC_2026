//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[17];
extern uint8_t ui_g_dirty_figure[17];
extern ui_interface_string_t ui_g_now_strings[12];
extern uint8_t ui_g_dirty_string[12];

extern uint8_t ui_g_max_send_count[29];

#define ui_g_Lift_Rect_Uplift_RF ((ui_interface_rect_t*)&(ui_g_now_figures[0]))
#define ui_g_Lift_Rect_Uplift_LB ((ui_interface_rect_t*)&(ui_g_now_figures[1]))
#define ui_g_Left_Rect_Uplift_RB ((ui_interface_rect_t*)&(ui_g_now_figures[2]))
#define ui_g_RobotStatus_PowerBar ((ui_interface_rect_t*)&(ui_g_now_figures[3]))
#define ui_g_RobotStatus_SpeedCircle ((ui_interface_round_t*)&(ui_g_now_figures[4]))
#define ui_g_Lift_Rect_Uplift_LF ((ui_interface_rect_t*)&(ui_g_now_figures[5]))
#define ui_g_RobotStatus_Now_Power ((ui_interface_rect_t*)&(ui_g_now_figures[6]))
#define ui_g_RobotStatus_Now_RF ((ui_interface_rect_t*)&(ui_g_now_figures[7]))
#define ui_g_RobotStatus_Now_LF ((ui_interface_rect_t*)&(ui_g_now_figures[8]))
#define ui_g_RobotStatus_Now_RB ((ui_interface_rect_t*)&(ui_g_now_figures[9]))
#define ui_g_RobotStatus_Now_LB ((ui_interface_rect_t*)&(ui_g_now_figures[10]))
#define ui_g_RunningRect_WoringRect ((ui_interface_rect_t*)&(ui_g_now_figures[11]))
#define ui_g_RunningRect_MovingRect ((ui_interface_rect_t*)&(ui_g_now_figures[12]))
#define ui_g_RunningRect_UpliftRect ((ui_interface_rect_t*)&(ui_g_now_figures[13]))
#define ui_g_RunningRect_DownliftRect ((ui_interface_rect_t*)&(ui_g_now_figures[14]))
#define ui_g_RunningRect_SaveloadRect ((ui_interface_rect_t*)&(ui_g_now_figures[15]))
#define ui_g_Now_FSM_FSM_Number ((ui_interface_number_t*)&(ui_g_now_figures[16]))

#define ui_g_RobotStatus_Orientation (&(ui_g_now_strings[0]))
#define ui_g_RobotStatus_none (&(ui_g_now_strings[1]))
#define ui_g_RobotStatus_FSM_Status (&(ui_g_now_strings[2]))
#define ui_g_Tips_Wheels (&(ui_g_now_strings[3]))
#define ui_g_RobotStatus_Wheel_Status (&(ui_g_now_strings[4]))
#define ui_g_Tips_Gripper (&(ui_g_now_strings[5]))
#define ui_g_RobotStatus_Gripper_Status (&(ui_g_now_strings[6]))
#define ui_g_RunningMode_WoringMode (&(ui_g_now_strings[7]))
#define ui_g_RunningMode_MovingMode (&(ui_g_now_strings[8]))
#define ui_g_RunningMode_UpliftMode (&(ui_g_now_strings[9]))
#define ui_g_RunningMode_DownliftMode (&(ui_g_now_strings[10]))
#define ui_g_RunningMode_SaveloadMode (&(ui_g_now_strings[11]))

#define ui_g_Lift_Rect_Uplift_RF_max_send_count (ui_g_max_send_count[0])
#define ui_g_Lift_Rect_Uplift_LB_max_send_count (ui_g_max_send_count[1])
#define ui_g_Left_Rect_Uplift_RB_max_send_count (ui_g_max_send_count[2])
#define ui_g_RobotStatus_PowerBar_max_send_count (ui_g_max_send_count[3])
#define ui_g_RobotStatus_SpeedCircle_max_send_count (ui_g_max_send_count[4])
#define ui_g_Lift_Rect_Uplift_LF_max_send_count (ui_g_max_send_count[5])
#define ui_g_RobotStatus_Now_Power_max_send_count (ui_g_max_send_count[6])
#define ui_g_RobotStatus_Now_RF_max_send_count (ui_g_max_send_count[7])
#define ui_g_RobotStatus_Now_LF_max_send_count (ui_g_max_send_count[8])
#define ui_g_RobotStatus_Now_RB_max_send_count (ui_g_max_send_count[9])
#define ui_g_RobotStatus_Now_LB_max_send_count (ui_g_max_send_count[10])
#define ui_g_RunningRect_WoringRect_max_send_count (ui_g_max_send_count[11])
#define ui_g_RunningRect_MovingRect_max_send_count (ui_g_max_send_count[12])
#define ui_g_RunningRect_UpliftRect_max_send_count (ui_g_max_send_count[13])
#define ui_g_RunningRect_DownliftRect_max_send_count (ui_g_max_send_count[14])
#define ui_g_RunningRect_SaveloadRect_max_send_count (ui_g_max_send_count[15])
#define ui_g_Now_FSM_FSM_Number_max_send_count (ui_g_max_send_count[16])

#define ui_g_RobotStatus_Orientation_max_send_count (ui_g_max_send_count[17])
#define ui_g_RobotStatus_none_max_send_count (ui_g_max_send_count[18])
#define ui_g_RobotStatus_FSM_Status_max_send_count (ui_g_max_send_count[19])
#define ui_g_Tips_Wheels_max_send_count (ui_g_max_send_count[20])
#define ui_g_RobotStatus_Wheel_Status_max_send_count (ui_g_max_send_count[21])
#define ui_g_Tips_Gripper_max_send_count (ui_g_max_send_count[22])
#define ui_g_RobotStatus_Gripper_Status_max_send_count (ui_g_max_send_count[23])
#define ui_g_RunningMode_WoringMode_max_send_count (ui_g_max_send_count[24])
#define ui_g_RunningMode_MovingMode_max_send_count (ui_g_max_send_count[25])
#define ui_g_RunningMode_UpliftMode_max_send_count (ui_g_max_send_count[26])
#define ui_g_RunningMode_DownliftMode_max_send_count (ui_g_max_send_count[27])
#define ui_g_RunningMode_SaveloadMode_max_send_count (ui_g_max_send_count[28])

#ifdef MANUAL_DIRTY
#define ui_g_Lift_Rect_Uplift_RF_dirty (ui_g_dirty_figure[0])
#define ui_g_Lift_Rect_Uplift_LB_dirty (ui_g_dirty_figure[1])
#define ui_g_Left_Rect_Uplift_RB_dirty (ui_g_dirty_figure[2])
#define ui_g_RobotStatus_PowerBar_dirty (ui_g_dirty_figure[3])
#define ui_g_RobotStatus_SpeedCircle_dirty (ui_g_dirty_figure[4])
#define ui_g_Lift_Rect_Uplift_LF_dirty (ui_g_dirty_figure[5])
#define ui_g_RobotStatus_Now_Power_dirty (ui_g_dirty_figure[6])
#define ui_g_RobotStatus_Now_RF_dirty (ui_g_dirty_figure[7])
#define ui_g_RobotStatus_Now_LF_dirty (ui_g_dirty_figure[8])
#define ui_g_RobotStatus_Now_RB_dirty (ui_g_dirty_figure[9])
#define ui_g_RobotStatus_Now_LB_dirty (ui_g_dirty_figure[10])
#define ui_g_RunningRect_WoringRect_dirty (ui_g_dirty_figure[11])
#define ui_g_RunningRect_MovingRect_dirty (ui_g_dirty_figure[12])
#define ui_g_RunningRect_UpliftRect_dirty (ui_g_dirty_figure[13])
#define ui_g_RunningRect_DownliftRect_dirty (ui_g_dirty_figure[14])
#define ui_g_RunningRect_SaveloadRect_dirty (ui_g_dirty_figure[15])
#define ui_g_Now_FSM_FSM_Number_dirty (ui_g_dirty_figure[16])

#define ui_g_RobotStatus_Orientation_dirty (ui_g_dirty_string[0])
#define ui_g_RobotStatus_none_dirty (ui_g_dirty_string[1])
#define ui_g_RobotStatus_FSM_Status_dirty (ui_g_dirty_string[2])
#define ui_g_Tips_Wheels_dirty (ui_g_dirty_string[3])
#define ui_g_RobotStatus_Wheel_Status_dirty (ui_g_dirty_string[4])
#define ui_g_Tips_Gripper_dirty (ui_g_dirty_string[5])
#define ui_g_RobotStatus_Gripper_Status_dirty (ui_g_dirty_string[6])
#define ui_g_RunningMode_WoringMode_dirty (ui_g_dirty_string[7])
#define ui_g_RunningMode_MovingMode_dirty (ui_g_dirty_string[8])
#define ui_g_RunningMode_UpliftMode_dirty (ui_g_dirty_string[9])
#define ui_g_RunningMode_DownliftMode_dirty (ui_g_dirty_string[10])
#define ui_g_RunningMode_SaveloadMode_dirty (ui_g_dirty_string[11])
#endif

void ui_init_g();
void ui_prepare_g();
void ui_sync_g();
void ui_update_g();

#endif // UI_g_H
