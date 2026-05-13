/**
 * @file ita_chariot.cpp
 * @author cjw by yssickjgd
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
#include "dvc_dwt.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
    #ifdef CHASSIS
    
        //裁判系统
        Referee.Init(&huart10);

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();
        Force_Control_Chassis.Referee = &Referee;
        Force_Control_Chassis.Init();
        // 底盘随动PID环初始化
        PID_Chassis_Fllow.Init(8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.001f);
        //Yaw轴电机初始化，只读数据，不参与控制
        Motor_Yaw_DM4310.Init(&hfdcan3,DM_Motor_ID_0xA3,DM_Motor_Control_Method_MIT_IMU_Angle);

        //超电
        Chassis.Supercap.Referee = &Referee;
        Force_Control_Chassis.Supercap.Referee = &Referee;

    #elif defined(GIMBAL)
        
        Chassis.Set_Velocity_X_Max(8.0f);
        Chassis.Set_Velocity_Y_Max(8.0f);

        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);

        //遥控器

        DR16.Init(&huart5,&huart1);
        DR16_Dead_Zone = __DR16_Dead_Zone;   


        #ifdef USE_VT13
        FSM_Alive_Control_VT13.Chariot = this;
        FSM_Alive_Control_VT13.Init(5,0);
        #endif

        // 初始化活动控制器为无
        Active_Controller = Controller_NONE;

        //云台
        Gimbal.Init();
        Gimbal.MiniPC = &MiniPC;

        //发射机构
        Booster.Init();
        Booster.MiniPC = &MiniPC;
        Booster.Referee = &Referee;

        //裁判系统
        // Referee.Init(&huart10);
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object);
        MiniPC.Init(&hfdcan1);
        // MiniPC.dmIMU = &Gimbal.dmIMU;
        MiniPC.IMU = &Gimbal.Boardc_BMI;
        MiniPC.Referee = &Referee;


    #endif
    buzzer_init_example();
}


#ifdef CHASSIS
#ifdef TRACK_LEG
/**
 * @brief 底盘给云台发送数据
 * 
 */
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    //发送数据给云台
    uint8_t robot_id,game_state;
    int16_t shoot_speed,Pos_X,Pos_Y;
    robot_id = Referee.Get_ID();
    game_state = Referee.Get_Game_Stage();
    shoot_speed = (int16_t)(Referee.Get_Shoot_Speed() * 1000.0f);
    Pos_X = (int16_t)(Referee.Get_Location_X() * 1000.0f);
    Pos_Y = (int16_t)(Referee.Get_Location_Y() * 1000.0f);
    memcpy(CAN3_Chassis_Tx_Gimbal_Data,&robot_id,sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 1,&game_state,sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 2, &shoot_speed, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 4, &Pos_X, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 6, &Pos_Y, sizeof(int16_t));
}

void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback_1()
{
    
    uint16_t Shooter_Barrel_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Shooter_Heat_CD;
    Shooter_Barrel_Heat_Limit = Referee.Get_Booster_42mm_Heat_Max();
    Shooter_Barrel_Heat = Referee.Get_Booster_42mm_Heat();
    Shooter_Heat_CD = Referee.Get_Booster_42mm_Heat_CD();
    // 发送数据给云台
    memcpy(CAN3_Chassis_Tx_Gimbal_Data_1 + 2, &Shooter_Barrel_Heat_Limit, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data_1 + 4, &Shooter_Barrel_Heat, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data_1 + 6, &Shooter_Heat_CD, sizeof(uint16_t));
}
#endif 

#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */

#ifdef CHASSIS
//控制类型字节
uint8_t control_type;
#ifdef TRACK_LEG
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    Gimbal_Alive_Flag++;
    // 控制类型字节
    uint8_t control_type,ui_type;
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 控制类型字节
    Enum_Chassis_Control_Type chassis_control_type;//底盘控制模式
    Enum_Pose_Control_Type pose_control_type;//位姿切换模式
    Enum_Track_Control_Type track_control_type;//履带控制模式

    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y,tmp_gimbal_pitch;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    memcpy(&ui_type,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(uint8_t));
    memcpy(&tmp_gimbal_pitch, &CAN_Manage_Object->Rx_Buffer.Data[5], sizeof(uint16_t));
    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[7], sizeof(uint8_t));

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    if(fabs(gimbal_velocity_x) < 0.0002f)
        gimbal_velocity_x = 0.0f;
    if(fabs(gimbal_velocity_y) < 0.0002f)
        gimbal_velocity_y = 0.0f;

    float Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch, 0, 0x7FFF, -50.f, 50.f);
    Set_Gimbal_Pitch_Angle(Gimbal_Tx_Pitch_Angle);
    JudgeReceiveData.Pitch_Angle = Gimbal_Tx_Pitch_Angle; // pitch角度
    chassis_control_type = (Enum_Chassis_Control_Type)((control_type >> 6) & 0x03);
    pose_control_type = (Enum_Pose_Control_Type)((control_type >> 4) & 0x03);
    track_control_type = (Enum_Track_Control_Type)((control_type >> 3) & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)((control_type >> 2) & 0x01);
    Chassis_Logics_Direction = (Enum_Chassis_Logics_Direction)((control_type >> 1) & 0x01);
    Supercap_Control_Status = (Enum_Supercap_Control_Status)(control_type & 0x01);

    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);
    Chassis.Set_Pose_Control_Type(pose_control_type);
    Chassis.Set_Track_Control_Type(track_control_type);
    //力控底盘控制
    if(chassis_control_type == Chassis_Control_Type_DISABLE)
        Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
    else
        Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL__);

    if(track_control_type == Track_On)
        Force_Control_Chassis.Set_Track_Control_Type(Track_On__);
    else
        Force_Control_Chassis.Set_Track_Control_Type(Track_Off__);
    
    //小陀螺补偿角度
    if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Positive||
        Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Negative)
    {
        Offset_Angle = 15.0f * DEG_TO_RAD;
    }
    else
    {
        Offset_Angle = 0.0f;
    }
    
    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    Chassis_Angle = Motor_Yaw_DM4310.Get_Now_Radian();
    derta_angle = (Reference_Angle - Chassis_Angle) + Offset_Angle;
    if(derta_angle > PI) derta_angle -= PI2;
    if(derta_angle < -PI) derta_angle += PI2;
    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) + gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(-gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));
    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Force_Control_Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Force_Control_Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
}

void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback_1()
{
    uint16_t before_game_bullet_num = 0;
    uint8_t control_type;
    Enum_Gimbal_Control_Type gimbal_control_type;
    Enum_AimShoot_Control_Type aim_type;

    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint8_t));
    memcpy(&Booster_fric_omega_left, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    memcpy(&Booster_fric_omega_right, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(uint16_t));
    memcpy(&Booster_bullet_num, &CAN_Manage_Object->Rx_Buffer.Data[6], sizeof(uint16_t));

    aim_type = (Enum_AimShoot_Control_Type)((control_type >> 1) & 0x01);
    gimbal_control_type = (Enum_Gimbal_Control_Type)((control_type >> 3) & 0x03);
    MiniPC_Mode = (Enum_MiniPC_Mode)((control_type >> 5) & 0x03);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)((control_type >> 7) & 0x01);
    JudgeReceiveData.Gimbal_Control_Type = gimbal_control_type;
    // JudgeReceiveData.MiniPC_Aim_Status = aim_type;
    if (Referee.Get_Game_Stage() == Referee_Game_Status_Stage_NOT_STARTED)
    {
        Booster_bullet_num_before = before_game_bullet_num;
    }
}


float Class_Chariot::Get_Chassis_Coordinate_System_Angle_Rad() // 小陀螺优劣弧优化
{
    float Yaw_Angle_Rad;
    if(Chassis_Logics_Direction == Chassis_Logic_Direction_Positive)
    {
        Yaw_Angle_Rad = Motor_Yaw_DM4310.Get_Now_Radian()  -Reference_Angle;
    }
    else
    {
        Yaw_Angle_Rad = Motor_Yaw_DM4310.Get_Now_Radian()  -(Reference_Angle + PI);
    }
    
    while(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= PI * 2.0f;
    while(Yaw_Angle_Rad < -PI)
        Yaw_Angle_Rad += PI * 2.0f;
        
    return (Yaw_Angle_Rad);
}

void Class_Chariot::Control_Chassis_Omega_TIM_PeriodElapsedCallback()
{

    // 目标角速度
    float chassis_omega;

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
        chassis_omega = Chassis.Get_Spin_Omega();
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Negative)
        chassis_omega = -Chassis.Get_Spin_Omega();

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {   
        PID_Chassis_Fllow.Set_Target(0);
        PID_Chassis_Fllow.Set_Now(Chassis_SglRound_Angle);
        PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        chassis_omega = -PID_Chassis_Fllow.Get_Out();

    }

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        chassis_omega = 0;
    }

    Chassis.Set_Target_Omega(chassis_omega);
    Force_Control_Chassis.Set_Target_Omega(chassis_omega);
}
#endif

#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */

float speed_a,speed_b;
#ifdef GIMBAL
#ifdef TRACK_LEG

void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;
    Enum_Referee_Data_Robots_ID robo_id;
    Enum_Referee_Game_Status_Stage game_stage;
    int16_t shoot_speed;
    memcpy(&robo_id,CAN_Manage_Object->Rx_Buffer.Data,sizeof(uint8_t));
    memcpy(&game_stage,CAN_Manage_Object->Rx_Buffer.Data+1,sizeof(uint8_t));
    memcpy(&shoot_speed,CAN_Manage_Object->Rx_Buffer.Data+2,sizeof(int16_t));
    Referee.Set_Robot_ID(robo_id);
    Referee.Set_Game_Stage(game_stage);
    Booster.Set_Referee_Bullet_Velocity((float)shoot_speed / 1000.0f);
    MiniPC.Set_Bullet_Speed((float)shoot_speed / 1000.0f);

}

void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback_1()
{
    Chassis_Alive_Flag++;
    uint16_t Shooter_Barrel_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Shooter_Heat_CD;
    memcpy(&Shooter_Barrel_Heat_Limit, CAN_Manage_Object->Rx_Buffer.Data + 2, sizeof(uint16_t));
    memcpy(&Shooter_Barrel_Heat, CAN_Manage_Object->Rx_Buffer.Data + 4, sizeof(uint16_t));
    memcpy(&Shooter_Heat_CD, CAN_Manage_Object->Rx_Buffer.Data + 6, sizeof(uint16_t));
    Referee.Set_Booster_42mm_Heat(Shooter_Barrel_Heat);
    Referee.Set_Booster_42mm_Heat_Max(Shooter_Barrel_Heat_Limit);
    Booster.Set_Cooling_Value(Shooter_Heat_CD);
    
}
#endif

#endif


/**
 * @brief can回调函数给底盘发送数据
 *
 */

#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    uint8_t control_type,ui_type;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    //云台pitch轴角度 float
    float gimbal_pitch = 0;
    // 映射之后的目标速度 int16_t
    uint16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0,tmp_gimbal_pitch = 0;
    // 控制类型字节
    Enum_Chassis_Control_Type chassis_control_type;//底盘控制模式
    Enum_Pose_Control_Type pose_control_type;//位姿切换模式
    Enum_Track_Control_Type track_control_type;//履带控制模式
    Enum_MiniPC_Status minipc_status;//上位机控制模式
    Enum_Supercap_Control_Status supercap_status = Supercap_Control_Status;//超电控制模式
    Enum_Chassis_Logics_Direction logics_direction = Chassis_Logics_Direction;//云台朝向控制

    chassis_velocity_x = Force_Control_Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Force_Control_Chassis.Get_Target_Velocity_Y();
    gimbal_pitch = Gimbal.Motor_Pitch_DM4310.Get_True_Angle_Pitch();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    pose_control_type = Chassis.Get_Pose_Control_Type();
    track_control_type = Chassis.Get_Track_Control_Type();
    minipc_status = (Enum_MiniPC_Status)(MiniPC.Get_MiniPC_Status());

    control_type = (uint8_t)(chassis_control_type << 6 | pose_control_type << 4 | track_control_type << 3 | minipc_status << 2 | logics_direction << 1 | supercap_status);

    // 设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max(), 0, 0x7FFF);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(uint16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max(), 0, 0x7FFF);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(uint16_t));



    tmp_gimbal_pitch = Math_Float_To_Int(gimbal_pitch, -50.f, 50.f ,0,0x7FFF);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 5, &tmp_gimbal_pitch, sizeof(uint16_t));

    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 7, &control_type, sizeof(uint8_t));
}

void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback_1()
{
    uint8_t control_type;
    uint16_t tmp_fric_omega_left = 0;
    uint16_t tmp_fric_omega_right = 0;
    uint16_t tmp_actual_bullet_num = 0;

    
    tmp_fric_omega_left = (uint16_t)abs(Booster.Fric[0].Get_Now_Omega_Radian());
    tmp_fric_omega_right = (uint16_t)abs(Booster.Fric[3].Get_Now_Omega_Radian());
    tmp_actual_bullet_num = Booster.actual_bullet_num;
    MiniPC.Set_Bullet_Num(Booster.actual_bullet_num);
    Enum_Gimbal_Control_Type gimbal_control_type = Gimbal.Get_Gimbal_Control_Type();
    Enum_AimShoot_Control_Type tmp_aim_status = Aim_Status;
    Enum_MiniPC_Mode miniPC_mode = MiniPC_Mode;
    control_type = (uint8_t)(Referee_UI_Refresh_Status << 7 | miniPC_mode << 5 | gimbal_control_type << 3 | tmp_aim_status << 1 );
    memcpy(CAN3_Gimbal_Tx_Chassis_Data_1, &control_type, sizeof(uint8_t));
    memcpy(CAN3_Gimbal_Tx_Chassis_Data_1 + 2, &tmp_fric_omega_left, sizeof(uint16_t));
    memcpy(CAN3_Gimbal_Tx_Chassis_Data_1 + 4, &tmp_fric_omega_right, sizeof(uint16_t));
    memcpy(CAN3_Gimbal_Tx_Chassis_Data_1 + 6, &tmp_actual_bullet_num, sizeof(uint16_t));

}


#endif
/**
 * @brief 底盘控制逻辑
 *
 */  		
float Offset_K = 0.175f;
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y, dr16_r_x;
    float vt13_l_x, vt13_l_y, vt13_r_x;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    // 目标角速度
    float chassis_omega = 0;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    static uint8_t track_change = 0, joint_change = 0;
    // 先判断当前活动的控制器
    Judge_Active_Controller();
    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
        dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);   
            chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();
            //关节电机失能
            Chassis.Set_Pose_Control_Type(Pose_DISABLE);
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            // 吊射模式 失能底盘
            // Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = Chassis.Get_Spin_Omega();
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Negative);
                chassis_omega = -Chassis.Get_Spin_Omega();
            }
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下 位姿切换
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            #ifdef LOCKED_SWITCH
            //遥控器直接控制伸缩腿逻辑：从中到下状态-伸腿;从下到中状态-缩腿
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN)
            {
                Chassis.Set_Pose_Control_Type(Pose_ENABLE);
            }
            else if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE)
            {
                Chassis.Set_Pose_Control_Type(Pose_STANDBY);
            }
            #endif
            #ifdef AUTO_SWITCH
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN)
            {
                Chassis.Set_Pose_Control_Type(Pose_ENABLE);
            }
            #endif
        }

        if(DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)//
        {
            //左拨杆向下,开启履带驱动电机
            Chassis.Set_Track_Control_Type(Track_On);
                
        }
        else
        {
            //关闭履带驱动电机
            Chassis.Set_Track_Control_Type(Track_Off);

        }
        
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        vt13_l_x = (Math_Abs(VT13.Get_Left_X()) > DR16_Dead_Zone) ? VT13.Get_Left_X() : 0;
        vt13_l_y = (Math_Abs(VT13.Get_Left_Y()) > DR16_Dead_Zone) ? VT13.Get_Left_Y() : 0;
        vt13_r_x = (Math_Abs(VT13.Get_Right_X()) > DR16_Dead_Zone) ? VT13.Get_Right_X() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = vt13_l_x * sqrt(1.0f - vt13_l_y * vt13_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = vt13_l_y * sqrt(1.0f - vt13_l_x * vt13_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if(VT13.Get_Switch() == VT13_Switch_Status_Left)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = Chassis.Get_Spin_Omega();
        }
        if(VT13.Get_Switch() == VT13_Switch_Status_Middle)
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);   
            chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();
            //关节电机失能
            Chassis.Set_Pose_Control_Type(Pose_DISABLE);
            //关闭履带驱动电机
            Chassis.Set_Track_Control_Type(Track_Off);
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Right)
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            //位姿切换
            if (VT13.Get_Button_Left() == VT13_Button_TRIG_FREE_PRESSED)
            {
                if(track_change == 0)
                {
                    //开启履带驱动电机
                    Chassis.Set_Track_Control_Type(Track_On);
                    track_change = 1;
                }
                else
                {
                    //关闭履带驱动电机
                    Chassis.Set_Track_Control_Type(Track_Off);
                    track_change = 0;
                }
                
            }
            if (VT13.Get_Button_Right() == VT13_Button_TRIG_FREE_PRESSED)
            {
                if(joint_change == 0)
                {
                    Chassis.Set_Pose_Control_Type(Pose_ENABLE);
                    joint_change = 1;
                }
                else
                {
                    Chassis.Set_Pose_Control_Type(Pose_STANDBY);
                    joint_change = 0;
                }
                
            }
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        if(Active_Controller == Controller_DR16)
        {
            //按键R：刷新UI
            if (DR16.Get_Keyboard_Key_R() == DR16_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
            }
            else
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
            }
            //按键Shift：底盘在移动的基础上按住Shift加速
            if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED) // 按住shift加速
            {
                DR16_Mouse_Chassis_Shift = 1.0f;
                Sprint_Status = Sprint_Status_ENABLE;
                Supercap_Control_Status = Supercap_Control_Status_ENABLE;
            }
            else
            {
                DR16_Mouse_Chassis_Shift = 2.0f;
                Sprint_Status = Sprint_Status_DISABLE;
                Supercap_Control_Status = Supercap_Control_Status_DISABLE;
            }
            //·按键W：底盘往前运动·按键S：底盘往后运动·按键A：底盘往左运动·按键D：底盘往右运动
            if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED) // x轴
            {
                chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
            {
                chassis_velocity_x = Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
            {
                chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
            {
                chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
            }
            //按键E：切换小陀螺与随动
            if (DR16.Get_Keyboard_Key_E() == DR16_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                }
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }
        }
        if(Active_Controller == Controller_VT13)
        {
            //按键R：刷新UI
            if (VT13.Get_Keyboard_Key_R() == VT13_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
            }
            else
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
            }
            //按键Shift：底盘在移动的基础上按住Shift加速
            if (VT13.Get_Keyboard_Key_Shift() == VT13_Key_Status_PRESSED) // 按住shift加速
            {
                VT13_Mouse_Chassis_Shift = 1.0f;
                Sprint_Status = Sprint_Status_ENABLE;
                Supercap_Control_Status = Supercap_Control_Status_ENABLE;
            }
            else
            {
                VT13_Mouse_Chassis_Shift = 2.0f;
                Sprint_Status = Sprint_Status_DISABLE;
                Supercap_Control_Status = Supercap_Control_Status_DISABLE;
            }
            // 按键Z:切换履带驱动电机开关
            if(VT13.Get_Keyboard_Key_Z() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if(Chassis.Get_Track_Control_Type() == Track_Off)
                {
                    Chassis.Set_Track_Control_Type(Track_On);
                    Supercap_Control_Status = Supercap_Control_Status_ENABLE;
                }
                else
                {
                    Chassis.Set_Track_Control_Type(Track_Off);
                    Supercap_Control_Status = Supercap_Control_Status_DISABLE;
                }
            }
            // 按键V:切换位姿控制模式
            if(VT13.Get_Keyboard_Key_V() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if(Chassis.Get_Pose_Control_Type() == Pose_DISABLE)
                {
                    Chassis.Set_Pose_Control_Type(Pose_ENABLE);
                }
                else if(Chassis.Get_Pose_Control_Type() == Pose_ENABLE)
                {
                    Chassis.Set_Pose_Control_Type(Pose_STANDBY);
                }
                else if(Chassis.Get_Pose_Control_Type() == Pose_STANDBY)
                {
                    Chassis.Set_Pose_Control_Type(Pose_DISABLE);
                }
            }
            //·按键W：底盘往前运动·按键S：底盘往后运动·按键A：底盘往左运动·按键D：底盘往右运动
            if (VT13.Get_Keyboard_Key_A() == VT13_Key_Status_PRESSED) // x轴
            {
                chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / VT13_Mouse_Chassis_Shift;
            }
            if (VT13.Get_Keyboard_Key_D() == VT13_Key_Status_PRESSED)
            {
                chassis_velocity_x = Chassis.Get_Velocity_X_Max() / VT13_Mouse_Chassis_Shift;
            }
            if (VT13.Get_Keyboard_Key_W() == VT13_Key_Status_PRESSED) // y轴
            {
                chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / VT13_Mouse_Chassis_Shift;
            }
            if (VT13.Get_Keyboard_Key_S() == VT13_Key_Status_PRESSED)
            {
                chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / VT13_Mouse_Chassis_Shift;
            }
            //按键E：切换小陀螺与随动
            if (VT13.Get_Keyboard_Key_E() == VT13_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                }
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }
        }
    }
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Omega(chassis_omega);
    //力控底盘任务
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    if(chassis_control_type == Chassis_Control_Type_DISABLE)
    {
        Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
    }
    else
    {
        Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL__);
    }
    Force_Control_Chassis.Set_Target_Velocity_X(chassis_velocity_y);
    Force_Control_Chassis.Set_Target_Velocity_Y(-chassis_velocity_x);

}
#endif

/**
 * @brief 鼠标数据转换
 *
 */
#ifdef GIMBAL
void Class_Chariot::Transform_Mouse_Axis()
{
    // 根据当前活动的控制器选择鼠标数据
    if (Active_Controller == Controller_DR16)
    {
        True_Mouse_X = -DR16.Get_Mouse_X();
        True_Mouse_Y = -DR16.Get_Mouse_Y();
        True_Mouse_Z = DR16.Get_Mouse_Z();
    }
    else if (Active_Controller == Controller_VT13)
    {
        True_Mouse_X = -VT13.Get_Mouse_X();
        True_Mouse_Y = -VT13.Get_Mouse_Y();
        True_Mouse_Z = VT13.Get_Mouse_Z();
    }
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
float Dt6;
uint32_t last_cnt6;
#ifdef GIMBAL
void Class_Chariot::Control_Gimbal()
{
    // 角度目标值
    //float tmp_gimbal_yaw, tmp_gimbal_pitch;
    // 遥控器摇杆值
    float dr16_y, dr16_r_y;
    float vt13_y = 0, vt13_r_y = 0;
    static float Remote_K = 2.0f;

    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    // 静态变量记录上一次的发射模式
    static Enum_Gimbal_Launch_Mode last_launch_mode = Launch_Disable;
    // 在 switch 之前检测模式变化
    if (Gimbal.Get_Gimbal_Launch_Mode() != last_launch_mode) 
    {
        if (Gimbal.Get_Gimbal_Launch_Mode() == Launch_Enable)
        {
            // 刚进入吊射模式：将目标角度同步为当前编码器角度（防止突变）
            float now_enc_yaw = Gimbal.Motor_Yaw_DM4310.Get_True_Angle_Yaw_From_Encoder();
            float now_enc_pitch = Gimbal.Motor_Pitch_DM4310.Get_True_Angle_Pitch_From_Encoder();
            Gimbal.Set_Target_Yaw_Angle(now_enc_yaw);
            Gimbal.Set_Target_Pitch_Angle(now_enc_pitch);
            // 同时更新 tmp 变量(遥控器累加的基础值)
            tmp_gimbal_yaw = now_enc_yaw;
            tmp_gimbal_pitch = now_enc_pitch;
        }
        else if (Gimbal.Get_Gimbal_Launch_Mode() == Launch_Disable)
        {
            // 退出吊射模式：同步到IMU角度
            float now_imu_yaw = Gimbal.Motor_Yaw_DM4310.Get_True_Angle_Yaw();  // IMU角度
            float now_imu_pitch = Gimbal.Motor_Pitch_DM4310.Get_True_Angle_Pitch();
            Gimbal.Set_Target_Yaw_Angle(now_imu_yaw);
            Gimbal.Set_Target_Pitch_Angle(now_imu_pitch);
            // 同时更新 tmp 变量(遥控器累加的基础值)
            tmp_gimbal_yaw = now_imu_yaw;
            tmp_gimbal_pitch = now_imu_pitch;
        }
    last_launch_mode = Gimbal.Get_Gimbal_Launch_Mode();
    }
    /************************************遥控器控制逻辑*********************************************/   
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;


        // 遥控器操作逻辑
        tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * Remote_K;
        tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Angle_Resolution * Remote_K;

        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下 
        {
            // Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC); // 上位机控制
            Gimbal.Set_Gimbal_Launch_Mode(Launch_Disable); // 非吊射
        }
        else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) //左上
        {
            // Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC); // 上位机控制
            // Gimbal.Set_Gimbal_Launch_Mode(Launch_Enable); // 吊射
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }
        else // 其余位置都是遥控器控制
        {
            // 中间遥控模式
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Gimbal.Set_Gimbal_Launch_Mode(Launch_Disable); // 非吊射
        }
        
    }
    else if(Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        vt13_y = (Math_Abs(VT13.Get_Right_X()) > DR16_Dead_Zone) ? VT13.Get_Right_X() : 0;
        vt13_r_y = (Math_Abs(VT13.Get_Right_Y()) > DR16_Dead_Zone) ? VT13.Get_Right_Y() : 0;

        // 遥控器操作逻辑
        tmp_gimbal_yaw -= vt13_y * DR16_Yaw_Angle_Resolution;
        tmp_gimbal_pitch += vt13_r_y * DR16_Pitch_Angle_Resolution;

        if(VT13.Get_Switch() != VT13_Switch_Status_Right && VT13.Get_Button_Right() == VT13_Button_PRESSED)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC); // 上位机控制
        }
        else
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        //修正坐标系正方向
        Transform_Mouse_Axis();
        if(Active_Controller == Controller_DR16)
        {
            tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch += DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
            // 长按右键  开启自瞄
            if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
                if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
                {
                    tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                    tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                }
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            // C键按下 一键调头
            if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
            }
        }
        if(Active_Controller == Controller_VT13)
        {
            tmp_gimbal_yaw -= VT13.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch += VT13.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
            // 长按右键  开启自瞄
            if (VT13.Get_Mouse_Right_Key() == VT13_Key_Status_PRESSED)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            // C键按下 一键调头
            if (VT13.Get_Keyboard_Key_C() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if(Chassis_Logics_Direction == Chassis_Logic_Direction_Positive)
                {
                    Chassis_Logics_Direction = Chassis_Logic_Direction_Negative;
                }
                else 
                    Chassis_Logics_Direction = Chassis_Logic_Direction_Positive;
                tmp_gimbal_yaw += 180;
            }
            // X键按下 切换吊射模式
            if(VT13.Get_Keyboard_Key_X() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Gimbal.Get_Gimbal_Launch_Mode() == Launch_Disable) 
                {
                    Gimbal.Set_Gimbal_Launch_Mode(Launch_Enable);
                }
                else
                {
                    Gimbal.Set_Gimbal_Launch_Mode(Launch_Disable);
                }
            }
        }
    }
    // 设定角度
    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
}
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */
int Booster_Sign = 0;
float Dt2;
uint32_t last_cnt2 = 0;
static uint8_t Switch_Flag_1 = 0;
static uint8_t change_type = 1;
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    static uint8_t booster_sign = 0;
    // 先判断当前活动的控制器
    Judge_Active_Controller();
    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
       volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
       volatile int DR16_Right_Switch_Status = DR16.Get_Right_Switch();
       switch (DR16_Right_Switch_Status){
            case(DR16_Switch_Status_UP): // 右上 开启摩擦轮和发射机构
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                if (DR16_Left_Switch_Status == DR16_Switch_Status_DOWN)//左下
                {
                    Dt2 = DWT_GetDeltaT(&last_cnt2);
                    
                    // 静态变量记录上次开火时间
                    static uint32_t last_fire_time = 0;

                    // 获取当前系统时间（毫秒）
                    uint32_t now = HAL_GetTick();

                    // 开火条件：Fire 有效、冷却完成、热量允许
                    if (MiniPC.Get_Mode() == 2 &&
                        (now - last_fire_time) >= 500 &&
                        Booster.Cmd_if_Fire == 1 && MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)   // 假设热量上限为100
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                        last_fire_time = now;
                    }


                }
                if (DR16_Left_Switch_Status == DR16_Switch_Status_UP)
                {

                    if (DR16.Get_Yaw() < 0.2 && DR16.Get_Yaw() > -0.2)
                    {
                        booster_sign = 0;
                    }
                    else if (DR16.Get_Yaw() > 0.8 && booster_sign == 0)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                        booster_sign = 1;
                    }
                }
                else
                {
                    // if(Booster.Cmd_if_Fire == 1)
                    // {
                        if (DR16.Get_Yaw() < 0.2 && DR16.Get_Yaw() > -0.2)
                        {
                            booster_sign = 0;
                        }
                        else if (DR16.Get_Yaw() > 0.8 && booster_sign == 0)
                        {
                            Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                            booster_sign = 1;
                        }
                    // }
                }
                break;    
            }
            case(DR16_Switch_Status_MIDDLE):
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
               break;
            }
            case(DR16_Switch_Status_DOWN):
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                break;
            }
        }
        switch(DR16_Left_Switch_Status)
        {
            case(DR16_Switch_Status_UP):
            {
                Booster.Set_Shooter_Mode(Launcher);
                MiniPC_Mode = MiniPC_RADAR;
                break;
            }
            case(DR16_Switch_Status_MIDDLE):
            {
                Booster.Set_Shooter_Mode(Normal);
                MiniPC_Mode = MiniPC_DISABLE;
                break;
            }
            case(DR16_Switch_Status_DOWN):
            {
                Booster.Set_Shooter_Mode(Aimer);
                MiniPC_Mode = MiniPC_AIMER;
                break;
            }
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {

        Dt6 = 1.0f / DWT_GetDeltaT(&last_cnt6);

        if(VT13.Get_Switch() != VT13_Switch_Status_Right)
        {
            if(VT13.Get_Button_Left() == VT13_Button_TRIG_FREE_PRESSED)
            {
               if (Fric_Status == Fric_Status_OPEN)
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                    Fric_Status = Fric_Status_CLOSE;
                }
                else
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                    Fric_Status = Fric_Status_OPEN;
                }
                
            }
            if(Fric_Status == Fric_Status_OPEN)
            {
                if (VT13.Get_Button_Right() == VT13_Button_PRESSED)
                {
                    //自瞄模式
                    // 静态变量记录上次开火时间
                    static uint32_t last_fire_time = 0;

                    // 当前 Fire 信号
                    uint8_t current_fire = MiniPC.Get_Fire_Status();

                    // 获取当前系统时间（毫秒）
                    uint32_t now = HAL_GetTick();

                    // 开火条件：Fire 有效、冷却完成、热量允许
                    if (current_fire == 1 &&
                        (now - last_fire_time) >= 500 &&
                        Booster.Cmd_if_Fire == 1 && MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)   // 假设热量上限为100
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                        last_fire_time = now;
                    }
                }
                if (VT13.Get_Yaw() < 0.2 && VT13.Get_Yaw() > -0.2)
                {
                    booster_sign = 0;
                }
                if (VT13.Get_Yaw() < -0.8 && booster_sign == 0)
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    booster_sign = 1;
                }
            }
            
        }
        else if(VT13.Get_Switch() == VT13_Switch_Status_Right)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            Fric_Status = Fric_Status_CLOSE;
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        if (Active_Controller == Controller_DR16)
        {
            // 按下ctrl键 切换摩擦轮状态
            if (DR16.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Fric_Status == Fric_Status_CLOSE)
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                    Fric_Status = Fric_Status_OPEN;
                }
                else
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                    Fric_Status = Fric_Status_CLOSE;
                }
            }
            //长按鼠标右键自瞄
            if(DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED)
            {
                if(Fric_Status == Fric_Status_OPEN)
                {
                    //按下Q键自瞄控制打弹
                    if(DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_TRIG_FREE_PRESSED)
                    {
                            //自瞄模式
                            // 静态变量记录上次开火时间
                            static uint32_t last_fire_time = 0;

                            // 当前 Fire 信号
                            uint8_t current_fire = MiniPC.Get_Fire_Status();

                            // 获取当前系统时间（毫秒）
                            uint32_t now = HAL_GetTick();

                            // 开火条件：Fire 有效、冷却完成、热量允许
                            if (current_fire == 1 &&
                                (now - last_fire_time) >= 500 &&
                                Booster.Cmd_if_Fire == 1 && MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)   // 假设热量上限为100
                            {
                                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                                last_fire_time = now;
                            }

                    }
                    else if(DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED && Booster.Cmd_if_Fire == 1)
                    {
                        //单发
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    }
                    else
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    }
                }
            }
            //鼠标左键单点控制开火 单发
            else if((DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED) && Fric_Status == Fric_Status_OPEN
            && Booster.Cmd_if_Fire == 1)
            {
                //单发
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
        }
        if(Active_Controller == Controller_VT13)
        {
            
            //CTRL键控制摩擦轮
            if(VT13.Get_Keyboard_Key_Ctrl() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {

                if (Fric_Status == Fric_Status_CLOSE)
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                    Fric_Status = Fric_Status_OPEN;
                }
                else
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                    Fric_Status = Fric_Status_CLOSE;
                }
            }
            //长按鼠标右键自瞄
            if(VT13.Get_Mouse_Right_Key() == VT13_Key_Status_PRESSED)
            {
                if(Gimbal.Get_Gimbal_Launch_Mode() == Launch_Enable)
                {
                    Booster.Set_Shooter_Mode(Launcher);
                    MiniPC_Mode = MiniPC_RADAR;
                }
                else
                {
                    Booster.Set_Shooter_Mode(Aimer);
                    MiniPC_Mode = MiniPC_AIMER;
                    if(Fric_Status == Fric_Status_OPEN)
                    {

                        //自瞄模式
                        // 静态变量记录上次开火时间
                        static uint32_t last_fire_time = 0;

                        // 获取当前系统时间（毫秒）
                        uint32_t now = HAL_GetTick();

                        // 开火条件：Fire 有效、冷却完成、热量允许
                        if (MiniPC.Get_Mode() == 2 &&
                            (now - last_fire_time) >= 500 &&
                            Booster.Cmd_if_Fire == 1 && MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)   // 假设热量上限为100
                        {
                            Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                            last_fire_time = now;
                        }
                    }
                }
                
            }
            else
            {
                Booster.Set_Shooter_Mode(Normal);
                MiniPC_Mode = MiniPC_DISABLE;
            }
            //鼠标左键单点控制开火 单发
            if((VT13.Get_Mouse_Left_Key() == VT13_Key_Status_TRIG_FREE_PRESSED) && Fric_Status == Fric_Status_OPEN)
            {
                //单发
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }

        }

    }

    //UI显示检测摩擦轮是否开启
    if(abs(Booster.Fric[0].Get_Now_Omega_Rpm()) > Booster.Get_Friction_Omega_Threshold() &&
    abs(Booster.Fric[2].Get_Now_Omega_Rpm()) > Booster.Get_Friction_Omega_Threshold())
    {
        Fric_Status = Fric_Status_OPEN;
    }
    else
    {
        Fric_Status = Fric_Status_CLOSE;
    }

    MiniPC.Set_Lidar_if_Lob(MiniPC_Mode);

    if(VT13.Get_VT13_Status() == VT13_Status_ENABLE && VT13.Get_Switch() == VT13_Switch_Status_Right)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
    }
}
#endif

/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    #ifdef CHASSIS
    	
    #ifdef TRACK_LEG
        Chassis_SglRound_Angle = Get_Chassis_Coordinate_System_Angle_Rad();
        // 底盘给云台发消息
        CAN_Chassis_Tx_Gimbal_Callback();
        CAN_Chassis_Tx_Gimbal_Callback_1();
        //底盘Omega控制
        Control_Chassis_Omega_TIM_PeriodElapsedCallback();
        //底盘姿态控制
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        //超电使用策略
        if(Supercap_Control_Status == Supercap_Control_Status_ENABLE)
        {
            Force_Control_Chassis.Supercap.Set_Supercap_Usage_Stratage(Supercap_Usage_Stratage_Supercap_BufferPower);
        }
        else
        {
            Force_Control_Chassis.Supercap.Set_Supercap_Usage_Stratage(Supercap_Usage_Stratage_Referee_BufferPower);
        }
        Force_Control_Chassis.Supercap.TIM_Supercap_PeriodElapsedCallback();
        static uint8_t mod2 = 0;
        mod2++;
        if (mod2 == 2)
        {
            //力控底盘解算
            Force_Control_Chassis.TIM_2ms_Control_PeriodElapsedCallback();
            Force_Control_Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
            mod2 = 0;
        }
        //kalman滤波处理
        Force_Control_Chassis.TIM_1ms_Kalmancale_PeriodElapsedCallback();

            //底盘离线保护
    #ifdef CHASSIS

        if(Get_Gimbal_Status() == Gimbal_Status_DISABLE || Motor_Yaw_DM4310.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
        {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
        Chassis.Set_Pose_Control_Type(Pose_DISABLE);
        Chassis.Set_Track_Control_Type(Track_Off);
        Force_Control_Chassis.Set_Track_Control_Type(Track_Off__);
        }

    #endif
    
    #endif	
    #elif defined(GIMBAL)

        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster.TIM_Calculate_PeriodElapsedCallback();
        //传输数据给上位机
        MiniPC.TIM_Write_PeriodElapsedCallback();
        //给下板发送数据
        CAN_Gimbal_Tx_Chassis_Callback();
        CAN_Gimbal_Tx_Chassis_Callback_1();
    #endif   
}

#ifdef GIMBAL
/**
 * @brief 判断DR16控制数据来源
 *
 */
void Class_Chariot::Judge_DR16_Control_Type()
{
    if (DR16.Get_Left_X() != 0 ||
        DR16.Get_Left_Y() != 0 ||
        DR16.Get_Right_X() != 0 ||
        DR16.Get_Right_Y() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_REMOTE;
    }
    else if (DR16.Get_Mouse_X() != 0 ||
             DR16.Get_Mouse_Y() != 0 ||
             DR16.Get_Mouse_Z() != 0 ||
             DR16.Get_Keyboard_Key_A() != 0 ||
             DR16.Get_Keyboard_Key_D() != 0 ||
             DR16.Get_Keyboard_Key_W() != 0 ||
             DR16.Get_Keyboard_Key_S() != 0 ||
             DR16.Get_Keyboard_Key_Shift() != 0 ||
             DR16.Get_Keyboard_Key_Ctrl() != 0 ||
             DR16.Get_Keyboard_Key_Q() != 0 ||
             DR16.Get_Keyboard_Key_E() != 0 ||
             DR16.Get_Keyboard_Key_R() != 0 ||
             DR16.Get_Keyboard_Key_F() != 0 ||
             DR16.Get_Keyboard_Key_G() != 0 ||
             DR16.Get_Keyboard_Key_Z() != 0 ||
             DR16.Get_Keyboard_Key_C() != 0 ||
             DR16.Get_Keyboard_Key_V() != 0 ||
             DR16.Get_Keyboard_Key_B() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_KEYBOARD;
    }
    else
    {
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            DR16_Control_Type = DR16_Control_Type_NONE;
    }
}

/**
 * @brief 判断VT13控制数据来源
 *
 */
void Class_Chariot::Judge_VT13_Control_Type()
{
    if (VT13.Get_Left_X() != 0 ||
        VT13.Get_Left_Y() != 0 ||
        VT13.Get_Right_X() != 0 ||
        VT13.Get_Right_Y() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_REMOTE;
    }
    else if (VT13.Get_Mouse_X() != 0 ||
             VT13.Get_Mouse_Y() != 0 ||
             VT13.Get_Mouse_Z() != 0 ||
             VT13.Get_Keyboard_Key_A() != 0 ||
             VT13.Get_Keyboard_Key_D() != 0 ||
             VT13.Get_Keyboard_Key_W() != 0 ||
             VT13.Get_Keyboard_Key_S() != 0 ||
             VT13.Get_Keyboard_Key_Shift() != 0 ||
             VT13.Get_Keyboard_Key_Ctrl() != 0 ||
             VT13.Get_Keyboard_Key_Q() != 0 ||
             VT13.Get_Keyboard_Key_E() != 0 ||
             VT13.Get_Keyboard_Key_R() != 0 ||
             VT13.Get_Keyboard_Key_F() != 0 ||
             VT13.Get_Keyboard_Key_G() != 0 ||
             VT13.Get_Keyboard_Key_Z() != 0 ||
             VT13.Get_Keyboard_Key_C() != 0 ||
             VT13.Get_Keyboard_Key_V() != 0 ||
             VT13.Get_Keyboard_Key_B() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_KEYBOARD;
    }
    else
    {
        if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            VT13_Control_Type = VT13_Control_Type_NONE;
    }
}
#endif
/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    //判断DR16控制数据来源
    Judge_DR16_Control_Type();
    //判断VT13控制数据来源
    Judge_VT13_Control_Type();
    //底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
}
#endif

/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    static uint8_t mod50_mod7 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
        mod50_mod7++;
        //TIM_Unline_Protect_PeriodElapsedCallback();
        #ifdef CHASSIS
						
            Motor_Yaw_DM4310.TIM_Alive_PeriodElapsedCallback();
            #ifndef AGV
            for (auto& wheel : Chassis.Motor_Wheel) {
                wheel.TIM_Alive_PeriodElapsedCallback();
            }     
            #endif
            #ifdef TRACK_LEG
            for (int i = 0; i < 2; i++)
            {
                Chassis.Motor_Joint[i].TIM_Alive_PeriodElapsedCallback();
                Chassis.Motor_Leg[i].TIM_Alive_PeriodElapsedCallback();
                Chassis.Motor_Guider[i].TIM_Alive_PeriodElapsedCallback();
            }
            // Chassis.Motor_Track[0].TIM_Alive_PeriodElapsedCallback();
            // Chassis.Motor_Track[1].TIM_Alive_PeriodElapsedCallback();
            Chassis.BoardDM_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();
            //力控底盘
            Force_Control_Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
            #endif
            if(mod50_mod3%3 == 0)
            {
                TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
                Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
                Force_Control_Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;
            }  

        #elif defined(GIMBAL)

            if(mod50_mod3%3==0)
            {
                //判断底盘通讯在线状态
                TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
                DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
                VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;         
            }

            if (mod50_mod7 % 7 == 0)
            {
                MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
                mod50_mod7 = 0;
            }
                
            Gimbal.Motor_Pitch_DM4310.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw_DM4310.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();
            Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            #ifdef Single_Friction
            Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
			#endif
            #ifdef Double_Friction
            for (auto i = 0; i < 4; i++)
            {
                Booster.Fric[i].TIM_Alive_PeriodElapsedCallback();
            }
            #endif
            // Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

        #endif

        mod50 = 0;
    }    
}

/**
 * @brief 离线保护函数
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    //云台离线保护
    #ifdef GIMBAL
        #ifdef defined(USE_DR16)
                #ifdef DEBUG
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #else
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
                {
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                }
                #endif
            #elif defined(USE_VT13)
                #ifdef DEBUG
                    if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #else

                #endif

        #endif
        #ifndef DEBUG
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE && VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            // 记录离线前一状态
            Pre_Gimbal_Control_Type = Gimbal.Get_Gimbal_Control_Type();
            Pre_Chassis_Control_Type = Chassis.Get_Chassis_Control_Type();
            // 控制模块禁用
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            // 遥控器中途断联导致错误，重启 DMA
            if (huart5.ErrorCode)
            {
                HAL_UART_DMAStop(&huart5); // 停止以重启
                // HAL_Delay(10); // 等待错误结束
                HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
            }
        }
        #endif
    #endif

    //底盘离线保护
    #ifdef CHASSIS

        if(Get_Gimbal_Status() == Gimbal_Status_DISABLE || Motor_Yaw_DM4310.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
        {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
        Chassis.Set_Pose_Control_Type(Pose_DISABLE);
        Chassis.Set_Track_Control_Type(Track_Off);
        Force_Control_Chassis.Set_Track_Control_Type(Track_Off__);
        }

    #endif

}

/**
 * @brief 底盘通讯在线判断回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback()
{
    if (Chassis_Alive_Flag == Pre_Chassis_Alive_Flag)
    {
        Chassis_Status = Chassis_Status_DISABLE;
        buzzer_setTask(&buzzer , BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        Chassis_Status = Chassis_Status_ENABLE;
    }
    Pre_Chassis_Alive_Flag = Chassis_Alive_Flag;   
}
#endif

#ifdef CHASSIS
void Class_Chariot::TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback()
{
    if (Gimbal_Alive_Flag == Pre_Gimbal_Alive_Flag)
    {
        Gimbal_Status = Gimbal_Status_DISABLE;
        buzzer_setTask(&buzzer,BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        Gimbal_Status = Gimbal_Status_ENABLE;
    }
    Pre_Gimbal_Alive_Flag = Gimbal_Alive_Flag;  
}
#endif
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                Chariot->Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
                Chariot->Chassis.Set_Pose_Control_Type(Pose_DISABLE);
                Chariot->Chassis.Set_Track_Control_Type(Track_Off);
            }
            
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Chariot->Set_Pre_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chariot->Set_Pre_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart5); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif

/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control_VT13::Reload_TIM_Status_PeriodElapsedCallback(){
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                Chariot->Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
                Chariot->Chassis.Set_Pose_Control_Type(Pose_DISABLE);
                Chariot->Chassis.Set_Track_Control_Type(Track_Off);
            }
            

            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Chariot->Set_Pre_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chariot->Set_Pre_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart1); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif

/**
 * @brief 判断当前活动的控制器
 *
 */
#ifdef GIMBAL
void Class_Chariot::Judge_Active_Controller()
{
    // 检查DR16是否有输入
    Judge_DR16_Control_Type();

    // 检查VT13是否有输入
    Judge_VT13_Control_Type();

    // 判断当前活动的控制器
    if (VT13_Control_Type != VT13_Control_Type_NONE)
    {
        Active_Controller = Controller_VT13;
    }
    else if (DR16_Control_Type != DR16_Control_Type_NONE)
    {
        Active_Controller = Controller_DR16;
    }
    else
    {
        Active_Controller = Controller_NONE;
    }
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
