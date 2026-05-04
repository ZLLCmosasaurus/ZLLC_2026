/**
 * @file config.h
 * @author cjw
 * @brief 工程配置文件
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
// #define MINIPC_COMM_CAN
#define MINIPC_COMM_USB

//底盘或云台状态
// #define CHASSIS
#define GIMBAL


//调试或比赛状态
// #define DEBUG

//#define Only_Chassis
//功率控制相关
// #define POWER_CONTROL 1 //启用功率控制
#define BUFFER_LOOP

//使能超电
#define SuperCap 0

//底盘解算选择
//#define SPEED_SLOPE

//底盘控制选择
// #define Speed_Controller // 速控
#define Force_Controller // 力控

//位姿控制选择
// #define LOCKED_SWITCH
#define AUTO_SWITCH

//发射机构选择
// #define Single_Friction
#define Double_Friction

//遥控器选择
#define USE_VT13
#define USE_DR16

/* 兵种/底盘类型选择*/
// #define AGV      //舵轮底盘
//#define OMNI_WHEEL //全向轮底盘
#define TRACK_LEG //前履带后轮腿底盘

//#define INFANTRY //步兵
#define HERO  //英雄
//#define SENTRY //哨兵

/*轮组数据*/
#ifdef INFANTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef HERO
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef SENTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#define Chassis_Radius 0.46000000f // 底盘半径，单位为m
#endif



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
