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

//底盘或云台状态
#define CHASSIS
//#define GIMBAL

//调试或比赛状态
#define DEBUG

//之前写的机械臂电机测试入口
//#define MOTOR_TEST

//用于测试正逆解角度映射的入口
//#define MY_DEBUG

//底盘电机测试入口
//#define MOTOR_TEST_CHASSIS

//底盘测试入口
#define CHASSIS_TEST

//功率控制相关
#define POWER_CONTROL 1 //启用功率控制
//#define BUFFER_LOOP

//遥控器选择
//#define USE_VT13
#define USE_DR16

/* 兵种/底盘类型选择*/
//#define AGV      //舵轮底盘
//#define OMNI_WHEEL //全向轮底盘

//#define INFANTRY //步兵
//#define HERO  //英雄
//#define SENTRY //哨兵
#define ENGINEER //工程

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

#ifdef ENGINEER
#define Wheel_Diameter 0.141000000f
#endif



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
