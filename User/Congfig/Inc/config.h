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
//#define CHASSIS
#define GIMBAL

//调试或比赛状态
#define DEBUG
//#define BATTLE

//功率控制相关
#define POWER_LIMIT_JH
//#define POWER_LIMIT_GY
#define POWER_CONTROL 0 //启用功率控制
//#define BUFFER_LOOP

//遥控器选择
#define USE_VT13
//#define USE_DR16

/* 兵种/底盘类型选择*/
#define AGV      //舵轮底盘



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
