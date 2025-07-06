/**
 * @file drv_can.h
 * @author cjw by yssickjgd
 * @brief CAN通信初始化与配置流程
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdlib.h"
#include "string.h"
  
#ifdef STM32H723xx 
#include "stm32h7xx_hal.h"
#endif  
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif   

/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE    0
#define CAN_REMOTE_TYPE  1

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct Struct_CAN_Rx_Buffer
{
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct Struct_CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
};

/* Exported variables ---------------------------------------------------------*/

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x1fe_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0xxf1_Tx_Data[];
extern uint8_t CAN1_0xxf2_Tx_Data[];
extern uint8_t CAN1_0xxf3_Tx_Data[];
extern uint8_t CAN1_0xxf4_Tx_Data[];
extern uint8_t CAN1_0xxf5_Tx_Data[];
extern uint8_t CAN1_0xxf6_Tx_Data[];
extern uint8_t CAN1_0xxf7_Tx_Data[];
extern uint8_t CAN1_0xxf8_Tx_Data[];

extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x1fe_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0xxf1_Tx_Data[];
extern uint8_t CAN2_0xxf2_Tx_Data[];
extern uint8_t CAN2_0xxf3_Tx_Data[];
extern uint8_t CAN2_0xxf4_Tx_Data[];
extern uint8_t CAN2_0xxf5_Tx_Data[];
extern uint8_t CAN2_0xxf6_Tx_Data[];
extern uint8_t CAN2_0xxf7_Tx_Data[];
extern uint8_t CAN2_0xxf8_Tx_Data[];

extern uint8_t CAN3_0x1ff_Tx_Data[];
extern uint8_t CAN3_0x1fe_Tx_Data[];
extern uint8_t CAN3_0x200_Tx_Data[];
extern uint8_t CAN3_0x2ff_Tx_Data[];
extern uint8_t CAN3_0xxf1_Tx_Data[];
extern uint8_t CAN3_0xxf2_Tx_Data[];
extern uint8_t CAN3_0xxf3_Tx_Data[];
extern uint8_t CAN3_0xxf4_Tx_Data[];
extern uint8_t CAN3_0xxf5_Tx_Data[];
extern uint8_t CAN3_0xxf6_Tx_Data[];
extern uint8_t CAN3_0xxf7_Tx_Data[];
extern uint8_t CAN3_0xxf8_Tx_Data[];

extern uint8_t CAN_Supercap_Tx_Data[];
extern uint8_t CAN3_Gimbal_Tx_Chassis_Data[];  //云台给底盘发送缓冲区
extern uint8_t CAN3_Sentry_CMD_Data[];     //哨兵自主决策缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_A[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_B[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_C[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_D[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_E[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_F[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_Chassis_Tx_Data_G[];   //底盘给云台发送缓冲区
extern uint8_t CAN3_MiniPC_Tx_Data_A[];   //下位机发送缓冲区
extern uint8_t CAN3_MiniPC_Tx_Data_B[];   //下位机发送缓冲区
extern uint8_t CAN3_MiniPC_Tx_Data_C[];   //下位机发送缓冲区
extern uint8_t CAN3_MiniPC_Tx_Data_D[];   //下位机发送缓冲区

/*********LK电机 控制缓冲区***********/
extern uint8_t CAN1_0x141_Tx_Data[8];
extern uint8_t CAN1_0x142_Tx_Data[8];
extern uint8_t CAN1_0x143_Tx_Data[8];
extern uint8_t CAN1_0x144_Tx_Data[8];
extern uint8_t CAN1_0x145_Tx_Data[8];
extern uint8_t CAN1_0x146_Tx_Data[8];
extern uint8_t CAN1_0x147_Tx_Data[8];
extern uint8_t CAN1_0x148_Tx_Data[8];

extern uint8_t CAN2_0x141_Tx_Data[8];    
extern uint8_t CAN2_0x142_Tx_Data[8];
extern uint8_t CAN2_0x143_Tx_Data[8];
extern uint8_t CAN2_0x144_Tx_Data[8];
extern uint8_t CAN2_0x145_Tx_Data[8];    
extern uint8_t CAN2_0x146_Tx_Data[8];
extern uint8_t CAN2_0x147_Tx_Data[8];
extern uint8_t CAN2_0x148_Tx_Data[8];

extern uint8_t CAN3_0x141_Tx_Data[8];
extern uint8_t CAN3_0x142_Tx_Data[8];
extern uint8_t CAN3_0x143_Tx_Data[8];
extern uint8_t CAN3_0x144_Tx_Data[8];
extern uint8_t CAN3_0x145_Tx_Data[8];
extern uint8_t CAN3_0x146_Tx_Data[8];
extern uint8_t CAN3_0x147_Tx_Data[8];
extern uint8_t CAN3_0x148_Tx_Data[8];
/* Exported function declarations ---------------------------------------------*/

void CAN_Init(FDCAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

uint8_t CAN_Send_Data(FDCAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void TIM_CAN_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
