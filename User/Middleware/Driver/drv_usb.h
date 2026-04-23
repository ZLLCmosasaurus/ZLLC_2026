/**
 * @file drv_usb.h
 * @author lez by yssickjgd
 * @brief  USB虚拟串口通信初始化与配置流程
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DRV_USB_H
#define DRV_USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// �������ֽڳ���
#define USB_BUFFER_SIZE 48

/* Exported types ------------------------------------------------------------*/


/**
 * @brief USBͨ�Ž��ջص�������������
 *
 */
typedef void (*USB_Call_Back)(uint8_t *Buffer, uint32_t Length);


/**
 * @brief USBͨ�Ŵ����ṹ��
 *
 */
struct Struct_USB_Manage_Object
{
    uint8_t Tx_Buffer[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer[USB_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
	  uint16_t Tx_Buffer_Length;
	  USB_Call_Back Callback_Function;
};

/* Exported variables --------------------------------------------------------*/

extern struct Struct_USB_Manage_Object MiniPC_USB_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void USB_Init(struct Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function);
void TIM_USB_PeriodElapsedCallback(struct Struct_USB_Manage_Object* MiniPC_USB_Manage_Object);


#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
