/**
 * @file drv_usb.cpp
 * @author lez by yssickjgd
 * @brief  USB虚拟串口通信初始化与配置流程
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"
#include "usbd_cdc_if.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//��λ��USBͨѶ����
Struct_USB_Manage_Object MiniPC_USB_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief ��ʼ��USB
 *
 * @param Callback_Function �����ص�����
 */
void USB_Init(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object, USB_Call_Back __Callback_Function)
{
	MiniPC_USB_Manage_Object->Callback_Function = __Callback_Function;
}

/**
 * @brief USB��TIM��ʱ���жϷ��ͻص�����
 *
 */
void TIM_USB_PeriodElapsedCallback(Struct_USB_Manage_Object* MiniPC_USB_Manage_Object)
{
	CDC_Transmit_FS(MiniPC_USB_Manage_Object->Tx_Buffer, MiniPC_USB_Manage_Object->Tx_Buffer_Length+2);  //֡ͷ+֡β
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
