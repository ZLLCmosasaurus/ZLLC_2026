/**
 * @file drv_can.cpp
 * @author lez by yssickjgd
 * @brief CAN通信初始化与配置流程
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "ita_chariot.h"
#include "drv_can.h"
#include "main.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0xxf1_Tx_Data[8];
uint8_t CAN1_0xxf2_Tx_Data[8];
uint8_t CAN1_0xxf3_Tx_Data[8];
uint8_t CAN1_0xxf4_Tx_Data[8];
uint8_t CAN1_0xxf5_Tx_Data[8];
uint8_t CAN1_0xxf6_Tx_Data[8];
uint8_t CAN1_0xxf7_Tx_Data[8];
uint8_t CAN1_0xxf8_Tx_Data[8];

uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0xxf1_Tx_Data[8];
uint8_t CAN2_0xxf2_Tx_Data[8];
uint8_t CAN2_0xxf3_Tx_Data[8];
uint8_t CAN2_0xxf4_Tx_Data[8];
uint8_t CAN2_0xxf5_Tx_Data[8];
uint8_t CAN2_0xxf6_Tx_Data[8];
uint8_t CAN2_0xxf7_Tx_Data[8];
uint8_t CAN2_0xxf8_Tx_Data[8];

uint8_t CAN1_0x1fe_Tx_Data[8];
uint8_t CAN2_0x1fe_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8];
uint8_t CAN_Supercap_Tx_Data[8];

uint8_t CAN2_Gimbal_Tx_Chassis_Data[8];   // 云台给底盘发送缓冲区
uint8_t CAN2_Chassis_Tx_Gimbal_Data[8];   // 底盘给云台发送缓冲区
uint8_t CAN2_Gimbal_Tx_Chassis_Data_1[8]; // 云台给底盘发送缓冲区
uint8_t CAN2_Chassis_Tx_Gimbal_Data_1[8]; // 底盘给云台发送缓冲区

uint8_t CAN1_MiniPc_Tx_Data[8];
uint8_t CAN2_MiniPc_Tx_Data[8];
/*********LK电机 控制缓冲区***********/
uint8_t CAN1_0x141_Tx_Data[8];
uint8_t CAN1_0x142_Tx_Data[8];
uint8_t CAN1_0x143_Tx_Data[8];
uint8_t CAN1_0x144_Tx_Data[8];
uint8_t CAN1_0x145_Tx_Data[8];
uint8_t CAN1_0x146_Tx_Data[8];
uint8_t CAN1_0x147_Tx_Data[8];
uint8_t CAN1_0x148_Tx_Data[8];

uint8_t CAN2_0x141_Tx_Data[8];
uint8_t CAN2_0x142_Tx_Data[8];
uint8_t CAN2_0x143_Tx_Data[8];
uint8_t CAN2_0x144_Tx_Data[8];
uint8_t CAN2_0x145_Tx_Data[8];
uint8_t CAN2_0x146_Tx_Data[8];
uint8_t CAN2_0x147_Tx_Data[8];
uint8_t CAN2_0x148_Tx_Data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 筛选器编号0-27 | FIFOx | ID类型 | 帧类型
 * @param ID 验证码
 * @param Mask_ID 屏蔽码(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{

    // 检测传参是否正确
    assert_param(hcan != NULL);

    // CAN过滤器初始化结构体
    CAN_FilterTypeDef can_filter_init_structure;
    // 滤波器序号, 0-27, 共28个滤波器
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    // 滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;

    if ((Object_Para & 0x02))
    {
        // 29位 拓展帧
        //  32位滤波
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
        // 验证码 高16bit
        can_filter_init_structure.FilterIdHigh = (ID << 3) >> 16;
        // 验证码 低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | (Object_Para & 0x03) << 1;
        // 屏蔽码 高16bit
        can_filter_init_structure.FilterMaskIdHigh = (Mask_ID << 3) >> 16;
        // 屏蔽码 低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (0x03) << 1;
    }
    else
    {
        // 11位 标准帧
        //  32位滤波
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_16BIT;
        // 标准帧验证码 高16bit不启用
        can_filter_init_structure.FilterIdHigh = 0x0000;
        // 验证码 低16bit
        can_filter_init_structure.FilterIdLow = ID << 5 | (Object_Para & 0x02) << 4;
        // 标准帧屏蔽码 高16bit不启用
        can_filter_init_structure.FilterMaskIdHigh = 0x0000;
        // 屏蔽码 低16bit
        can_filter_init_structure.FilterMaskIdLow = (Mask_ID << 5) | 0x01 << 4;
    }

    // 滤波器绑定FIFO0或FIFO1
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    // 从机模式选择开始单元 , 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.SlaveStartFilterBank = 14;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;

    // 过滤器配置
    if (HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        //         can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x200 ,0x7F8);  //只接收0x200-0x207
        //         can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x200, 0x7F8);
        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0); // 只接收
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    /*离开初始模式*/
    HAL_CAN_Start(hcan);

    /*开中断*/
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // can 接收fifo 0不为空中断
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // can 接收fifo 1不为空中断
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    // 检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    if (HAL_ERROR == HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox))
        return HAL_ERROR;
    else
        return HAL_OK;
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback()
{

#ifdef CHASSIS

    static uint8_t mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;
        // CAN1超级电容
        CAN_Send_Data(&hcan1, 0x66, CAN_Supercap_Tx_Data, 8);
        CAN_Send_Data(&hcan2, 0x88, CAN2_Chassis_Tx_Gimbal_Data, 8);
        CAN_Send_Data(&hcan2, 0x89, CAN2_Chassis_Tx_Gimbal_Data_1, 8);
    }

    // CAN1总线  四个底盘电机
    //  memset(CAN1_0x200_Tx_Data, 0, sizeof(CAN1_0x200_Tx_Data));
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    // 上板
    //  memset(CAN2_Chassis_Tx_Gimbal_Data, 0, sizeof(CAN2_Chassis_Tx_Gimbal_Data));

#elif defined(GIMBAL)
    static int mod = 0;
    mod++;
    if (mod == 10)
    {
        mod = 0;
    }
    // CAN1
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8); // 摩擦轮 按照0x200 ID 发送 可控制多个电机
    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8); // pitch

    // CAN2
    if (mod % 2 == 0)
    {
        CAN_Send_Data(&hcan1, 0xa0, CAN1_MiniPc_Tx_Data, 8); // 上位机
		
        CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8); // yaw-GM6020  按照0x1ff ID 发送 可控制多个电机
        CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8); // 拨弹轮
    }
    if (mod % 5 == 0)
    {
        CAN_Send_Data(&hcan2, 0x77, CAN2_Gimbal_Tx_Chassis_Data, 8); // 给底盘发送控制命令 按照0x77 ID 发送
        CAN_Send_Data(&hcan2, 0x78, CAN2_Gimbal_Tx_Chassis_Data_1, 8);
    }

#endif
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
