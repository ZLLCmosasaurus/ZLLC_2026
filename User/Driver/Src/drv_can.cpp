/**
 * @file drv_can.cpp
 * @author cjw by yssickjgd
 * @brief CAN通信初始化与配置流程
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
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
Struct_CAN_Manage_Object CAN3_Manage_Object = {0};

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x1fe_Tx_Data[8];
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
uint8_t CAN2_0x1fe_Tx_Data[8];
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

uint8_t CAN3_0x1ff_Tx_Data[8];
uint8_t CAN3_0x1fe_Tx_Data[8];
uint8_t CAN3_0x200_Tx_Data[8];
uint8_t CAN3_0x2ff_Tx_Data[8];
uint8_t CAN3_0xxf1_Tx_Data[8];
uint8_t CAN3_0xxf2_Tx_Data[8];
uint8_t CAN3_0xxf3_Tx_Data[8];
uint8_t CAN3_0xxf4_Tx_Data[8];
uint8_t CAN3_0xxf5_Tx_Data[8];
uint8_t CAN3_0xxf6_Tx_Data[8];
uint8_t CAN3_0xxf7_Tx_Data[8];
uint8_t CAN3_0xxf8_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];
uint8_t CAN3_Chassis_Tx_Data_A[8];   //底盘给云台发送缓冲区
uint8_t CAN3_Chassis_Tx_Data_B[8];   //底盘给云台发送缓冲区
uint8_t CAN3_Chassis_Tx_Data_C[8];   //底盘给云台发送缓冲区
uint8_t CAN3_Chassis_Tx_Data_D[8];   //底盘给云台发送缓冲区
uint8_t CAN3_Chassis_Tx_Data_E[8];   //底盘给云台发送缓冲区
uint8_t CAN3_Chassis_Tx_Data_F[8];   //底盘给云台发送缓冲区
uint8_t CAN3_Chassis_Tx_Data_G[8];   //底盘给云台发送缓冲区
uint8_t CAN3_MiniPC_Tx_Data_A[8];   //下位机发送缓冲区
uint8_t CAN3_MiniPC_Tx_Data_B[8];   //下位机发送缓冲区
uint8_t CAN3_MiniPC_Tx_Data_C[8];   //下位机发送缓冲区
uint8_t CAN3_MiniPC_Tx_Data_D[8];   //下位机发送缓冲区
uint8_t CAN3_Gimbal_Tx_Chassis_Data[8];  //云台给底盘发送缓冲区
uint8_t CAN3_Sentry_CMD_Data[8];   //云台给底盘发送缓冲区

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

uint8_t CAN3_0x141_Tx_Data[8];
uint8_t CAN3_0x142_Tx_Data[8];
uint8_t CAN3_0x143_Tx_Data[8];
uint8_t CAN3_0x144_Tx_Data[8];
uint8_t CAN3_0x145_Tx_Data[8];
uint8_t CAN3_0x146_Tx_Data[8];
uint8_t CAN3_0x147_Tx_Data[8];
uint8_t CAN3_0x148_Tx_Data[8];

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
//void can_filter_mask_config(FDCAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
//{
//	
//    //检测传参是否正确
//    assert_param(hcan != NULL);

//	   //CAN过滤器初始化结构体
//    FDCAN_FilterTypeDef can_filter_init_structure;
//    //滤波器序号, 0-27, 共28个滤波器
//    can_filter_init_structure.FilterBank = Object_Para >> 3;
//    //滤波器模式，设置ID掩码模式
//    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
//    
//	
//    if ((Object_Para & 0x02))
//    {   
//        //29位 拓展帧
//			  // 32位滤波
//        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
//        //验证码 高16bit
//        can_filter_init_structure.FilterIdHigh = (ID << 3) >> 16;
//        //验证码 低16bit
//        can_filter_init_structure.FilterIdLow = ID << 3 | (Object_Para & 0x03) << 1;
//        //屏蔽码 高16bit
//        can_filter_init_structure.FilterMaskIdHigh = (Mask_ID << 3) >> 16;
//        //屏蔽码 低16bit
//        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (0x03) << 1 ;
//    }
//    else
//    {
//        //11位 标准帧
//			  // 32位滤波
//        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_16BIT;
//        //标准帧验证码 高16bit不启用
//        can_filter_init_structure.FilterIdHigh = 0x0000 ; 
//        //验证码 低16bit
//			  can_filter_init_structure.FilterIdLow =ID << 5 | (Object_Para & 0x02) << 4;  
//        //标准帧屏蔽码 高16bit不启用
//        can_filter_init_structure.FilterMaskIdHigh =  0x0000 ;
//        //屏蔽码 低16bit
//        can_filter_init_structure.FilterMaskIdLow =(Mask_ID << 5) | 0x01 << 4 ; 
//    }

//    //滤波器绑定FIFO0或FIFO1
//    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
//    //从机模式选择开始单元 , 前14个在CAN1, 后14个在CAN2
//    can_filter_init_structure.SlaveStartFilterBank = 14;
//    //使能滤波器
//    can_filter_init_structure.FilterActivation = ENABLE;

//    // 过滤器配置
//    if(HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure)!=HAL_OK)
//    {
//        Error_Handler();
//    }
//	
//}

/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter); 		 				  //接收ID2
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}


/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    FDCAN_FilterTypeDef fdcan_filter;
    if (hcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;	
	
        fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
        fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
        fdcan_filter.FilterID1 = 0x00;                               
        fdcan_filter.FilterID2 = 0x00;

        HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
        //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
        // HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
        // HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);			
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;

        fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
        fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;           //过滤器0关联到FIFO0  
        fdcan_filter.FilterID1 = 0x00;                               
        fdcan_filter.FilterID2 = 0x00;

        HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter); 		 				  //接收ID2
        //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
        // HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
        // HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN3_Manage_Object.CAN_Handler = hcan;
        CAN3_Manage_Object.Callback_Function = Callback_Function;

        fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
        fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
        fdcan_filter.FilterID1 = 0x00;                               
        fdcan_filter.FilterID2 = 0x00;

        HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter); 		 				  //接收ID2
        //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
        HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
    // can_filter_init(hcan);
    // HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(hcan); //开启FDCAN
    
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
uint8_t CAN_Send_Data(FDCAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=ID;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(Length<=8)
		pTxHeader.DataLength = Length;
	if(Length==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(Length==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(Length==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(Length==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(Length==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(Length==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(Length==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
    // if(HAL_FDCAN_GetTxFifoFreeLevel(hcan)!=0)
	return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &pTxHeader, Data);	
}
uint32_t ECR_value = FDCAN1->ECR;
int test = 0;
/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback()
{
    
    #ifdef CHASSIS
    static uint8_t mod5 = 0,mod100 = 0,mod20 = 0;
    mod5++, mod100++,mod20++;
    if (mod5 == 5)  //200Hz
    {
        mod5 = 0;
        //3508    
        CAN_Send_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, 8);
        #ifdef AGV
        //6020
        CAN_Send_Data(&hfdcan2, 0x1fe, CAN2_0x1fe_Tx_Data, 8);
        #endif
    }
    
    if (mod100 == 10) //10Hz
    {
        CAN_Send_Data(&hfdcan3, 0x191, CAN3_Chassis_Tx_Data_G, 8);
        mod100 = 0;
    }
    if (mod20 == 20) //50Hz
    {
        //上板
        CAN_Send_Data(&hfdcan3, 0x188, CAN3_Chassis_Tx_Data_A, 8);
        CAN_Send_Data(&hfdcan3, 0x199, CAN3_Chassis_Tx_Data_B, 8);
        CAN_Send_Data(&hfdcan3, 0x178, CAN3_Chassis_Tx_Data_C, 8);      
        CAN_Send_Data(&hfdcan3, 0x197, CAN3_Chassis_Tx_Data_E, 8);
        CAN_Send_Data(&hfdcan3, 0x198, CAN3_Chassis_Tx_Data_D, 8);
        CAN_Send_Data(&hfdcan3, 0x196, CAN3_Chassis_Tx_Data_F, 8);
        //超电
        CAN_Send_Data(&hfdcan2, 0x66, CAN_Supercap_Tx_Data, 8);
        mod20 = 0;
    }
    #elif defined (GIMBAL)

    static uint8_t mod5 = 0,mod4 = 0,mod20 = 0;
    mod5++;
    mod4++;
    mod20++;
    
    if(mod5 == 5)
    {
        mod5 = 0;
        CAN_Send_Data(&hfdcan2, 0x1fe, CAN2_0x1fe_Tx_Data, 8); //GM6020  按照0x1fe ID 发送 可控制多个电机
        CAN_Send_Data(&hfdcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8); //摩擦轮 按照0x1ff ID 发送 可控制多个电机

        //  CAN3  下板       
        CAN_Send_Data(&hfdcan3, 0x200, CAN3_0x200_Tx_Data, 8); //拨弹盘  按照0x200 ID 发送 可控制多个电机
        CAN_Send_Data(&hfdcan3, 0x77, CAN3_Gimbal_Tx_Chassis_Data, 8); //给底盘发送控制命令 按照0x77 ID 发送
        
    }
    if(mod4 == 4)
    {
        mod4 = 0;
    }   
    if (mod20 == 20) //50Hz
    {
        mod20 = 0;
    }
    #endif

}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
    //选择回调函数
    if (hcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == FDCAN3)
    {
        HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &CAN3_Manage_Object.Rx_Buffer.Header, CAN3_Manage_Object.Rx_Buffer.Data);
        CAN3_Manage_Object.Callback_Function(&CAN3_Manage_Object.Rx_Buffer);
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan,uint32_t RxFifo1ITs)
{
    //选择回调函数
    if (hcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == FDCAN3)
    {
        HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &CAN3_Manage_Object.Rx_Buffer.Header, CAN3_Manage_Object.Rx_Buffer.Data);
        CAN3_Manage_Object.Callback_Function(&CAN3_Manage_Object.Rx_Buffer);
    }
}
int ppp=0;
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfdcan);
  UNUSED(BufferIndexes);
	ppp++;
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_FDCAN_TxBufferCompleteCallback could be implemented in the user file
   */
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
