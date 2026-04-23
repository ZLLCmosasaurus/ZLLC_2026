/**
 * @file drv_spi.cpp
 * @author lez by yssickjgd
 * @brief SPI通信初始化与配置流程
 * @version 1.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_spi.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_SPI_Manage_Object SPI1_Manage_Object = {0};
Struct_SPI_Manage_Object SPI2_Manage_Object = {0};
Struct_SPI_Manage_Object SPI3_Manage_Object = {0};
Struct_SPI_Manage_Object SPI4_Manage_Object = {0};
Struct_SPI_Manage_Object SPI5_Manage_Object = {0};
Struct_SPI_Manage_Object SPI6_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化SPI
 *
 * @param hspi UART编号
 * @param Callback_Function 处理回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back Callback_Function)
{
    if (hspi->Instance == SPI1)
    {
        SPI1_Manage_Object.SPI_Handler = hspi;
        SPI1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (hspi->Instance == SPI2)
    {
        SPI2_Manage_Object.SPI_Handler = hspi;
        SPI2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (hspi->Instance == SPI3)
    {
        SPI3_Manage_Object.SPI_Handler = hspi;
        SPI3_Manage_Object.Callback_Function = Callback_Function;
    }
//    else if (hspi->Instance == SPI4)
//    {
//        SPI4_Manage_Object.SPI_Handler = hspi;
//        SPI4_Manage_Object.Callback_Function = Callback_Function;
//    }
//    else if (hspi->Instance == SPI5)
//    {
//        SPI5_Manage_Object.SPI_Handler = hspi;
//        SPI5_Manage_Object.Callback_Function = Callback_Function;
//    }
//    else if (hspi->Instance == SPI6)
//    {
//        SPI6_Manage_Object.SPI_Handler = hspi;
//        SPI6_Manage_Object.Callback_Function = Callback_Function;
//    }
}

/**
 * @brief 交互数据帧
 * 
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t SPI_Send_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t Tx_Length, uint16_t Rx_Length)
{
    //HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);

    if (hspi->Instance == SPI1)
    {
        SPI1_Manage_Object.Now_GPIOx = GPIOx;
        SPI1_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI1_Manage_Object.Now_Tx_Length = Tx_Length;
        SPI1_Manage_Object.Now_Rx_Length = Rx_Length;
		
		return (HAL_SPI_TransmitReceive(hspi, SPI1_Manage_Object.Tx_Buffer, SPI1_Manage_Object.Rx_Buffer, 1, 1000));
        //return (HAL_SPI_TransmitReceive_DMA(hspi, SPI1_Manage_Object.Tx_Buffer, SPI1_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
    }
    else if (hspi->Instance == SPI2)
    {
        SPI2_Manage_Object.Now_GPIOx = GPIOx;
        SPI2_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI2_Manage_Object.Now_Tx_Length = Tx_Length;
        SPI2_Manage_Object.Now_Rx_Length = Rx_Length;
        
        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI2_Manage_Object.Tx_Buffer, SPI2_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
    }
    else if (hspi->Instance == SPI3)
    {
        SPI3_Manage_Object.Now_GPIOx = GPIOx;
        SPI3_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI3_Manage_Object.Now_Tx_Length = Tx_Length;
        SPI3_Manage_Object.Now_Rx_Length = Rx_Length;
        
        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI3_Manage_Object.Tx_Buffer, SPI3_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
    }
    else
	    return 0;
//    else if (hspi->Instance == SPI4)
//    {
//        SPI4_Manage_Object.Now_GPIOx = GPIOx;
//        SPI4_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
//        SPI4_Manage_Object.Now_Tx_Length = Tx_Length;
//        SPI4_Manage_Object.Now_Rx_Length = Rx_Length;
//        
//        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI4_Manage_Object.Tx_Buffer, SPI4_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
//    }
//    else if (hspi->Instance == SPI5)
//    {
//        SPI5_Manage_Object.Now_GPIOx = GPIOx;
//        SPI5_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
//        SPI5_Manage_Object.Now_Tx_Length = Tx_Length;
//        SPI5_Manage_Object.Now_Rx_Length = Rx_Length;
//        
//        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI5_Manage_Object.Tx_Buffer, SPI5_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
//    }
//    else if (hspi->Instance == SPI6)
//    {
//        SPI6_Manage_Object.Now_GPIOx = GPIOx;
//        SPI6_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
//        SPI6_Manage_Object.Now_Tx_Length = Tx_Length;
//        SPI6_Manage_Object.Now_Rx_Length = Rx_Length;
//        
//        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI6_Manage_Object.Tx_Buffer, SPI6_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
//    }
}

/**
 * @brief SPI的TIM定时器中断交互回调函数
 * 
 */
void TIM_SPI_PeriodElapsedCallback()
{

}

/**
 * @brief HAL库SPI交互DMA中断
 * 
 * @param hspi SPI编号
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // //判断程序初始化完成
    // if(init_finished == 0)
    // {
    //     return;
    // }
    
    //选择回调函数
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_TogglePin(SPI1_Manage_Object.Now_GPIOx, SPI1_Manage_Object.Now_GPIO_Pin);
        
        SPI1_Manage_Object.Callback_Function(SPI1_Manage_Object.Tx_Buffer, SPI1_Manage_Object.Rx_Buffer, SPI1_Manage_Object.Now_Tx_Length);
    }
    else if (hspi->Instance == SPI2)
    {
        HAL_GPIO_TogglePin(SPI2_Manage_Object.Now_GPIOx, SPI2_Manage_Object.Now_GPIO_Pin);
        
        SPI2_Manage_Object.Callback_Function(SPI2_Manage_Object.Tx_Buffer, SPI2_Manage_Object.Rx_Buffer, SPI2_Manage_Object.Now_Tx_Length);
    }
    else if (hspi->Instance == SPI3)
    {
        HAL_GPIO_TogglePin(SPI3_Manage_Object.Now_GPIOx, SPI3_Manage_Object.Now_GPIO_Pin);
        
        SPI3_Manage_Object.Callback_Function(SPI3_Manage_Object.Tx_Buffer, SPI3_Manage_Object.Rx_Buffer, SPI3_Manage_Object.Now_Tx_Length);
    }
//    else if (hspi->Instance == SPI4)
//    {
//        HAL_GPIO_TogglePin(SPI4_Manage_Object.Now_GPIOx, SPI4_Manage_Object.Now_GPIO_Pin);
//        
//        SPI4_Manage_Object.Callback_Function(SPI4_Manage_Object.Tx_Buffer, SPI4_Manage_Object.Rx_Buffer, SPI4_Manage_Object.Now_Tx_Length);
//    }
//    else if (hspi->Instance == SPI5)
//    {
//        HAL_GPIO_TogglePin(SPI5_Manage_Object.Now_GPIOx, SPI5_Manage_Object.Now_GPIO_Pin);
//        
//        SPI5_Manage_Object.Callback_Function(SPI5_Manage_Object.Tx_Buffer, SPI5_Manage_Object.Rx_Buffer, SPI5_Manage_Object.Now_Tx_Length);
//    }
//    else if (hspi->Instance == SPI6)
//    {
//        HAL_GPIO_TogglePin(SPI6_Manage_Object.Now_GPIOx, SPI6_Manage_Object.Now_GPIO_Pin);
//        
//        SPI6_Manage_Object.Callback_Function(SPI6_Manage_Object.Tx_Buffer, SPI6_Manage_Object.Rx_Buffer, SPI6_Manage_Object.Now_Tx_Length);
//    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
