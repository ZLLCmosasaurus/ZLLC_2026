#include "drv_rs485.h"
#include "ita_chariot.h"
#include "tsk_config_and_callback.h"
#include "config.h"

extern Class_Chariot chariot;

// 放在 D2 指向的内存区（如果 DMA 无法访问 DTCM 请开启此项）
DMA_BUFFER_ALIGN uint8_t rs485_rx_buf[RS485_RX_SIZE];

/**
 * @brief 初始化 RS485 接收
 */
void RS485_Init(void) {
    // 开启“接收到空闲为止”的 DMA 模式
    // 这种模式下，HAL 会在收到 IDLE 信号时自动产生回调
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rs485_rx_buf, RS485_RX_SIZE);
    
    // 禁用 DMA 的半传输中断，防止一包数据收一半就进回调
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

/**
 * @brief DMA 发送封装
 */
void RS485_Send_DMA(uint8_t *pData, uint16_t len) {
    // H7 必须：在发送前手动刷 Cache，确保 DMA 拿到的内存数据是最新的
    SCB_CleanDCache_by_Addr((uint32_t *)pData, len);
    HAL_UART_Transmit_DMA(&huart2, pData, len);
}

//-----------------------------------------------------------------------------------
/*此部分代码在drv_uart.cpp实现 */
// /**
//  * @brief HAL库 扩展接收回调函数
//  * 只要串口收到空闲信号（一包发完），HAL库的中断处理程序会自动调用这个函数
//  * 它是弱定义的，我们在这里重写它
//  */
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//     if (huart->Instance == USART2) {
//         // H7 必须：接收后失效 Cache，确保 CPU 读取的是 DMA 搬回来的新数据
//         SCB_InvalidateDCache_by_Addr((uint32_t *)rs485_rx_buf, RS485_RX_SIZE);
        
//         // 调用我们自己的处理逻辑
//         RS485_Receive_Handler(rs485_rx_buf, Size);
        
//         // 处理完后，重新开启接收（如果是循环模式则不需要，但为了严谨通常重新开启）
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rs485_rx_buf, RS485_RX_SIZE);
//         __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//     }
// }
//-----------------------------------------------------------------------------------


/**
 * @brief RS485的TIM定时器中断发送回调函数
 * 
 */
void TIM_RS485_PeriodElapsedCallback()
{
#if defined(GIMBAL)
    static uint8_t mod2 = 0, mod5 = 0, mod4 = 0, mod20 = 0,mod25 = 0,mod50 = 0;
    mod2++;
    mod5++;
    mod4++;
    mod20++;
	mod25++;
	mod50++;

    if(mod2 == 2) // 500Hz
    {
        mod2 = 0;
    }
    if (mod5 == 5) // 200Hz
    {
        mod5 = 0;
    }
    if (mod4 == 4) // 250Hz
    {
        mod4 = 0;
    }
    if (mod20 == 20) // 50Hz
    {
        mod20 = 0;
    }
	if (mod25 == 25) //40Hz
	{
        chariot.Booster.TensionMeter.Poll();
		mod25 = 0;
	}
	if(mod50 == 50) // 20Hz
	{
		mod50 = 0;
	}		
#endif
}
