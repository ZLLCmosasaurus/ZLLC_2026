#ifndef DRV_RS485_H
#define DRV_RS485_H
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

#include "stdlib.h"
#include "string.h"
  
#ifdef STM32H723xx 
#include "stm32h7xx_hal.h"
#endif  
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif   

/* Exported macros -----------------------------------------------------------*/

#define RS485_RX_SIZE 128

// 为 H7 D-Cache 准备的对齐宏
#if defined ( __ICCARM__ )
#define DMA_BUFFER_ALIGN  _Pragma("data_alignment=32")
#else
#define DMA_BUFFER_ALIGN  __attribute__((aligned(32)))
#endif

extern DMA_BUFFER_ALIGN uint8_t rs485_rx_buf[RS485_RX_SIZE];

void RS485_Init(void);
void RS485_Send_DMA(uint8_t *pData, uint16_t len);

// 这个函数可以由你在 tsk_config_and_callback.cpp 中重新实现逻辑
void RS485_Receive_Handler(uint8_t *pData, uint16_t len);
void TIM_RS485_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

#endif
