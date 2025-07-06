/**
 * @file drv_dwt.cpp
 * @author cjw by yssickjgd
 * @brief DWT初始化与配置流程
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#include "drv_dwt.h"

DWT_Time_t SysTime;
uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
uint32_t CYCCNT_RountCount;
uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;


void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  
    //Ennable write access  only M7 need this operateion
    #ifdef STM32H7
    DWT->LAR |= 0xC5ACCE55;//访问锁寄存器
    #endif
  
    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

