/**
 * @file drv_dwt.h
 * @author lez by yssickjgd
 * @brief DWT初始化与配置流程
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);

extern DWT_Time_t SysTime;


#ifdef __cplusplus
}
#endif

#endif /* BSP_DWT_H_ */