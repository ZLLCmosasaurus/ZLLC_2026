/**
 * @file TensionMeter.h
 * @author mqx by
 * @brief GYSA tension meter
 * @version 0.1
 * @date 2026.1.29
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_TENSIONMETER_H
#define DVC_TENSIONMETER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

#ifndef MODBUS_READ_REG
#define MODBUS_READ_REG      0x03
#endif

#ifndef MODBUS_WRITE_REG
#define MODBUS_WRITE_REG     0x06
#endif

class Class_TensionMeter 
{
public:
    // 构造函数：初始化站地址（默认0x01）
    Class_TensionMeter(uint8_t id = 0x01);

    // --- 配置接口 ---
     /**
     * @brief 修改采样频率
     * @param rate 可选值: 10, 40, 640, 1280 (Hz)
     */
    void Set_SamplingRate(uint16_t rate);

    /**
     * @brief 修改波特率 (注意：修改后需断电重启，且驱动层波特率也需同步修改)
     * @param index 1=2400, 3=9600, 8=115200, 10=500000
     */
    void Set_BaudRate(uint8_t index);

    // --- 外部调度接口 ---
    void Poll();                                      // 发送读取请求（建议 20-50ms 调用一次）
    void Data_Process(uint8_t *pData, uint16_t len);  // 处理串口收到的原始数据

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    // --- 控制接口 ---
    void Set_Zero();                                  // 远程归零/去皮
    
    // --- 数据获取接口 ---
    float Get_Tension() { return Tension; }           // 获取拉力值
    bool Is_Online() { return (HAL_GetTick() - Last_Update_Time < 200); } // 离线检测

private:

    uint8_t  Station_ID;        // 站地址
    float    Tension;           // 转换后的拉力值
    int32_t  Raw_Value;         // 原始16进制数值
    uint32_t Last_Update_Time;  // 记录最后一次更新的时间戳

    uint8_t Tx_Buffer[16]; // 发送缓冲区

    // --- 内部协议工具 ---
    uint16_t CRC16(uint8_t *ptr, uint16_t len);
};


/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
