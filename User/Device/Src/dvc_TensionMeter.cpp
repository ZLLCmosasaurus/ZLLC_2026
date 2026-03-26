/**
 * @file TensionMeter.cpp
 * @author mqx by
 * @brief GYSA tension meter
 * @version 0.1
 * @date 2026.1.29
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_TensionMeter.h"
#include "drv_rs485.h"
#include <string.h> 


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
uint8_t test_meter_tension[4];
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 构造函数
 */
Class_TensionMeter::Class_TensionMeter(uint8_t id) {
    Station_ID = id;
    Tension = 0.0f;
    Raw_Value = 0;
    Last_Update_Time = 0;
}

/**
 * @brief 定时轮询，发送 Read Holding Registers 指令 (0x03)
 * 读取地址 0x0000 开始的 2 个寄存器 (共4字节)
 */
void Class_TensionMeter::Poll() {
    // 关键：改用成员变量 Tx_Buffer
    Tx_Buffer[0] = Station_ID;
    Tx_Buffer[1] = 0x03; 
    Tx_Buffer[2] = 0x00; Tx_Buffer[3] = 0x00; 
    Tx_Buffer[4] = 0x00; Tx_Buffer[5] = 0x02; 

    uint16_t crc = CRC16(Tx_Buffer, 6);
    Tx_Buffer[6] = crc & 0xFF;
    Tx_Buffer[7] = (crc >> 8) & 0xFF;

    // 此刻发送的是持久内存地址，函数执行完内存也不会消失
    RS485_Send_DMA(Tx_Buffer, 8);
}

float Filter_K = 0.15f; // 滤波系数
/**
 * @brief 核心解析逻辑（增加偏移扫描，抗干扰）
 */
void Class_TensionMeter::Data_Process(uint8_t *pData, uint16_t len) 
{
    if (len < 9) return;

    uint16_t i = 0;
    while (i <= len - 9) 
    {
        if (pData[i] == Station_ID && pData[i+1] == 0x03 && pData[i+2] == 0x04) 
        {
            uint16_t rcv_crc = (pData[i + 8] << 8) | pData[i + 7];
            uint16_t cal_crc = CRC16(&pData[i], 7);

            if (rcv_crc == cal_crc) 
            {
                uint8_t b1 = pData[i+3]; 
                uint8_t b2 = pData[i+4]; 
                uint8_t b3 = pData[i+5]; 
                uint8_t b4 = pData[i+6]; 

                //顺序组合成一个32位整数（注意字节顺序）
                Raw_Value = (int32_t)((b1 << 24) | (b2 << 16) | (b3 << 8) | b4);
                // 转换为实际拉力值，单位g（根据测试出来的比例系数） 可能不准 暂时用这个
                float Current_Raw = (float)Raw_Value * 0.00015874f; 

                // --- 3阶中值滤波 (静态变量实现) ---
                static float buf[3] = {0};
                static uint8_t idx = 0;
                
                // 给第一次初始化赋初值，防止开机从0爬升
                if(buf[0]==0 && buf[1]==0 && buf[2]==0) {
                     buf[0]=buf[1]=buf[2] = Current_Raw;
                }

                buf[idx] = Current_Raw;
                idx = (idx + 1) % 3;

                // 找中值 (冒泡排序太麻烦，3个数直接比)
                float mid;
                float a = buf[0], b = buf[1], c = buf[2];
                if ((a <= b && b <= c) || (c <= b && b <= a)) mid = b;
                else if ((b <= a && a <= c) || (c <= a && a <= b)) mid = a;
                else mid = c;

                // --- 进低通 ---
                if(abs(Tension) < 0.001) Tension = mid;
                Tension = Filter_K * mid + (1.0f - Filter_K) * Tension;

                Last_Update_Time = HAL_GetTick();
            }
            i += 9; 
        } 
        else 
        {
            i++;
        }
    }
}

/**
 * @brief 串口接收完成后的入口（这个函数是底层驱动直接调用的）
 */
void Class_TensionMeter::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 这里直接调用 Data_Process 逻辑
    this->Data_Process(Rx_Data, Length);
}

/**
 * @brief 远程归零 (写入 01 到 0x0016)
 */
void Class_TensionMeter::Set_Zero() {
    // 工业设备通常需要先写 0x0017 为 1 (解锁)
    uint8_t unlock_cmd[8] = {Station_ID, 0x06, 0x00, 0x17, 0x00, 0x01, 0, 0};
    uint16_t crc1 = CRC16(unlock_cmd, 6);
    unlock_cmd[6] = crc1 & 0xFF; unlock_cmd[7] = (crc1 >> 8) & 0xFF;
    RS485_Send_DMA(unlock_cmd, 8);
    
    HAL_Delay(5); // 稍微等待总线空闲
    
    // 发送归零命令
    uint8_t zero_cmd[8] = {Station_ID, 0x06, 0x00, 0x16, 0x00, 0x01, 0, 0};
    uint16_t crc2 = CRC16(zero_cmd, 6);
    zero_cmd[6] = crc2 & 0xFF; zero_cmd[7] = (crc2 >> 8) & 0xFF;
    RS485_Send_DMA(zero_cmd, 8);
}

/**
 * @brief Modbus 标准 CRC16 算法
 */
uint16_t Class_TensionMeter::CRC16(uint8_t *ptr, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 修改采样速率 (寄存器 0x000E)
 */
void Class_TensionMeter::Set_SamplingRate(uint16_t rate) {
    // 1. 关闭写保护 (向 0x0017 写入 0x0001)
    uint8_t unlock_cmd[8] = {Station_ID, 0x06, 0x00, 0x17, 0x00, 0x01, 0, 0};
    uint16_t crc1 = CRC16(unlock_cmd, 6);
    unlock_cmd[6] = crc1 & 0xFF; unlock_cmd[7] = (crc1 >> 8) & 0xFF;
    RS485_Send_DMA(unlock_cmd, 8);
    
    HAL_Delay(10); // 等待变送器响应

    // 2. 写入采样频率 (例如 1280Hz 对应 05 00)
    uint8_t rate_cmd[8];
    rate_cmd[0] = Station_ID;
    rate_cmd[1] = 0x06;         // 写寄存器
    rate_cmd[2] = 0x00;         // 地址高位
    rate_cmd[3] = 0x0E;         // 地址低位 0x000E
    rate_cmd[4] = (uint8_t)(rate >> 8);
    rate_cmd[5] = (uint8_t)(rate & 0xFF);
    
    uint16_t crc2 = CRC16(rate_cmd, 6);
    rate_cmd[6] = crc2 & 0xFF; rate_cmd[7] = (crc2 >> 8) & 0xFF;
    RS485_Send_DMA(rate_cmd, 8);
}

/**
 * @brief 修改波特率 (寄存器 0x0010)
 */
void Class_TensionMeter::Set_BaudRate(uint8_t index) {
    // 1. 关闭写保护
    uint8_t unlock_cmd[8] = {Station_ID, 0x06, 0x00, 0x17, 0x00, 0x01, 0, 0};
    uint16_t crc1 = CRC16(unlock_cmd, 6);
    unlock_cmd[6] = crc1 & 0xFF; unlock_cmd[7] = (crc1 >> 8) & 0xFF;
    RS485_Send_DMA(unlock_cmd, 8);
    
    HAL_Delay(10);

    // 2. 写入波特率索引 (例如 115200 对应 8)
    uint8_t baud_cmd[8];
    baud_cmd[0] = Station_ID;
    baud_cmd[1] = 0x06;
    baud_cmd[2] = 0x00;
    baud_cmd[3] = 0x10;         // 地址 0x0010
    baud_cmd[4] = 0x00;
    baud_cmd[5] = index;
    
    uint16_t crc2 = CRC16(baud_cmd, 6);
    baud_cmd[6] = crc2 & 0xFF; baud_cmd[7] = (crc2 >> 8) & 0xFF;
    RS485_Send_DMA(baud_cmd, 8);
    
    // 注意：说明书提到修改波特率立即生效，需断电重启
}


/* Function prototypes -------------------------------------------------------*/



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
