/**
 * @file TOFSense.h
 * @author hsl
 * @brief Nooploop TOFSense lazer lidar
 * @version 0.1
 * @date 2024-12-18 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include "string.h"
/* Private macros ------------------------------------------------------------*/
typedef struct {
  uint8_t id;               // ID
  uint32_t system_time;     // 系统上电至此帧的时间
  float dis;                // 测距值
  uint8_t dis_status;       // 距离状态指示
  uint16_t signal_strength; // 信号强度
  uint8_t range_precision;  // 仅tofsense-f/fP型号有效
} TOFSense_Rx_Data;

// 测距模块通信在线状态
enum TOFSense_Comm_Status
{
    TOFSense_Comm_OFFLINE,
    TOFSense_Comm_ONLINE
};

/* Private types -------------------------------------------------------------*/

class Class_TOFSense
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __dvc_id);

    void TOFSense_UART_RxCplt_Callback(uint8_t *Rx_Data, uint16_t Length);

    void TIM1msMod150_Alive_PeriodElapsedCallback();

    // 查询模式下使用
    void TIM_UART_Tx_PeriodElapsedCallback();

    inline float Get_Now_Distance();

private:

    // 绑定的UART结构体
    Struct_UART_Manage_Object *UART_Manage_Object;
    // 测距模块设置的编号，默认为0
    uint8_t dvc_id = 0;
    // 测距模块返回的数据
    TOFSense_Rx_Data TOFSense_Data;

    
    // 当前时刻通信flag
    uint32_t Flag = 0;
    // 前一时刻通信flag
    uint32_t Pre_Flag = 0;
    // 测距模块在线状态
    TOFSense_Comm_Status TOFSense_Communication_Status = TOFSense_Comm_OFFLINE;

    void Data_Process(uint8_t* Frame_Ptr);
};

inline float Class_TOFSense::Get_Now_Distance()
{
    return TOFSense_Data.dis;
}

/************************ COPYRIGHT(C) NEUQ-ZLLC **************************/
