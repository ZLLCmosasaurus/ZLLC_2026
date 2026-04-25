#include <stdint.h>
#include <string.h>
#include "dvc_dwt.h"

#define CUSTOM_CONTROLLER_DATA_LENGTH 30

// 自定义控制器数据格式
struct Struct_Custom_Controller_Data
{
    float Angle[6];
    bool gripper_status;
} __attribute__((packed));


class Class_Custom_Controller
{
public:
    // 对外数据接口
    Struct_Custom_Controller_Data Custom_Controller_Data;
    // 数据解包函数
    void Custom_Controller_Data_Process(uint8_t* Rx_Data);
private:
    // 数据解包缓冲区
    uint8_t Data_Buffer[CUSTOM_CONTROLLER_DATA_LENGTH];
    // 接收频率计数器
    uint32_t frequence_cnt = 0;
    float rx_frequence = 0;
};