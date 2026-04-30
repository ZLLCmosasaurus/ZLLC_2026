
#include <stdint.h>
#include <string.h>
#include "dvc_dwt.h"

#define FRAME_HEADER_LENGTH 5
#define CMD_ID_LENGTH 2
#define CUSTOM_CONTROLLER_DATA_LENGTH 30
#define KEYBOARD_DATA_LENGTH 12

// 自定义控制器数据格式
struct Struct_Custom_Controller_Data
{
    float Angle[6];
    bool gripper_status;
} __attribute__((packed));

// 图传链路键鼠缓冲区数据格式
struct Struct_Image_Keyborad_UART_Data
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} __attribute__((packed));

class Class_Custom_Controller
{
public:
    // 对外数据接口
    Struct_Custom_Controller_Data Custom_Controller_Data;
    // 键鼠数据
    Struct_Image_Keyborad_UART_Data Image_Keyborad_Data;
    // 数据解包函数
    void Custom_Controller_Data_Process(uint8_t *Rx_Data);
    void Image_Keyboard_Data_Process(uint8_t *Rx_Data);

private:
    // 自定义控制器数据解包缓冲区
    uint8_t Data_Buffer[CUSTOM_CONTROLLER_DATA_LENGTH];
    // 接收频率计数器
    uint32_t frequence_cnt = 0;
    float rx_frequence = 0;
};