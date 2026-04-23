#ifndef _Blue_H
#define _Blue_H
#include "main.h"
//按键开关位置
#define KEY_FREE (0)
#define KEY_PRESSED (1)
enum Enum_Key_Status 
{
    Key_Status_FREE = 0,           //松开状态
    Key_Status_PRESSED,            //按下状态
    Key_Status_TRIG_FREE_PRESSED,  //松开到按下的突变状态
    Key_Status_TRIG_PRESSED_FREE,  //按下到松开的突变状态
};
struct Struct_Bluetooth_RxData
{
    uint16_t Frame_header;
    uint8_t Frame_Length;
    uint8_t Frame_number;
    int8_t Remote_Left_X;
    int8_t Remote_Left_Y;
    int8_t Remote_Right_X;
    int8_t Remote_Right_Y;
    uint8_t Button_One;
    uint8_t Button_Two;
    uint8_t _CRC;//累加和
}__attribute__((packed));

struct Struct_Bluetooth_PorcessData
{
    float Remote_Left_X;
    float Remote_Left_Y;
    float Remote_Right_X;
    float Remote_Right_Y;
    enum Enum_Key_Status Key[16];
};
#define Bluetooth_length 11

extern void Bluetooth_Data_Process(uint8_t *data);
extern void Key_Scan();

extern uint8_t Bluetooth_RxData[Bluetooth_length];
extern struct Struct_Bluetooth_RxData RxData;
extern struct Struct_Bluetooth_PorcessData PorcessData;
#endif

