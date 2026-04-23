#ifndef DVC_WS2812
#define DVC_WS2812

#include "drv_tim.h"
#include "arm_math.h"

/*
本文档基于STM32 HAL库编写，用于驱动WS2812 LED灯带
开发板：RoboMaster STM32F407开发板
使用TIM1_CH1输出PWM波形，波形频率为800KHz
CUBEMX食用方法：
1. 配置TIM1_CH1输出PWM波形，波形频率为800KHz
2. 配置TIM1_CH1的DMA传输，采用normal模式，传输长度为half word，传输方向为memory to peripheral
CPP食用指南：
1.根据实际灯珠数量修改LED_Num的值
2.根据所选arr修改WS2812_1_Pulse以及WS2812_0_Pulse的值
3.调用Init函数初始化WS2812 LED驱动
4.调用WriteAll/WaterFall/Breathing函数写入所有LED的颜色和亮度
*/

// 定义LED数量和缓冲区长度
#define LED_Num 64
#define Reset_Buffer_Length 230        // Reset 280us/1.25us=224
#define Buffer_Length (LED_Num*24+Reset_Buffer_Length)

// 定义WS2812的脉冲宽度
#define WS2812_1_Pulse  162
#define WS2812_0_Pulse  49

// 声明颜色数组
extern uint8_t Color_RED[3];
extern uint8_t Color_BLUE[3];
extern uint8_t Color_GREEN[3];

class Class_WS2812
{
    public:
        // 初始化WS2812 LED驱动
        void Init(TIM_HandleTypeDef *__Driver_PWM_TIM,uint8_t __Driver_PWM_TIM_Channel);
        
        // 重新加载WS2812 LED数据
        void Load(void);
        
        // 关闭所有LED
        void CloseAll(void);
        
        // 写入所有LED的颜色和亮度
        void WriteAll(uint8_t* __color,float __brightness);
        
        // 写入单个LED的颜色和亮度
        void WriteOne(uint32_t __led_num,uint8_t* __color,float __brightness);
        
        // 实现流水灯效果
        void WaterFall(uint32_t __led_num,uint8_t* __color);
        
        // 实现呼吸灯效果
        void Breathing(uint8_t* __color);
        
    protected:
        TIM_HandleTypeDef *Driver_PWM_TIM; // PWM定时器句柄
        uint8_t Driver_PWM_TIM_Channel;    // PWM定时器通道
        uint16_t Length = Buffer_Length;   // 缓冲区长度
        uint16_t Send_Buffer[Buffer_Length]; // 发送缓冲区
};

#endif // !DVC_WS2812