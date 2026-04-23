#include "dvc_ws2812.h"

// 定义颜色的GRB值
uint8_t Color_RED[3]= {0,255,0};
uint8_t Color_BLUE[3]= {0,0,255};
uint8_t Color_GREEN[3]= {255,0,0};
uint8_t Color_None[3]= {0,0,0};

// 初始化WS2812 LED驱动
void Class_WS2812::Init(TIM_HandleTypeDef *__Driver_PWM_TIM,uint8_t __Driver_PWM_TIM_Channel)
{
    Driver_PWM_TIM  =   __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel  =   __Driver_PWM_TIM_Channel;
    HAL_TIM_PWM_Start_DMA(Driver_PWM_TIM,Driver_PWM_TIM_Channel,(uint32_t*)&Send_Buffer,Length);
}

// 重新加载WS2812 LED数据
void Class_WS2812::Load(void)
{
    HAL_TIM_PWM_Start_DMA(Driver_PWM_TIM,Driver_PWM_TIM_Channel,(uint32_t*)&Send_Buffer,Length);
}

// 关闭所有LED
void Class_WS2812::CloseAll(void)
{
    uint16_t i;
    for ( i = 0; i < LED_Num*24; i++)
    {
        Send_Buffer[i]  =   WS2812_0_Pulse;
    }
    for ( i = LED_Num*24; i < Buffer_Length; i++)
    {
        Send_Buffer[i]  =   0;
    }
    Load();
}

// 写入单个LED的颜色和亮度
// __led_num:LED编号
// __color:颜色
// __brightness:亮度
void Class_WS2812::WriteOne(uint32_t __led_num,uint8_t* __color,float __brightness)
{
    uint32_t i,j;
    uint8_t dat[24];
    uint8_t color[3];
    for (i = 0; i <3; i++)
    {
        color[i] = __color[i] * __brightness;
    }
    for (i = 0; i < 8; i++)
    {
        dat[i] = ((color[0] & 0x80) ? WS2812_1_Pulse : WS2812_0_Pulse);
        color[0] <<= 1;
    }
    for (i = 0; i < 8; i++)
    {
        dat[i + 8] = ((color[1] & 0x80) ? WS2812_1_Pulse : WS2812_0_Pulse);
        color[1] <<= 1;
    }
    for (i = 0; i < 8; i++)
    {
        dat[i + 16] = ((color[2] & 0x80) ? WS2812_1_Pulse : WS2812_0_Pulse);
        color[2] <<= 1;
    }
    for (j = 0; j < 24; j++)
    {
        Send_Buffer[__led_num * 24 + j] = dat[j];
    }
}

// 写入所有LED的颜色和亮度
// __color:颜色
// __brightness:亮度
void Class_WS2812::WriteAll(uint8_t* __color,float __brightness)
{
    uint32_t i;
    for (i = 0; i < LED_Num; i++)
    {
        WriteOne(i, __color,__brightness);
    }
    for (i = LED_Num * 24; i < Buffer_Length; i++)
        Send_Buffer[i] = 0; // 占空比为0，全为低电平
    Load();
}

// 实现呼吸灯效果
// __color:呼吸灯颜色
void Class_WS2812::Breathing(uint8_t* __color)
{
    static uint32_t cnt_tick = 0;
    static bool cnt_flag = 0;
    if (cnt_flag == 0)
    {
        if (cnt_tick < 255)
        {
            cnt_tick++;
        }
        else
        {
            cnt_flag = 1;
        }
    }
    else
    {
        if (cnt_tick > 0)
        {
            cnt_tick--;
        }
        else
        {
            cnt_flag = 0;
        }
    }

    WriteAll(__color, cnt_tick / 255.0);
}

// 实现流水灯效果
// __led_num:流水灯数量
// __color:流水灯颜色
void Class_WS2812::WaterFall(uint32_t __led_num,uint8_t* __color)
{
    uint32_t i;
    static uint32_t cnt_fall_start = 0;
    static uint32_t cnt_fall_end = 0;
    cnt_fall_end = cnt_fall_start + __led_num;
    if(cnt_fall_start >= LED_Num)
    {
        cnt_fall_start -= LED_Num;
    }
    if(cnt_fall_end >= LED_Num)
    {
        cnt_fall_end -= LED_Num;
    }
    if(cnt_fall_end > cnt_fall_start)
    {
        for ( i = 0; i < cnt_fall_start; i++)
        {
            WriteOne(i, Color_None, 1.f);
        }
        for (i = cnt_fall_start; i < cnt_fall_end; i++)
        {
            WriteOne(i, __color, 1.f);
        }
        for ( i = cnt_fall_end; i < LED_Num; i++)
        {
            WriteOne(i, Color_None, 1.f);
        }
    }
    else
    {
        for ( i = 0; i < cnt_fall_end; i++)
        {
            WriteOne(i, __color, 1.f);
        }
        for ( i = cnt_fall_end; i < cnt_fall_start; i++)
        {
            WriteOne(i, Color_None, 1.f);
        }
        for ( i = cnt_fall_start; i < LED_Num; i++)
        {
            WriteOne(i, __color, 1.f);
        }
    }
    cnt_fall_start++;
    Load();
}