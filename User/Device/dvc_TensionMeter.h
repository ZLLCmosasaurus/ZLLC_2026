/**
 * @file TensionMeter.h
 * @author lez
 * @brief HX711  tension meter
 * @version 0.1
 * @date 2024-12-18 0.1 24赛季定稿
 *
 * @copyright ZLLC 2025
 *
 */

#ifndef DVC_TENSIONMETER_H
#define DVC_TENSIONMETER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

// 卡尔曼滤波器结构体
typedef struct 
{
    float x;  // 当前估计值
    float P;  // 误差协方差
    float K;  // 卡尔曼增益
} KalmanFilter;

/* 用于保存HX711的增益 */
enum Enum_HX711_Gain{
    GAIN_128 = 1,
    GAIN_32,
    GAIN_64,
};
 
/* 结构体 用于实现HX711本身的属性 */
struct Struct_HX711_Data{
    Enum_HX711_Gain gain; /* 增益 */
    bool isTare;    /* 判断传感器数据是否是皮重 true-是 false-否 */
    float k;        /* 比例系数 */
    float b;        /* 补偿值 */
    float tare;     /* 皮重 */
    float actual;   /* 实重 */
	float tare_get_data[30];
};

class Class_TensionMeter
{
public:
    void Init(GPIO_TypeDef *_Gpio_Data, uint16_t _Pin_Data, GPIO_TypeDef * _Gpio_Clk, uint16_t _Pin_Clk);

    void TensionMeter_Cal(void);

    inline void Set_Ratio(float ratio);
    inline void Set_OffsetValue(float offset);/* 设置传感器误差校正/补偿值 */
    inline void Set_Gain(Enum_HX711_Gain gain);/* 设置传感器增益 */

    inline float Get_Tension_Meter(void);
    inline float Get_Tare(void);
    inline uint32_t Get_Raw_Data(void);

protected:

    GPIO_TypeDef *Gpio_Data;
    GPIO_TypeDef *Gpio_Clk;
    uint16_t Pin_Data;
    uint16_t Pin_Clk;

    // 卡尔曼滤波参数
    float Q = 0.01f; // 过程噪声协方差
    float R = 1.0f;  // 观测噪声协方差
    float A = 1.0f;  // 状态转移矩阵
    float H = 1.0f; // 观测矩阵

    // 传感器值转实际重量 系数
    float Data_To_Weigth = 23.91304347f;

    // 卡尔曼滤波器
    KalmanFilter Kf;

    // HX711 结构体
    Struct_HX711_Data HX711;

    // 测量值
    float Measurement;
    // 滤波输出
    float Tension_Value;
    // 原始数据
    uint32_t Raw_Data;


    uint32_t init_data_cal;
    uint8_t init_cnt;

    // 读取HX711原始数据 中断中调用
    void Read_Raw_Data_FromISR();
    // 读取HX711原始数据 阻塞方式
    void Read_Raw_Data();
    
    // 初始化卡尔曼滤波器
    void kalman_init(float initial_value);
    // 卡尔曼滤波更新函数
    void kalman_update();

    // 计算实际重量
    void Cal_Actual_Weight();

    // 读取数据引脚
    GPIO_PinState READ_DT();
    // 拉高数据引脚
    void SCK_HIGH();
    // 拉低数据引脚
    void SCK_LOW();
};
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/* 设置传感器转换系数 */
void Class_TensionMeter::Set_Ratio(float ratio)
{
    HX711.k = ratio;
}
 
/* 设置传感器误差校正/补偿值 */
void Class_TensionMeter::Set_OffsetValue(float offset)
{
    HX711.b = offset;
}
 
/* 设置传感器增益 */
void Class_TensionMeter::Set_Gain(Enum_HX711_Gain gain)
{
    HX711.gain = gain;
}

float Class_TensionMeter::Get_Tension_Meter(void)
{
    return (Tension_Value);
}

float Class_TensionMeter::Get_Tare(void)
{
    return (HX711.tare);
}

uint32_t Class_TensionMeter::Get_Raw_Data(void)
{
    return (Raw_Data);
}
/* Function prototypes -------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
