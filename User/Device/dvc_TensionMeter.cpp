/**
 * @file TensionMeter.cpp
 * @author lez
 * @brief HX711  tension meter
 * @version 0.1
 * @date 2024-12-18 0.1 24赛季定稿
 *
 * @copyright ZLLC 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_TensionMeter.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

void Class_TensionMeter::TensionMeter_Cal(void)
{
    if(HX711.isTare == false) { /* 当该标志位复位 代表此时应计算皮重 */
        Read_Raw_Data();
        HX711.tare_get_data[init_cnt] = (float)Raw_Data * HX711.k + HX711.b;
        init_data_cal += (HX711.tare_get_data[init_cnt] - HX711.tare_get_data[0]); 
        init_cnt++;
        if(init_cnt == 30)
        {
            HX711.tare = init_data_cal/30 + HX711.tare_get_data[0];
            HX711.isTare = true;
            init_cnt = 0;
        }
    }
    else
    {
        // 读取传感器原始数据
        Read_Raw_Data();
        // 计去皮实际重量
        Cal_Actual_Weight();
        // 卡尔曼滤波
        kalman_update();
    }
}


void Class_TensionMeter::Init(GPIO_TypeDef *_Gpio_Data, uint16_t _Pin_Data, GPIO_TypeDef * _Gpio_Clk, uint16_t _Pin_Clk){

    Gpio_Data = _Gpio_Data;
    Gpio_Clk = _Gpio_Clk;
    Pin_Data = _Pin_Data;
    Pin_Clk = _Pin_Clk;

    HX711.gain = GAIN_128;	/* 当前增益 128 */
    HX711.isTare = false;	/* 默认第一次称重为皮重 */
    HX711.k = 0.004757f;	/* 传感器转换系数 */
    HX711.b = 0.0f;			/* 传感器误差校正/补偿值 */
    HX711.tare = 0.f;		/* 保存皮重数据 */
    HX711.actual = 0.f;		/* 保存净重数据 */

    init_cnt = 0;

    // 初始化卡尔曼滤波器
    kalman_init(0.0f); // 初始估计值为0.0
}

// 初始化卡尔曼滤波器
void Class_TensionMeter::kalman_init(float initial_value) {
    Kf.x = initial_value;  // 初始估计值
    Kf.P = 1000.0;  // 初始误差协方差，较大值表示初始的不确定性
}

// 卡尔曼滤波更新函数
void Class_TensionMeter::kalman_update() {
    // 预测步骤
    float x_pred = A * Kf.x;  // 预测的值
    float P_pred = A * Kf.P * A + Q;  // 预测的协方差

    // 更新步骤
    Kf.K = P_pred * H / (H * P_pred * H + R);  // 计算卡尔曼增益
    Kf.x = x_pred + Kf.K * (Measurement - H * x_pred);  // 更新估计值
    Kf.P = (1 - Kf.K * H) * P_pred;  // 更新协方差

    Tension_Value = Kf.x;  // 保存滤波后的值
}

// HX711 读取 24 位数据函数
void Class_TensionMeter::Read_Raw_Data(void)
{
    uint32_t data = 0;
    // 阻塞等待DT变为低电平（数据准备好）
    while(READ_DT());
    for (int i = 0; i < 24; i++) {
        SCK_HIGH();                // 产生时钟上升沿
        data = (data << 1) | READ_DT(); // 读取DT引脚电平
        SCK_LOW();                 // 时钟下降沿
    }

    // 提供额外的时钟脉冲，选择增益
    SCK_HIGH();
    SCK_LOW();

    Raw_Data = data ^ 0x800000; // 如果最高位为负数标志，转化为有符号值
}


void Class_TensionMeter::Read_Raw_Data_FromISR()
{
    uint32_t data = 0;
    // 等待DT变为低电平（数据准备好）
    if(!READ_DT()) 
    {
        for (int i = 0; i < 24; i++) {
            SCK_HIGH();                // 产生时钟上升沿
            data = (data << 1) | READ_DT(); // 读取DT引脚电平
            SCK_LOW();                 // 时钟下降沿
        }

        // 提供额外的时钟脉冲，选择增益
        SCK_HIGH();
        SCK_LOW();

        Raw_Data = data ^ 0x800000; // 如果最高位为负数标志，转化为有符号值
    }
}

/* 获取传感器实际重量 */
void Class_TensionMeter::Cal_Actual_Weight(void)
{
    float weight = 0.f;
    if(HX711.isTare == true) { /* 当该标志位复位 代表此时应计算净重 */
        weight = (float)Raw_Data * HX711.k + HX711.b;
        HX711.actual = (weight - HX711.tare)/Data_To_Weigth;
        Measurement = HX711.actual;
    }
}

// 读取数据引脚
GPIO_PinState Class_TensionMeter::READ_DT()
{
    return HAL_GPIO_ReadPin(Gpio_Data, Pin_Data);
}

// 拉高数据引脚
void Class_TensionMeter::SCK_HIGH()
{
    HAL_GPIO_WritePin(Gpio_Clk, Pin_Clk, GPIO_PIN_SET);
}
// 拉低数据引脚
void Class_TensionMeter::SCK_LOW() 
{ 
    HAL_GPIO_WritePin(Gpio_Clk, Pin_Clk, GPIO_PIN_RESET); 
}


/* Function prototypes -------------------------------------------------------*/



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
