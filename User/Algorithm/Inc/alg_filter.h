/**
 * @file alg_filter.h
 * @author cjw by yssickjgd
 * @brief 滤波器
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 * 
 * @copyright ZLLC 2026
 *
 */

#ifndef ALG_FILTER_H
#define ALG_FILTER_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

//滤波器阶数
#define FILTER_FOURIER_ORDER (50)
//采样频率
#define SAMPLING_FREQUENCY (1000.0f)
#define MAX_BUFFER_SIZE 1000
/* Exported types ------------------------------------------------------------*/

#define WINDOW_SIZE 5  // 建议奇数窗口（3/5/7）

typedef struct {
    float* buffer;       // 单精度数据缓存
    int window_size;     // 滤波窗口尺寸
    int current_index;   // 环形缓冲区指针
} SpikeFilter;

void free_filter(SpikeFilter* filter);
void init_filter(SpikeFilter* filter, int window_size);
int compare(const void* a, const void* b);
float process_sample(SpikeFilter* filter, float input);
float addSampleAndFilter(float input, int windowSize);
/**
 * @brief 滤波器类型
 *
 */
enum Enum_Filter_Fourier_Type
{
    Filter_Fourier_Type_LOWPASS = 0,
    Filter_Fourier_Type_HIGHPASS,
    Filter_Fourier_Type_BANDPASS,
    Filter_Fourier_Type_BANDSTOP,
};

/**
 * @brief Reusable, Fourier滤波器算法
 *
 */
class Class_Filter_Fourier
{
public:
    void Init(float __Value_Constrain_Low = 0.0f, float __Value_Constrain_High = 1.0f, Enum_Filter_Fourier_Type __Filter_Fourier_Type = Filter_Fourier_Type_LOWPASS, float __Frequency_Low = 0.0f, float __Frequency_High = SAMPLING_FREQUENCY / 2.0f, float __Sampling_Frequency = SAMPLING_FREQUENCY, int Filter_Fourier_Order = FILTER_FOURIER_ORDER);

    inline float Get_Out();

    inline void Set_Now(float __Now);

    void TIM_Adjust_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //输入限幅
    float Value_Constrain_Low;
    float Value_Constrain_High;
    int Filter_Fourier_Order;

    //滤波器类型
    Enum_Filter_Fourier_Type Filter_Fourier_Type;
    //滤波器特征低频
    float Frequency_Low;
    //滤波器特征高频
    float Frequency_High;
    //滤波器采样频率
    float Sampling_Frequency;

    //常量

    //内部变量

    //卷积系统函数向量
    float System_Function[FILTER_FOURIER_ORDER + 1];

    //输入信号向量
    float Input_Signal[FILTER_FOURIER_ORDER + 1];

    //新数据指示向量
    uint8_t Signal_Flag = 0;

    //读变量

    //输出值
    float Out = 0;

    //写变量

    //内部函数
};

/**
 * @brief Reusable, Kalman滤波器算法
 *
 */
class Class_Filter_Kalman
{
public:
    void Init(float __Error_Measure = 1.0f, float __Error_Estimate = 1.0f, float __X = 0.0f, float __P = 1.0f);

    inline float Get_Out();

    inline void Set_Now(float __Now);

    void Recv_Adjust_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量
    float P;                    //协方差矩阵
    float P_hat;                //先验协方差

    float H = 1.0f;                    //观测转移矩阵
    float A = 1.0f;                    //状态转移矩阵

    //内部变量

    //测量误差(观测噪声)   可以看作传感器的正态分布噪声
    float Error_Measure = 1.0f;
    //估计误差(过程噪声)   可以看作基于模型下，模型存在的正态分布噪声
    float Error_Estimate = 1.0f;
    //增益
    float Kalman_Gain = 0.0f;
    //读变量

    //输出值
    float Out = 0.0f;

    float X_hat;            //先验估计值
    float X;                //最终估计值

    //写变量

    //当前值
    float Now = 0.0f;

    //内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Class_Filter_Fourier::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
void Class_Filter_Fourier::Set_Now(float __Now)
{
    //输入限幅
    Math_Constrain(&__Now, Value_Constrain_Low, Value_Constrain_High);

    //将当前值放入被卷积的信号中
    Input_Signal[Signal_Flag] = __Now;
    Signal_Flag++;

    //若越界则轮回
    if (Signal_Flag == Filter_Fourier_Order + 1)
    {
        Signal_Flag = 0;
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Class_Filter_Kalman::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定观测值
 *
 * @param __Now 观测值
 */
void Class_Filter_Kalman::Set_Now(float __Now)
{
    Now = __Now;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
