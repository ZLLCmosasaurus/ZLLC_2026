/**
 * @file alg_filter.cpp
 * @author cjw by yssickjgd
 * @brief 滤波器
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 * 
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化滤波器
 *
 * @param __Value_Constrain_Low 滤波器最小值
 * @param __Value_Constrain_High 滤波器最大值
 * @param __Filter_Fourier_Type 滤波器类型
 * @param __Frequency_Low 滤波器特征低频, 非高通有效
 * @param __Frequency_High 滤波器特征高频, 非低通有效
 * @param __Sampling_Frequency 滤波器采样频率
 */
void Class_Filter_Fourier::Init(float __Value_Constrain_Low, float __Value_Constrain_High, Enum_Filter_Fourier_Type __Filter_Fourier_Type, float __Frequency_Low, float __Frequency_High, float __Sampling_Frequency, int __Filter_Fourier_Order)
{
    Value_Constrain_Low = __Value_Constrain_Low;
    Value_Constrain_High = __Value_Constrain_High;
    Filter_Fourier_Type = __Filter_Fourier_Type;
    Frequency_Low = __Frequency_Low;
    Frequency_High = __Frequency_High;
    Sampling_Frequency = __Sampling_Frequency;
    Filter_Fourier_Order = __Filter_Fourier_Order;
    
    //平均数求法
    float system_function_sum = 0.0f;
    //特征低角速度
    float omega_low;
    //特征高角速度
    float omega_high;

    omega_low = 2.0f * PI * Frequency_Low / Sampling_Frequency;
    omega_high = 2.0f * PI * Frequency_High / Sampling_Frequency;

    //计算滤波器系统

    switch (Filter_Fourier_Type)
    {
    case (Filter_Fourier_Type_LOWPASS):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }
    }
    break;
    case (Filter_Fourier_Type_HIGHPASS):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }
    }
    break;
    case (Filter_Fourier_Type_BANDPASS):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high) - omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }
    }
    break;
    case (Filter_Fourier_Type_BANDSTOP):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) + omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }
    }
    break;
    }

    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        system_function_sum += System_Function[i];
    }

    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        System_Function[i] /= system_function_sum;
    }
}

/**
 * @brief 滤波器调整值
 *
 */
void Class_Filter_Fourier::TIM_Adjust_PeriodElapsedCallback()
{
    Out = 0.0f;
    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        Out += System_Function[i] * Input_Signal[(Signal_Flag + i) % (Filter_Fourier_Order + 1)];
    }
}

/**
 * @brief 初始化Kalman滤波器
 * 
 * @param __Error_Measure 测量误差
 * @param __Value 当前值
 * @param __Error_Estimate 估计误差
 */
void Class_Filter_Kalman::Init(float __Error_Measure, float __Error_Estimate,float __X, float __P)
{
    Error_Measure = __Error_Measure;
    Error_Estimate = __Error_Estimate;
    X   = __X;
    P   = __P; 
}

/**
 * @brief 滤波器调整值
 *
 */
void Class_Filter_Kalman::Recv_Adjust_PeriodElapsedCallback()
{
    X_hat = X;                  //最普通的估计方式 直接令本次模型估计值等于上次  A = 1, U = 0;
    P_hat = P + Error_Estimate;

    //省略了Now 与 X 测量值的转移，因为H = 1

    Kalman_Gain = P_hat / (P_hat + Error_Measure);              //因为 H 和 A 等于1退化

    X = X + Kalman_Gain * (Now - X);
    P = (1.0f - Kalman_Gain) * P_hat;

    Out = X;
}

// 初始化滤波器
void init_filter(SpikeFilter* filter, int window_size) {
    filter->window_size = window_size;
    filter->current_index = 0;
    filter->buffer = (float*)malloc(window_size * sizeof(float));
    memset(filter->buffer, 0, window_size * sizeof(float));
}

// 快速排序比较函数（适配float）
int compare(const void* a, const void* b) {
    float diff = *(const float*)a - *(const float*)b;
    return (diff > 0.0f) ? 1 : ((diff < 0.0f) ? -1 : 0);
}

// 滤波处理核心
float process_sample(SpikeFilter* filter, float input) {
    // 更新环形缓冲区
    filter->buffer[filter->current_index] = input;
    filter->current_index = (filter->current_index + 1) % filter->window_size;

    // 创建临时排序数组
    float sort_array[WINDOW_SIZE];
    memcpy(sort_array, filter->buffer, filter->window_size * sizeof(float));
    
    // 快速排序取中值
    qsort(sort_array, filter->window_size, sizeof(float), compare);
    
    return sort_array[filter->window_size / 2];
}

// 资源释放
void free_filter(SpikeFilter* filter) {
    free(filter->buffer);
}

float addSampleAndFilter(float input, int windowSize) {
    static float buffer[MAX_BUFFER_SIZE];  // 内部缓冲区，保存历史数据
    static int count = 0;                  // 当前样本个数

    // 存储新输入的数据
    if (count < MAX_BUFFER_SIZE) {
        buffer[count++] = input;
    } else {
        // 如果缓冲区已满，则将数据左移，丢弃最旧数据
        for (int i = 1; i < MAX_BUFFER_SIZE; i++) {
            buffer[i-1] = buffer[i];
        }
        buffer[MAX_BUFFER_SIZE-1] = input;
        // 保持 count 不变
    }

    // 确定实际参与平均计算的元素个数
    int effectiveCount = (count < windowSize) ? count : windowSize;
    float sum = 0.0f;

    // 计算最近 effectiveCount 个数的和
    for (int i = count - effectiveCount; i < count; i++) {
        sum += buffer[i];
    }

    // 返回平均值
    return sum / effectiveCount;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
