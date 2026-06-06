#ifndef CLASS_WS2812_H
#define CLASS_WS2812_H

#include "stm32f4xx_hal.h"

class Class_WS2812
{
public:
    // 空构造函数，不进行任何初始化
    Class_WS2812() {}

    // 初始化：必须在使用任何颜色控制函数前调用一次
    // htim : TIM1 句柄
    // hdma : TIM1_CH1 对应的 DMA 句柄
    // numLeds : 灯珠数量（>=1）
    void Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma, uint16_t numLeds);

    // 析构函数，释放动态分配的缓冲区
    ~Class_WS2812();

    // 颜色开关接口（所有灯珠同时生效）
    void RedOn();
    void RedOff();
    void GreenOn();
    void GreenOff();
    void BlueOn();
    void BlueOff();

    // DMA 完成回调（由 HAL 中断服务调用，禁止用户直接使用）
    void OnDMAComplete();

private:
    // 构建 PWM 缓冲区并启动 DMA 传输
    void Update();

    // 将当前颜色值填充到 m_pwmBuffer 中（所有灯珠）
    void BuildBuffer();

    // WS2812 时序常数（基于 800kHz PWM）
    static constexpr uint16_t PULSE_ZERO = 67;    // 0.4 µs 高电平
    static constexpr uint16_t PULSE_ONE  = 134;   // 0.8 µs 高电平
    static constexpr uint16_t RESET_CYCLES = 40;  // 50 µs 低电平

    TIM_HandleTypeDef *m_htim = nullptr;
    DMA_HandleTypeDef *m_hdma = nullptr;
    uint32_t m_channel = TIM_CHANNEL_1;

    uint16_t m_numLeds = 0;               // 灯珠总数
    uint16_t m_bufferSize = 0;            // pwmBuffer 长度 = 24*numLeds + RESET
    uint16_t *m_pwmBuffer = nullptr;      // 动态分配的 PWM 波形缓冲区

    // 全局颜色（0 或 255），所有灯珠统一
    uint8_t m_red = 0;
    uint8_t m_green = 0;
    uint8_t m_blue = 0;

    volatile bool m_transferComplete = true; // DMA 空闲标志
};

#endif // CLASS_WS2812_H