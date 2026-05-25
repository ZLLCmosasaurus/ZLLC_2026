#include "dvc_ws2812.h"
#include <cstring>  // for memset

// 静态全局指针，用于中断回调访问当前实例
static Class_WS2812 *g_ws2812Instance = nullptr;

// DMA 完成中断回调（由 stm32f4xx_it.c 中的 DMA2_Stream5_IRQHandler 触发）
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (g_ws2812Instance != nullptr)
    {
        g_ws2812Instance->OnDMAComplete();
    }
}

void Class_WS2812::Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma, uint16_t numLeds)
{
    // 1. 保存句柄和参数
    m_htim = htim;
    m_hdma = hdma;
    m_channel = TIM_CHANNEL_4;
    m_numLeds = numLeds;
    m_bufferSize = 24 * m_numLeds + RESET_CYCLES;

    // 2. 释放旧缓冲区（如果重复调用 Init）
    if (m_pwmBuffer != nullptr)
    {
        delete[] m_pwmBuffer;
        m_pwmBuffer = nullptr;
    }

    // 3. 动态分配波形缓冲区
    m_pwmBuffer = new uint16_t[m_bufferSize];
    // 初始化为全零（复位电平）
    memset(m_pwmBuffer, 0, m_bufferSize * sizeof(uint16_t));

    // 4. 初始化颜色为全灭
    m_red   = 0;
    m_green = 0;
    m_blue  = 0;
    m_transferComplete = true;

    // 5. 注册当前实例用于回调
    g_ws2812Instance = this;

    // 6. 开启 PWM 通道（但不立即输出，后续由 DMA 更新）
    HAL_TIM_PWM_Start(m_htim, m_channel);
}

Class_WS2812::~Class_WS2812()
{
    if (m_pwmBuffer != nullptr)
    {
        delete[] m_pwmBuffer;
        m_pwmBuffer = nullptr;
    }
    // 若本对象已注册为回调实例，注销之
    if (g_ws2812Instance == this)
    {
        g_ws2812Instance = nullptr;
    }
}

// ---------- 颜色控制 ----------
void Class_WS2812::RedOn()
{
    m_red = 255;
    Update();
}

void Class_WS2812::RedOff()
{
    m_red = 0;
    Update();
}

void Class_WS2812::GreenOn()
{
    m_green = 255;
    Update();
}

void Class_WS2812::GreenOff()
{
    m_green = 0;
    Update();
}

void Class_WS2812::BlueOn()
{
    m_blue = 255;
    Update();
}

void Class_WS2812::BlueOff()
{
    m_blue = 0;
    Update();
}

// ---------- 内部实现 ----------
void Class_WS2812::Update()
{
    // 等待上一次传输完成
    while (!m_transferComplete) { }

    // 标记为传输中
    m_transferComplete = false;

    // 取消可能未完成的 DMA 传输
    HAL_DMA_Abort(m_hdma);

    // 生成新波形数据
    BuildBuffer();

    // 确保 PWM 输出开启
    HAL_TIM_PWM_Start(m_htim, m_channel);

    // 启动 DMA：将整个波形缓冲区送入 TIM1->CCR1
    HAL_DMA_Start_IT(m_hdma,
                     reinterpret_cast<uint32_t>(m_pwmBuffer),
                     reinterpret_cast<uint32_t>(&(m_htim->Instance->CCR4)),  // 改为 CCR4
                     m_bufferSize);
}

void Class_WS2812::BuildBuffer()
{
    // 为每个灯珠填充 24 位颜色数据（顺序：GRB）
    uint8_t g = m_green;
    uint8_t r = m_red;
    uint8_t b = m_blue;

    uint16_t *buf = m_pwmBuffer; // 缓冲区起始指针

    for (uint16_t led = 0; led < m_numLeds; ++led)
    {
        // 绿色 8 位
        uint8_t temp_g = g;
        for (int i = 0; i < 8; ++i)
        {
            *buf++ = (temp_g & 0x80) ? PULSE_ONE : PULSE_ZERO;
            temp_g <<= 1;
        }
        // 红色 8 位
        uint8_t temp_r = r;
        for (int i = 0; i < 8; ++i)
        {
            *buf++ = (temp_r & 0x80) ? PULSE_ONE : PULSE_ZERO;
            temp_r <<= 1;
        }
        // 蓝色 8 位
        uint8_t temp_b = b;
        for (int i = 0; i < 8; ++i)
        {
            *buf++ = (temp_b & 0x80) ? PULSE_ONE : PULSE_ZERO;
            temp_b <<= 1;
        }
    }

    // 填充复位信号（全 0）
    for (int i = 0; i < RESET_CYCLES; ++i)
    {
        *buf++ = 0;
    }
}

void Class_WS2812::OnDMAComplete()
{
    // DMA 传输完成，灯带数据发送完毕
    m_transferComplete = true;
}