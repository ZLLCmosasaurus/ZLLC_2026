#include "stm32f4xx_hal.h"
#include "math.h"
#include "cstring"



// 定义蜂鸣器的最大优先级数，用于控制不同音调的播放优先级
#define BUZZER_MAX_PRIORITY_NUM 16

// 定义蜂鸣器的最大音调数，表示可以播放的不同音调的数量
#define BUZZER_MAX_TONE_NUM 32

// 定义蜂鸣器任务的时间差容忍度，用于调整任务调度的时间精度
#define BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE 20

// 定义标准音A4的频率，用于音调的基准
#define TONE_A4_FREQUENCY		440 // 标准音频率

// 定义半音的频率比例，用于计算不同音调的频率
#define SEMITONE_COEFFICIENT	1.0594630943592953 // 半音频率比例

// 定义最低音调的幂指数，表示最低音调的相对位置
#define LOWEST_TONE_POW			-11

// 定义最高音调的幂指数，表示最高音调的相对位置
#define HIGHEST_TONE_POW		12

// 用于查表确定次方数，从而计算蜂鸣器频率
typedef enum
{
	TONE_A3			= -12,
	TONE_A_SLASH_3	= -11,
	TONE_B3			= -10,
	TONE_C4			= -9,
	TONE_C_SLASH_4	= -8,
	TONE_D4			= -7,
	TONE_D_SLASH_4	= -6,
	TONE_E4			= -5,
	TONE_F4			= -4,
	TONE_F_SLASH_4	= -3,
	TONE_G4			= -2,
	TONE_G_SLASH_4	= -1,
	TONE_A4			= 0,
	TONE_A_SLASH_4	= 1,
	TONE_B4			= 2,
	TONE_C5			= 3,
	TONE_C_SLASH_5	= 4,
	TONE_D5			= 5,
	TONE_D_SLASH_5	= 6,
	TONE_E5			= 7,
	TONE_F5			= 8,
	TONE_F_SLASH_5	= 9,
	TONE_G5			= 10,
	TONE_G_SLASH_5	= 11,
	TONE_A5			= 12,
} BUZZER_BSP_TONE_TO_POW_LUT_H;


typedef uint8_t BUZZER_BEEP_TASK_PRIORITY_E;
// 枚举类型，定义了不同蜂鸣器任务优先级的常量
enum {
 
  BUZZER_FORCE_STOP_PRIORITY = 0,
 
  BUZZER_DJI_STARTUP_PRIORITY = 5,
  
  BUZZER_CALIBRATING_PRIORITY = 2,
  
  BUZZER_CALIBRATED_PRIORITY = 3,
 
  BUZZER_DEVICE_OFFLINE_PRIORITY = 1,
  
  BUZZER_FREE_PRIORITY = BUZZER_MAX_PRIORITY_NUM-1, // 空闲状态
};


// 定义一个枚举类型 BUZZER_SWITCH_STATE_E，用于表示蜂鸣器的开关状态
typedef enum
{
	BUZZER_SWITCH_ON	= 0,
	BUZZER_SWITCH_OFF	= 1,
} BUZZER_SWITCH_STATE_E;


class Class_Buzzer_Tone
{
  public:
    // 定义一个枚举类型，用于表示不同的音调
    BUZZER_BSP_TONE_TO_POW_LUT_H Tone;
    // 定义一个浮点型变量，用于表示音调的持续时间
    float Duration;

    // 设置音调的函数
    void SetTone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone);
    // 设置音调持续时间的函数
    void SetDuration(float __Duration);
    // 获取当前音调的函数
    BUZZER_BSP_TONE_TO_POW_LUT_H GetTone(void);
    // 获取当前音调持续时间的函数
    float GetDuration(void);
};


// 定义一个名为 Class_Sheet_Music 的类，用于表示乐谱
class Class_Sheet_Music
{
  public:
    // ToneList 是一个数组，用于存储乐谱中的音符，数组大小为 BUZZER_MAX_TONE_NUM
  Class_Buzzer_Tone ToneList[BUZZER_MAX_TONE_NUM];
    // Tone 是一个指针，指向一个查找表，用于将音符转换为功率级别
  BUZZER_BSP_TONE_TO_POW_LUT_H *Tone; 
    // Duration 是一个指针，指向一个数组，用于存储每个音符的持续时间
  float *Duration;
    // Num 是一个无符号8位整数，表示乐谱中的音符数量
  uint8_t Num;
    // SetToneList 函数用于设置乐谱中的音符列表、每个音符的持续时间和音符数量
  void SetToneList(int *__Tone, int *__Duration, uint8_t __Num);
    // PlayToneList 函数用于播放乐谱中的音符列表
  void PlayToneList(void);
    // SetNum 函数用于设置乐谱中的音符数量
  void SetNum(uint8_t __Num);
    // GetNum 函数用于获取乐谱中的音符数量
  uint8_t GetNum(void);
    
};


// 定义一个名为Class_Buzzer的类，用于控制蜂鸣器
class Class_Buzzer
{
  public:
 
    // 初始化蜂鸣器，设置PWM定时器和通道，以及默认的驱动频率
    void Buzzer_Init(TIM_HandleTypeDef *__Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel,uint32_t __Driver_Frequency=84000000);
    // 初始化蜂鸣器的乐谱列表
    void Buzzer_SheetListInit(void);
    // 打开蜂鸣器
    void Buzzer_On(void);
    // 关闭蜂鸣器
    void Buzzer_Off(void);
    // 播放指定的音调
    void Play_Tone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone );
    // 设置定时器的自动重装载寄存器值
    void Set_Arr(uint16_t __Arr);
    // 设置当前时间
    void Set_NowTime(float __NowTime);
    // 设置开始时间
    void Set_StartTime(float __StartTime);
    // 设置等待时间
    void Set_WaitTime(void);
    // 获取等待时间
    float Get_WaitTime(void);
    // 设置蜂鸣器的状态
    void Set_State(BUZZER_SWITCH_STATE_E __State);
    // 设置目标频率
    void Set_TargetFrequency(float __TargetFrequency);
    // 设置当前任务
    void Set_NowTask(BUZZER_BEEP_TASK_PRIORITY_E __NowTask);
    // 设置上一个任务
    void Set_LastTask(BUZZER_BEEP_TASK_PRIORITY_E __LastTask);
    // 获取当前任务
    BUZZER_BEEP_TASK_PRIORITY_E Get_NowTask(void);
    // 获取上一个任务
    BUZZER_BEEP_TASK_PRIORITY_E Get_LastTask(void);
    // 计算周期中断回调
    void Buzzer_Calculate_PeriodElapsedCallback(void);
    // 重新加载蜂鸣器状态周期中断回调
    void Reload_Buzzer_Status_PeriodElapsedCallback(void);
    // 播放音调列表
    void PlayToneList(BUZZER_BEEP_TASK_PRIORITY_E __Task);

    // 指向PWM定时器的句柄
    TIM_HandleTypeDef *Driver_PWM_TIM;
    // PWM定时器的通道
    uint8_t Driver_PWM_TIM_Channel;

    // 时钟频率
    uint32_t ClkFreq;
    // 预分频器值
    uint16_t Psc;
    // 自动重装载寄存器值
    uint16_t Arr;

    // 蜂鸣器的状态
    BUZZER_SWITCH_STATE_E BuzzerState;
    // 蜂鸣器的开始时间
    float BuzzerStartTime;
    // 蜂鸣器的当前时间
    float BuzzerNowTime;
    // 蜂鸣器的等待时间
    float BuzzerWaitTime;

    // 蜂鸣器的乐谱列表
    Class_Sheet_Music SheetMusic[BUZZER_MAX_PRIORITY_NUM];

    // 目标频率
    float TargetFrequency;

    // 当前任务
    BUZZER_BEEP_TASK_PRIORITY_E NowTask;
    // 上一个任务
    BUZZER_BEEP_TASK_PRIORITY_E LastTask;

    
};



