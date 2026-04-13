/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
// Define board and chips
#define ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C	// 配置适用于 C 板的蜂鸣器设置
// #define MOSASAURUS_ELITE_BOARD					// 配置适用于菁英板的蜂鸣器设置
//#define MOSASAURUS_STEERING_CONTROL_BOARD		// 配置适用于舵轮控制板的蜂鸣器设置

#define BUZZER_MAX_PRIORITY_NUM 64

/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE 20 // 允许音调使用时间点和进入时间点的最大差值

#if defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined(MOSASAURUS_ELITE_BOARD) | defined(MOSASAURUS_STEERING_CONTROL_BOARD) // 时钟配置
	#define PCLK1_APB1_TIMER_CLK_FREQ_COEFFICIENT 2
#endif

#if defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C)
	#define ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C_RECOMMENDED_PSC_VALUE 84
#elif defined(MOSASAURUS_ELITE_BOARD)
	#define MOSASAURUS_ELITE_BOARD_RECOMMENDED_PSC_VALUE 36
#elif defined(MOSASAURUS_STEERING_CONTROL_BOARD) 
	#define MOSASAURUS_STEERING_CONTROL_BOARD_RECOMMENDED_PSC_VALUE 36
#endif
#define ARR_16BIT_MAXIMUM_VALUE	65535	
/* Includes ------------------------------------------------------------------*/
#include "buzzer_bsp.h"
#include <stdint.h>

// 用于规定各种蜂鸣器发声任务优先级，优先进行高优先级的发声任务
// （数字越小优先级越高）
typedef uint8_t BUZZER_BEEP_TASK_TO_PRIORITY_LUT_T; enum 
{
	BUZZER_FORCE_STOP_PRIORITY		= 0,
	BUZZER_DJI_STARTUP_PRIORITY		= 1,
	BUZZER_MARIO_SIMPLE_PRIORITY	= 2,
	BUZZER_DEVICE_OFFLINE_PRIORITY	= 3,
	BUZZER_POKEMON_HEALED			= 4,
	BUZZER_CALIBRATING_PRIORITY		= 5,
	BUZZER_CALIBRATED_PRIORITY		= 6, 
  BUZZER_FREE_PRIORITY				= BUZZER_MAX_PRIORITY_NUM, // 空闲状态
};

// 规定超级马里奥第一个八拍（约 1.5 秒）
typedef uint16_t BUZZER_MARIO_SIMPLE_TICK_TO_TONE_LUT_T; enum
{
    MARIO_SIMPLE_STEP1_E5     = 0,     // 第1个 E5
    MARIO_SIMPLE_STEP2_E5     = 200,   // 第2个 E5
    MARIO_SIMPLE_STEP3_E5     = 400,   // 第3个 E5
    MARIO_SIMPLE_STEP4_C5     = 600,   // C5
    MARIO_SIMPLE_STEP5_E5     = 800,   // E5
    MARIO_SIMPLE_STEP6_G5     = 1000,  // G5
    MARIO_SIMPLE_STEP7_G5     = 1200,  // 最后一个 G5 延长
    MARIO_SIMPLE_STEP8_OFF    = 1500,  // 关闭蜂鸣器
    MARIO_SIMPLE_STEP9_RELEASE= 1700,  // 释放任务
};

typedef uint16_t BUZZER_POKEMON_HEALED_TICK_TO_TONE_LUT_T; enum
{
PKM_RUBY_PRE_A5   = 0,      // 新增：短促低音前奏 (Offset 12)
    PKM_RUBY_STEP1_B5 = 150,    // 第1个 B5
    PKM_RUBY_GAP1     = 600,    // 静音间隔1
    PKM_RUBY_STEP2_B5 = 650,    // 第2个 B5
    PKM_RUBY_GAP2     = 1100,   // 静音间隔2
    PKM_RUBY_STEP3_B5 = 1150,   // 第3个 B5 (进入快节奏部分)
    PKM_RUBY_STEP4_A5 = 1400,   // A5
    PKM_RUBY_STEP5_E6 = 1650,   // E6 (最高音)
    
    PKM_RUBY_OFF      = 2150,   
    PKM_RUBY_RELEASE  = 2300,
};

// 规定 DJI 风格的开机启动声各音调的时间点
typedef uint16_t BUZZER_DJI_STARTUP_TICK_TO_TONE_LUT_T; enum 
{
  BUZZER_DJI_STARTUP_STEP1_PLAY_TONE_C5 = 0,
  BUZZER_DJI_STARTUP_STEP2_PLAY_TONE_D5 = 300,
  BUZZER_DJI_STARTUP_STEP3_PLAY_TONE_G5 = 500,
  BUZZER_DJI_STARTUP_STEP4_BUZZER_SWITCH_OFF = 900,
  BUZZER_DJI_STARTUP_STEP5_BUZZER_RELEASE = 1300,
};

// 规定校准中各音调的时间点
typedef uint16_t BUZZER_CALIBRATING_TICK_TO_TONE_LUT_T; enum
{
  BUZZER_CALIBRATING_STEP1_PLAY_TONE_C5 = 0,
  BUZZER_CALIBRATING_STEP2_PLAY_TONE_F5 = 300,
  BUZZER_CALIBRATING_STEP3_BUZZER_SWITCH_OFF = 700,
  BUZZER_CALIBRATING_STEP4_BUZZER_RELEASE = 1000,
};

// 规定校准完成各音调的时间点
typedef uint16_t BUZZER_CALIBRATED_TICK_TO_TONE_LUT_T; enum
{
	BUZZER_CALIBRATED_STEP1_PLAY_TONE_C5 = 0,
	BUZZER_CALIBRATED_STEP2_PLAY_TONE_F5 = 400,
	BUZZER_CALIBRATED_STEP3_BUZZER_SWITCH_OFF = 900,
	BUZZER_CALIBRATED_STEP4_BUZZER_RELEASE = 1300,
};

// 规定设备离线各音调的时间点
typedef uint16_t BUZZER_DEVICE_OFFLINE_TICK_TO_TONE_LUT_T; enum
{
	BUZZER_DEVICE_OFFLINE_STEP1_PLAY_TONE_E5 = 0,
	BUZZER_DEVICE_OFFLINE_STEP2_PLAY_TONE_C_SLASH_5 = 300,
	BUZZER_DEVICE_OFFLINE_STEP3_BUZZER_SWITCH_OFF = 600,
	BUZZER_DEVICE_OFFLINE_STEP4_BUZZER_RELEASE = 1000,
};
typedef struct {
	uint32_t clkFreq;
	uint16_t psc;
	uint16_t arr;
	buzzer_ctx_t ctx;
} buzzer_parameter_t;

typedef struct {
	BUZZER_BSP_SWITCH_T state;
	BUZZER_BEEP_TASK_TO_PRIORITY_LUT_T currentTask;
	BUZZER_BEEP_TASK_TO_PRIORITY_LUT_T lastTask;
	uint32_t tickStart;
} buzzer_status_t;

typedef struct {
	buzzer_parameter_t parameter;
	buzzer_status_t status;
} buzzer_t;

typedef enum {
	BUZZER_OK,
	BUZZER_ERROR,

	BUZZER_WRONG_PARAM,

	BUZZER_LIST_FULL,
	BUZZER_ILLEGAL_HANDLE
} BUZZER_RETURN_T;

BUZZER_RETURN_T buzzer_playTone(buzzer_t *buzzer, BUZZER_BSP_TONE_TO_POW_LUT_H tone);
BUZZER_RETURN_T buzzer_setState(buzzer_t *buzzer, BUZZER_BSP_SWITCH_T state);
BUZZER_RETURN_T buzzer_getTick(buzzer_t *buzzer, uint32_t *tick);

BUZZER_RETURN_T buzzer_init_example(void);
BUZZER_RETURN_T buzzer_setTask(buzzer_t *buzzer, BUZZER_BEEP_TASK_TO_PRIORITY_LUT_T task);
BUZZER_RETURN_T buzzer_taskScheduler(buzzer_t *buzzer);

extern buzzer_t buzzer;

#ifdef __cplusplus
}
#endif
#endif
