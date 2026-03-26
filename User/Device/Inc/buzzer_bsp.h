#ifndef BUZZER_BSP_H
#define BUZZER_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/


/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define TONE_A4_FREQUENCY		440 // 标准音频率
#define SEMITONE_COEFFICIENT	1.0594630943592953 // 半音频率比例
#define LOWEST_TONE_POW			-11
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

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>


typedef enum
{
	BUZZER_BSP_OK		= 0x00,
	BUZZER_BSP_ERROR,

	BUZZER_BSP_WRONG_PARAM,
} BUZZER_BSP_RETURN_T;

typedef enum
{
	BUZZER_SWITCH_ON	= 0,
	BUZZER_SWITCH_OFF	= 1,
} BUZZER_BSP_SWITCH_T;





typedef uint8_t (*buzzer_set_freq_ptr)		(void*, uint32_t, uint16_t);
typedef uint8_t (*buzzer_set_switch_ptr)	(void*, uint32_t, BUZZER_BSP_SWITCH_T);
typedef uint8_t (*buzzer_get_msTick_ptr)	(uint32_t*);

typedef struct
{
	buzzer_set_switch_ptr	set_switch;
	buzzer_set_freq_ptr		set_freq;
	buzzer_get_msTick_ptr	get_tick;
	void *peripheral_handle;
	uint32_t channel;
} buzzer_ctx_t;

BUZZER_BSP_RETURN_T buzzerBsp_setSwitch(buzzer_ctx_t *ctx, BUZZER_BSP_SWITCH_T state);
BUZZER_BSP_RETURN_T buzzerBsp_setTimer(buzzer_ctx_t *ctx, uint16_t arr);
BUZZER_BSP_RETURN_T buzzzerBsp_getTick(buzzer_ctx_t *ctx, uint32_t* tick);
BUZZER_BSP_RETURN_T buzzerBsp_checkTickTolerance(uint32_t target, uint32_t measure, uint32_t tolerance);

#ifdef __cplusplus
}
#endif
#endif
