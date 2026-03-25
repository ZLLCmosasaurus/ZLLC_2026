/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"
#include "string.h"
#if defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined(MOSASAURUS_ELITE_BOARD) | defined(MOSASAURUS_STEERING_CONTROL_BOARD)
	#include "tim.h"
#elif blablabla
	
#endif

buzzer_t buzzer;


/* Private function prototypes -----------------------------------------------*/
BUZZER_RETURN_T buzzer_playDjiStartUp(buzzer_t *buzzer);
BUZZER_RETURN_T buzzer_playCalibrating(buzzer_t *buzzer);
BUZZER_RETURN_T buzzer_playCalibrated(buzzer_t *buzzer);
BUZZER_RETURN_T buzzer_playDeviceOffline(buzzer_t *buzzer);

BUZZER_RETURN_T buzzer_handleInit(buzzer_t *buzzer, buzzer_parameter_t param);

/*
 *   WARNING:
 *   Functions declare in this section are defined AT THE END of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static uint8_t platform_setFreq		(void* peripheral_handle, uint32_t channel, uint16_t value);
static uint8_t platform_setSwitch	(void* peripheral_handle, uint32_t channel, BUZZER_BSP_SWITCH_T state);
static uint8_t platform_getMsTick	(uint32_t* tick);


/* User functions -----------------------------------------------------------*/

/**
 * @brief 用户设置蜂鸣器任务的函数。当设置的任务与当前任务冲突时，进行优先级抢占。优先级低的任务不会被成功设置。
 *
 * @param buzzer 蜂鸣器的句柄
 * @param task 选择希望的蜂鸣器任务
 * @return BUZZER_RETURN_T
 */
BUZZER_RETURN_T buzzer_setTask(buzzer_t *buzzer, BUZZER_BEEP_TASK_TO_PRIORITY_LUT_T task)
{
	if (task < buzzer->status.currentTask) // 高优先级任务抢占成功
	{
		buzzer->status.currentTask = task;
		return BUZZER_OK;
	}
	else
		return BUZZER_ERROR;
}

/**
 * @brief 用户调用的蜂鸣器任务调度器，建议以 1ms 的频率运行
 *
 * @param buzzer 蜂鸣器的句柄
 * @return BUZZER_RETURN_T
 */
BUZZER_RETURN_T buzzer_taskScheduler(buzzer_t *buzzer)
{
	uint32_t ret;
	if (buzzer->status.currentTask!=buzzer->status.lastTask && buzzer->status.currentTask!=BUZZER_FREE_PRIORITY) // 有新的蜂鸣器任务，记录启动时间
	{
		buzzer_setState(buzzer, BUZZER_SWITCH_ON);
		buzzer_getTick(buzzer, &buzzer->status.tickStart);
	}
	switch (buzzer->status.currentTask)
	{
		case BUZZER_FORCE_STOP_PRIORITY:
			ret = buzzer_setState(buzzer, BUZZER_SWITCH_OFF); // 强制关闭蜂鸣器
			break;
		case BUZZER_FREE_PRIORITY:
			ret = buzzer_setState(buzzer, BUZZER_SWITCH_OFF); // 没有任务，关闭蜂鸣器
			break;
		case BUZZER_DJI_STARTUP_PRIORITY:
			ret = buzzer_playDjiStartUp(buzzer); // 开机启动音
			break;
		case BUZZER_CALIBRATING_PRIORITY:
			ret = buzzer_playCalibrating(buzzer); // 校准音
			break;
		case BUZZER_CALIBRATED_PRIORITY:
			buzzer_playCalibrated(buzzer); // 校准完成音
			break;
		case BUZZER_DEVICE_OFFLINE_PRIORITY:
			buzzer_playDeviceOffline(buzzer); // 设备离线音
			break;
		// case BUZZER_TEST_1:
		// 	buzzer_playtest1(buzzer); // 调试警报1
		default:
			ret = BUZZER_ERROR;
			break;
	}
	buzzer->status.lastTask = buzzer->status.currentTask;
	return ret;
}

/**
 * @brief 一个示例的蜂鸣器句柄初始化
 *
 * @return BUZZER_RETURN_T
 */
BUZZER_RETURN_T buzzer_init_example(void)
{
	buzzer_parameter_t buzzer_init_param;
	#if defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C)
		buzzer_init_param.ctx.peripheral_handle	= &htim12;
		buzzer_init_param.ctx.channel			= TIM_CHANNEL_2;
		buzzer_init_param.clkFreq	= HAL_RCC_GetPCLK2Freq()*PCLK1_APB1_TIMER_CLK_FREQ_COEFFICIENT;
		if (buzzer_init_param.clkFreq/htim4.Instance->PSC/TONE_A4_FREQUENCY/pow(SEMITONE_COEFFICIENT, LOWEST_TONE_POW) < ARR_16BIT_MAXIMUM_VALUE) // PSC 值无法满足发声需求
			htim4.Instance->PSC = 240;
		buzzer_init_param.psc		= htim4.Instance->PSC;
		buzzer_init_param.arr		= htim4.Instance->ARR;
		buzzer_handleInit(&buzzer, buzzer_init_param);
	#elif defined (MOSASAURUS_ELITE_BOARD)
		buzzer_init_param.ctx.peripheral_handle	= &htim3;
		buzzer_init_param.ctx.channel			= TIM_CHANNEL_4;
		buzzer_init_param.clkFreq	= HAL_RCC_GetPCLK1Freq()*PCLK1_APB1_TIMER_CLK_FREQ_COEFFICIENT;
		if (buzzer_init_param.clkFreq/htim3.Instance->PSC/TONE_A4_FREQUENCY/pow(SEMITONE_COEFFICIENT, LOWEST_TONE_POW) < ARR_16BIT_MAXIMUM_VALUE) // PSC 值无法满足发声需求
			htim3.Instance->PSC = MOSASAURUS_ELITE_BOARD_RECOMMENDED_PSC_VALUE;
		buzzer_init_param.psc		= htim3.Instance->PSC;
		buzzer_init_param.arr		= htim3.Instance->ARR;
		buzzer_handleInit(&buzzer, buzzer_init_param);
	#elif defined(MOSASAURUS_STEERING_CONTROL_BOARD)
		buzzer_init_param.ctx.peripheral_handle	= &htim1;
		buzzer_init_param.ctx.channel			= TIM_CHANNEL_3;
		buzzer_init_param.clkFreq	= HAL_RCC_GetPCLK1Freq()*PCLK1_APB1_TIMER_CLK_FREQ_COEFFICIENT;
		if (buzzer_init_param.clkFreq/htim1.Instance->PSC/TONE_A4_FREQUENCY/pow(SEMITONE_COEFFICIENT, LOWEST_TONE_POW) < ARR_16BIT_MAXIMUM_VALUE) // PSC 值无法满足发声需求
			htim1.Instance->PSC = MOSASAURUS_STEERING_CONTROL_BOARD_RECOMMENDED_PSC_VALUE;
		buzzer_init_param.psc		= htim1.Instance->PSC;
		buzzer_init_param.arr		= htim1.Instance->ARR;
		buzzer_handleInit(&buzzer, buzzer_init_param);
	#endif
}

/* Private functions -----------------------------------------------------------*/

BUZZER_RETURN_T buzzer_playDjiStartUp(buzzer_t *buzzer)
{
	uint32_t current_tick, wait;
	buzzer_getTick(buzzer, &current_tick); // 获取当前 Tick
	wait = current_tick - buzzer->status.tickStart;
	if (!buzzerBsp_checkTickTolerance(BUZZER_DJI_STARTUP_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_C5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_DJI_STARTUP_STEP2_PLAY_TONE_D5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_D5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_DJI_STARTUP_STEP3_PLAY_TONE_G5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_G5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_DJI_STARTUP_STEP4_BUZZER_SWITCH_OFF, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_setState(buzzer, BUZZER_SWITCH_OFF);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_DJI_STARTUP_STEP5_BUZZER_RELEASE, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer->status.currentTask = BUZZER_FREE_PRIORITY; // 任务完成，释放优先级
}

BUZZER_RETURN_T buzzer_playCalibrating(buzzer_t *buzzer)
{
	uint32_t current_tick, wait;
	buzzer_getTick(buzzer, &current_tick); // 获取当前 Tick
	wait = current_tick - buzzer->status.tickStart;
	if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATING_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_C5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATING_STEP2_PLAY_TONE_F5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_F5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATING_STEP3_BUZZER_SWITCH_OFF, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_setState(buzzer, BUZZER_SWITCH_OFF);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATING_STEP4_BUZZER_RELEASE, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer->status.currentTask = BUZZER_FREE_PRIORITY; // 任务完成，释放优先级
}

BUZZER_RETURN_T buzzer_playCalibrated(buzzer_t *buzzer)
{
	uint32_t current_tick, wait;
	buzzer_getTick(buzzer, &current_tick); // 获取当前 Tick
	wait = current_tick - buzzer->status.tickStart;
	if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATED_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_C5);
	else if	(!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATED_STEP2_PLAY_TONE_F5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_F5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATED_STEP3_BUZZER_SWITCH_OFF, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_setState(buzzer, BUZZER_SWITCH_OFF);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_CALIBRATED_STEP4_BUZZER_RELEASE, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer->status.currentTask = BUZZER_FREE_PRIORITY; // 任务完成，释放优先级
}

BUZZER_RETURN_T buzzer_playDeviceOffline(buzzer_t *buzzer)
{
	uint32_t current_tick, wait;
	buzzer_getTick(buzzer, &current_tick); // 获取当前 Tick
	wait = current_tick - buzzer->status.tickStart;
	if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP1_PLAY_TONE_E5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_E5);
	else if	(!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP2_PLAY_TONE_C_SLASH_5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_playTone(buzzer, TONE_C_SLASH_5);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP3_BUZZER_SWITCH_OFF, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer_setState(buzzer, BUZZER_SWITCH_OFF);
	else if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP4_BUZZER_RELEASE, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
		buzzer->status.currentTask = BUZZER_FREE_PRIORITY; // 任务完成，释放优先级

}

// BUZZER_RETURN_T buzzer_playtest1(buzzer_t *buzzer)
// {
// 	uint32_t current_tick, wait;
// 	buzzer_getTick(buzzer, &current_tick); // 获取当前 Tick
// 	wait = current_tick - buzzer->status.tickStart;
// 	if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
// 		buzzer_playTone(buzzer, TONE_C5);
// 	else if	(!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
// 		buzzer_playTone(buzzer, TONE_C5);
// 	else if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
// 		buzzer_setState(buzzer, TONE_C5);
// 	else if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP1_PLAY_TONE_C5, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
// 		buzzer_setState(buzzer, TONE_C5);
// 	else if (!buzzerBsp_checkTickTolerance(BUZZER_DEVICE_OFFLINE_STEP4_BUZZER_RELEASE, wait, BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE))
// 		buzzer->status.currentTask = BUZZER_FREE_PRIORITY; // 任务完成，释放优先级

// }

BUZZER_RETURN_T buzzer_handleInit(buzzer_t *buzzer, buzzer_parameter_t param)
{

	// 写入参数
	memcpy(&buzzer->parameter, &param, sizeof(param));
	buzzer->status.currentTask	= BUZZER_FREE_PRIORITY;
	// 加载外设函数
	buzzer->parameter.ctx.get_tick		= platform_getMsTick;
	buzzer->parameter.ctx.set_freq		= platform_setFreq;
	buzzer->parameter.ctx.set_switch	= platform_setSwitch;
}

/* Bsp interaction functions ------------------------------------------------------*/

BUZZER_RETURN_T buzzer_playTone(buzzer_t *buzzer, BUZZER_BSP_TONE_TO_POW_LUT_H tone)
{
	// 判断音调是否有效
	if (tone<LOWEST_TONE_POW || tone>HIGHEST_TONE_POW)
		return BUZZER_WRONG_PARAM;
	uint32_t ret;
	float freq;
	// 计算频率
	freq = TONE_A4_FREQUENCY*pow(SEMITONE_COEFFICIENT, tone);
	// 设置arr
	buzzer->parameter.arr = buzzer->parameter.clkFreq / buzzer->parameter.psc / freq;
	return ret = buzzerBsp_setTimer(&buzzer->parameter.ctx, buzzer->parameter.arr);
}

BUZZER_RETURN_T buzzer_setState(buzzer_t *buzzer, BUZZER_BSP_SWITCH_T state)
{
	if (state != BUZZER_SWITCH_ON && state != BUZZER_SWITCH_OFF)
		return BUZZER_WRONG_PARAM;
	uint32_t ret = buzzerBsp_setSwitch(&buzzer->parameter.ctx, state);
	return ret;
}

BUZZER_RETURN_T buzzer_getTick(buzzer_t *buzzer, uint32_t* tick)
{
	uint32_t ret;
	return ret = buzzzerBsp_getTick(&buzzer->parameter.ctx, tick);
}

/* Platform functions -------------------------------------------------------*/

static uint8_t platform_setFreq(void* peripheral_handle,uint32_t channel, uint16_t value)
{
	uint32_t ret;
	#if defined(MOSASAURUS_ELITE_BOARD) | defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined(MOSASAURUS_STEERING_CONTROL_BOARD)
		// 为了符合 HAL 库的函数，要给外设句柄和 Channel 号套一层壳再赋值
		TIM_HandleTypeDef* peripheral_handleShell = peripheral_handle;
		if (value<0 || value>65535) return BUZZER_WRONG_PARAM; // 适用于 16 bit ARR 寄存器的保护机制
		peripheral_handleShell->Instance->ARR = value;
		ret = __HAL_TIM_SetCompare(peripheral_handleShell, channel, value/3*2);
	#endif
	return ret;
}

static uint8_t platform_setSwitch(void* peripheral_handle, uint32_t channel, BUZZER_BSP_SWITCH_T state)
{
	uint32_t ret;
	#if defined(MOSASAURUS_ELITE_BOARD) | defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined (MOSASAURUS_STEERING_CONTROL_BOARD)
		// 为了符合 HAL 库的函数，要给外设句柄和 Channel 号套一层壳再赋值
		TIM_HandleTypeDef* peripheral_handleShell = peripheral_handle;
		switch(state)
		{
			case BUZZER_SWITCH_ON:
				ret = HAL_TIM_PWM_Start(peripheral_handleShell, channel);
				break;
			case BUZZER_SWITCH_OFF:
				ret = HAL_TIM_PWM_Stop(peripheral_handleShell, channel);
				break;
			default:
				return BUZZER_WRONG_PARAM;
				break;
		}
	#endif
	return ret;
}

/**
 * @brief 供用户自定义的Tick获取函数，需要根据实际平台进行自定义。
 *
 * @param ms 延时的毫秒数
 *
 */
static uint8_t platform_getMsTick(uint32_t* tick)
{
	#if defined(MOSASAURUS_ELITE_BOARD) | defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined (MOSASAURUS_STEERING_CONTROL_BOARD)
		*tick = HAL_GetTick();
	#elif blablabla

	#endif
}
