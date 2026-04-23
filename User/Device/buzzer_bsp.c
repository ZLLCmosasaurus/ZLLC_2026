/* Includes ------------------------------------------------------------------*/
#include "buzzer_bsp.h"


BUZZER_BSP_RETURN_T buzzerBsp_setSwitch(buzzer_ctx_t *ctx, BUZZER_BSP_SWITCH_T state)
{
	uint32_t ret;
	return ret = ctx->set_switch(ctx->peripheral_handle, ctx->channel, state);
}

BUZZER_BSP_RETURN_T buzzerBsp_setTimer(buzzer_ctx_t *ctx, uint16_t arr)
{
	uint32_t ret;
	return ret = ctx->set_freq(ctx->peripheral_handle, ctx->channel, arr);
}

BUZZER_BSP_RETURN_T buzzzerBsp_getTick(buzzer_ctx_t *ctx, uint32_t* tick_ptr)
{
	uint32_t ret;
	return ret = ctx->get_tick(tick_ptr);
}

BUZZER_BSP_RETURN_T buzzerBsp_checkTickTolerance(uint32_t target, uint32_t measure, uint32_t tolerance)
{
	int32_t difference = target-measure;
	if ( (difference<tolerance))
		return BUZZER_BSP_OK;
	return BUZZER_BSP_ERROR;
}