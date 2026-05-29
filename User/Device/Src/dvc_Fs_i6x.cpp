/**
 * @file dvc_Fs_i6x.cpp
 * @author 
 * @brief FS-i6X ?????
 * @version 0.1
 * @date 
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_Fs_i6x.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief FS-i6X ??????
 *
 * @param huart ??? UART ??
 */
void Class_Fs_i6x::Init(UART_HandleTypeDef *huart)
{
    // ????? UART ????
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
}

/**
 * @brief ?????? SW3 ??????????????
 *
 * @param Switch ????????????????
 * @param Status ?????????SWITCH_UP / SWITCH_MIDDLE / SWITCH_DOWN?
 * @param Pre_Status ?????????
 */
void Class_Fs_i6x::Judge_Switch_3(Enum_FS_Switch_Status *Switch, uint16_t Status, uint16_t Pre_Status)
{
    // ??????????????????
    switch (Pre_Status)
    {
    case (SWITCH_UP):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FS_Switch_Status_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FS_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = FS_Switch_Status_TRIG_UP_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_DOWN):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FS_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FS_Switch_Status_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = FS_Switch_Status_TRIG_DOWN_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FS_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FS_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = FS_Switch_Status_MIDDLE;
        }
        break;
        }
    }
    break;
    }
}

/**
 * @brief ???????SW0?SW1?SW3???????????????
 *
 * @param Switch ????????????????
 * @param Status ?????????SWITCH_UP / SWITCH_DOWN?
 * @param Pre_Status ?????????
 */
void Class_Fs_i6x::Judge_Switch(Enum_FS_Switch_Status *Switch, uint16_t Status, uint16_t Pre_Status)
{
    // ??????????????????
    switch (Pre_Status)
    {
    case (SWITCH_UP):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FS_Switch_Status_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FS_Switch_Status_TRIG_UP_DOWN;
        }
        break;
        }
    }
    break;
    case (SWITCH_DOWN):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FS_Switch_Status_TRIG_DOWN_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FS_Switch_Status_DOWN;
        }
        break;
        }
    }
    break;
    }
}

/**
 * @brief ???????????
 *
 * @param Pre_UART_Rx_Data ???????????
 * @param Now_UART_Rx_Data ??????????
 */
void Class_Fs_i6x::Judge_Updata(Struct_FS_UART_Data Pre_UART_Rx_Data, Struct_FS_UART_Data Now_UART_Rx_Data)
{
    // ??????????????
    if ((Pre_UART_Rx_Data.ch0 == Now_UART_Rx_Data.ch0) &&
        (Pre_UART_Rx_Data.ch1 == Now_UART_Rx_Data.ch1) &&
        (Pre_UART_Rx_Data.ch2 == Now_UART_Rx_Data.ch2) &&
        (Pre_UART_Rx_Data.ch3 == Now_UART_Rx_Data.ch3))
    {
        FS_Updata_Status = FS_Status_DisUpdata;
    }
    else
    {
        FS_Updata_Status = FS_Status_Updata;
    }
}

/**
 * @brief ???????????????????????????
 *
 */
uint8_t x1=0,x2;
void Class_Fs_i6x::FS_Data_Process()
{
    // ?????????
    memcpy(&Now_UART_Rx_Data, UART_Manage_Object->Rx_Buffer, sizeof(Struct_FS_UART_Data));

    if(Now_UART_Rx_Data.start != 0x0F || Now_UART_Rx_Data.end != 0xFE){
        x1++;
        return;
    }
    x2++;
    FS_Flag += 1;

    // ??????
    Struct_FS_UART_Data *tmp_buffer = (Struct_FS_UART_Data *)UART_Manage_Object->Rx_Buffer;

    /* ????????? */

    // ????????? [-1.0f, 1.0f] ???
    FS_Data.Right_X = (tmp_buffer->ch0 - Rocker_Offset) / Rocker_Num;
    FS_Data.Right_Y = (tmp_buffer->ch1 - Rocker_Offset) / Rocker_Num;
    FS_Data.Left_X  = (tmp_buffer->ch2 - Rocker_Offset) / Rocker_Num;
    FS_Data.Left_Y  = (tmp_buffer->ch3 - Rocker_Offset) / Rocker_Num;

    // ?????????????
    Judge_Switch(&FS_Data.Switch_0, tmp_buffer->s0, Pre_UART_Rx_Data.s0);
    Judge_Switch(&FS_Data.Switch_1, tmp_buffer->s1, Pre_UART_Rx_Data.s1);
    Judge_Switch_3(&FS_Data.Switch_2, tmp_buffer->s2, Pre_UART_Rx_Data.s2);
    Judge_Switch(&FS_Data.Switch_3, tmp_buffer->s3, Pre_UART_Rx_Data.s3);

    // ???????????? [-1.0f, 1.0f] ???
    FS_Data.Yaw_left  = (tmp_buffer->ch4 - Rocker_Offset) / Rocker_Num;
    FS_Data.Yaw_right = (tmp_buffer->ch5 - Rocker_Offset) / Rocker_Num;

    // ????????
    Judge_Updata(Pre_UART_Rx_Data, Now_UART_Rx_Data);
}

/**
 * @brief UART ????????
 *
 * @param Rx_Data ????????
 */
void Class_Fs_i6x::FS_UART_RxCpltCallback(uint8_t *Rx_Data)
{

    FS_Data_Process();
    // ?????????????
    memcpy(&Pre_UART_Rx_Data, UART_Manage_Object->Rx_Buffer, sizeof(Struct_FS_UART_Data));
}

/**
 * @brief TIM ???????????????????? 50ms ?????
 *
 */
void Class_Fs_i6x::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    // ?????????????????
    if (FS_Flag == Pre_FS_Flag)
    {
        FS_Status = FS_Status_DISABLE;
        Unline_Cnt++;
    }
    else
    {
        FS_Status = FS_Status_ENABLE;
    }

    Pre_FS_Flag = FS_Flag;
}

/************************ COPYRIGHT(C) ZLLC 2026 **************************/