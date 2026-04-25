#include "dvc_CustomController.h"

void Class_Custom_Controller::Custom_Controller_Data_Process(uint8_t *Rx_Data)
{
    memcpy(Data_Buffer, Rx_Data, 15);

    // 寻找帧头
    int i;
    for (i = 0; i < 15; i++)
    {
        if (Data_Buffer[i] == 0xA5)
        {
            break;
        }
    }

    // 验证帧尾
    bool flag = Data_Buffer[(i + 14) % 15] == 0x11;

    if (flag)
    {
        for (int j = 0; j < 6; j++)
        {
            int index = 2 * j + i;
            int16_t temp = (Data_Buffer[index + 2] << 8) | Data_Buffer[index + 1];
            Custom_Controller_Data.Angle[j] = temp / 100.f;
        }

        Custom_Controller_Data.gripper_status = Data_Buffer[(i + 13) % 15] == 1 ? true : false;

        rx_frequence = DWT_GetDeltaT(&frequence_cnt);
    }
}