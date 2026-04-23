/**
 * @file dvc_boardc_ist8310.cpp
 * @author lez by yssickjgd
 * @brief 磁力计
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#include "dvc_boardc_ist8310.h"

/**
  * @brief          initialize ist8310 gpio.
  * @param[in]      none
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_GPIO_init(void)
{
}

/**
  * @brief          initialize ist8310 communication interface
  * @param[in]      none
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_com_init(void)
{
}

/**
  * @brief          read a byte of ist8310 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
/**
  * @brief          ��ȡIST8310��һ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ַ
  * @retval         �Ĵ���ֵ
  */
uint8_t Class_BoardC_IST8310::ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    IIC_Send_Receive_Data(IIC_Manage_Object->IIC_Handler, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10, IIC_READ);
    //HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1 , 10);
    return res;
}


/**
  * @brief          write a byte of ist8310 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          ͨ��I2Cд��һ���ֽڵ�IST8310�ļĴ�����
  * @param[in]      �Ĵ�����ַ
  * @param[in]      д��ֵ
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
  IIC_Send_Receive_Data(IIC_Manage_Object->IIC_Handler, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10, IIC_WRITE);
  //HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&data,1,10);
}

/**
  * @brief          read multiple byte of ist8310 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
/**
  * @brief          ��ȡIST8310�Ķ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  IIC_Send_Receive_Data(IIC_Manage_Object->IIC_Handler, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10, IIC_READ);
  //HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,buf,len,10);
}


/**
  * @brief          write multiple byte of ist8310 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
/**
  * @brief          д�����ֽڵ�IST8310�ļĴ���ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  IIC_Send_Receive_Data(IIC_Manage_Object->IIC_Handler, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10, IIC_WRITE);
  //HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,data,len,10);
}

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          ��ʱx����
  * @param[in]      ms: ms����
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}


/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
/**
  * @brief          ��ʱx΢��
  * @param[in]      us: us΢��
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 72;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}


/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����RSTN����Ϊ1
  * @param[in]      none
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin,GPIO_PIN_SET);
}


/**
  * @brief          set the RSTN PIN to 0
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����RSTN����Ϊ0
  * @param[in]      none
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}
				
				
/**
  * @brief          initialize ist8310
  * @param[in]      none
  * @retval         error value
  */
/**
  * @brief          ��ʼ��IST8310
  * @param[in]      none
  * @retval         error value
  */
uint8_t Class_BoardC_IST8310::ist8310_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_GPIO_init();
    ist8310_com_init();

    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }

    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

/**
  * @brief          if you have read the data from STAT1 to DATAZL usaully by I2C DMA , you can use the function to solve. 
  * @param[in]      status_buf:the data point from the STAT1(0x02) register of IST8310 to the DATAZL(0x08) register 
  * @param[out]     ist8310_real_data:ist8310 data struct 
  * @retval         none
  */
/**
  * @brief          ����Ѿ�ͨ��I2C��DMA��ʽ��ȡ���˴�STAT1��DATAZL�����ݣ�����ʹ������������д���
  * @param[in]      status_buf:����ָ��,��STAT1(0x02) �Ĵ����� DATAZL(0x08)�Ĵ��� 
  * @param[out]     ist8310_real_data:ist8310�����ݽṹ
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/**
  * @brief          read mag magnetic field strength data of IST8310 by I2C
  * @param[out]     mag variable
  * @retval         none
  */
/**
  * @brief          ͨ����ȡ�ų�����
  * @param[out]     �ų�����
  * @retval         none
  */
void Class_BoardC_IST8310::ist8310_read_mag(float mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    //read the "DATAXL" register (0x03)
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}


uint8_t Class_BoardC_IST8310::init(I2C_HandleTypeDef* hi2c)
{
    if(hi2c == NULL)
    {
      return 0;
    }
    if(hi2c->Instance == I2C1)
    {
      IIC_Manage_Object = &IIC1_Manage_Object;
    }
    else if(hi2c->Instance == I2C2)
    {
      IIC_Manage_Object = &IIC2_Manage_Object;
    }
    else if(hi2c->Instance == I2C3)
    {
      IIC_Manage_Object = &IIC3_Manage_Object;
    }
    else
    {
      return 0;
    }
    
    while(ist8310_init());
    return 1;
}