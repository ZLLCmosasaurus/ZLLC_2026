/**
 * @file dvc_boardc_ist8310.h
 * @author lez by yssickjgd
 * @brief 磁力计
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */
#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "drv_i2c.h"

//#define IST8310_DATA_READY_BIT 2

//#define IST8310_NO_ERROR 0x00
//#define IST8310_NO_SENSOR 0x40

//#define MAG_SEN 0.3f //raw int16 data change to uT unit. ԭʼ�������ݱ�� ��λut

//#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I " 
//#define IST8310_WHO_AM_I_VALUE 0x10 //device ID

//#define IST8310_WRITE_REG_NUM 4 

//#define IST8310_IIC_ADDRESS 0x0E  //the I2C address of IST8310

//the first column:the registers of IST8310. ��һ��:IST8310�ļĴ���
//the second column: the value to be writed to the registers.�ڶ���:��Ҫд��ļĴ���ֵ
//the third column: return error value.������:���صĴ�����


typedef struct ist8310_real_data_t
{
  uint8_t status;
  float mag[3];
} IST8310_Real_Data_t;

class Class_BoardC_IST8310
{
  public:
    uint8_t init(I2C_HandleTypeDef *hi2c);

    uint8_t ist8310_init(void);    
    void ist8310_GPIO_init(void);
    void ist8310_com_init(void);

    void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);
    void ist8310_read_mag(float mag[3]);


    uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
    void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
    void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
    void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

    void ist8310_delay_ms(uint16_t ms);
    void ist8310_delay_us(uint16_t us);

    void ist8310_RST_L(void);
    void ist8310_RST_H(void);

  protected:

    Struct_IIC_Manage_Object *IIC_Manage_Object;
    IST8310_Real_Data_t ist8310_real_data;
	
		const uint8_t IST8310_DATA_READY_BIT = 2;

		const uint8_t IST8310_NO_ERROR = 0x00;
		const uint8_t IST8310_NO_SENSOR = 0x40;

		const float MAG_SEN = 0.3f; //raw int16 data change to uT unit. ԭʼ�������ݱ�� ��λut

		const uint8_t IST8310_WHO_AM_I = 0x00;      //ist8310 "who am I " 
		const uint8_t IST8310_WHO_AM_I_VALUE = 0x10; //device ID

		const uint8_t IST8310_WRITE_REG_NUM = 4; 

		const uint8_t IST8310_IIC_ADDRESS = 0x0E;  //the I2C address of IST8310	
	
		const uint8_t ist8310_write_reg_data_error[4][3] ={
						{0x0B, 0x08, 0x01},     //enalbe interrupt  and low pin polarity.�����жϣ��������õ͵�ƽ
						{0x41, 0x09, 0x02},     //average 2 times.ƽ����������
						{0x42, 0xC0, 0x03},     //must be 0xC0. ������0xC0
						{0x0A, 0x0B, 0x04}};    //200Hz output rate.200Hz���Ƶ��
};









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
#endif
