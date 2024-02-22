#include "drv_as5600.h"
#include "i2c.h"

float data=0,degress =0,angle;

/**************************************************************
 * ��������uint8_t readOneByte(uint8_t in_adr)
 * �������ܣ�������ַ��һ���ֽ�
 * ���������in_adr �������ݵĵ�ַ
 * ���������retVal����������
 * ����ֵ����
**************************************************************/
uint8_t readOneByte(uint8_t in_adr)//��һ���ֽں�������5600��ָ���Ĵ�����ȡһ��8λֵ
{
	uint8_t a3=0x00;
	HAL_I2C_Mem_Read(&hi2c2, I2C_ADDRESS, (uint16_t)in_adr, I2C_MEMADD_SIZE_8BIT, &a3, 1, 0xff);
	return a3;
}

/**************************************************************
 * ��������uint16_t readTwoBytes(uint8_t h_adr, uint8_t l_adr)
 * �������ܣ�����������ַ ���Զ���һ���ֽ�
 * ���������in_adr ������λ���ݵĵ�ַ in_adr_lo������λ���ݵĵ�ַ
 * ���������retVal����������
 * ����ֵ����
**************************************************************/
uint16_t readTwoBytes(uint8_t h_adr , uint8_t l_adr)
{
	uint16_t retVal = 0;
  uint8_t low=0,high=0;
	/* Read High Byte */ 
	high = readOneByte(h_adr);
	high = high & 0x0f;
	/* Read Low Byte */
	low = readOneByte(l_adr);	
	//��������λ �ϳ�һ��16λ
	retVal = high << 8;
	retVal = (retVal | low) ;
	
	return retVal;//���һ��16λ
}


/**************************************************************
 * ��������uint8_t detectMagnet(void)
 * �������ܣ��ж����޴���
 * �����������
 * ���������1�д��� 0�޴���
 * ����ֵ����
**************************************************************/
uint8_t detectMagnet(void)
{
  uint8_t retVal = 0;
  retVal = readOneByte(_stat);
  
  if(retVal & 0x20)
    retVal = 1; 
	else
		degress=400;
  
  return retVal;
}

/**************************************************************
 * ��������float Programe_Run(void)
 * �������ܣ���ȡ�������Ƕ�ֵ
 * �����������
 * ����������������Ƕ�ֵ  400�޴���
 * ����ֵ����
**************************************************************/
float Programe_Run(void)
{
	if(detectMagnet())
	{
		data =readTwoBytes(_raw_ang_hi, _raw_ang_lo);
		/* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */ 
		degress = data*360/4095;
		if(degress < 0)
		{
			degress = degress + 360.0f;
		}
	}
	return degress;
}

