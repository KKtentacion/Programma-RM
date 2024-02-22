#include "driveas5600.h"
#include "i2c.h"

float data=0,degress =0,angle;

/**************************************************************
 * 函数名：uint8_t readOneByte(uint8_t in_adr)
 * 函数功能：给定地址读一个字节
 * 输入参数：in_adr 待读数据的地址
 * 输出参数：retVal读出的数据
 * 返回值：无
**************************************************************/
uint8_t readOneByte(uint8_t in_adr,I2C_HandleTypeDef IIC)//读一个字节函数，从5600的指定寄存器提取一个8位值
{
	uint8_t a3=0x00;
	HAL_I2C_Mem_Read(&IIC, I2C_ADDRESS, (uint16_t)in_adr, I2C_MEMADD_SIZE_8BIT, &a3, 1, 0xff);
	return a3;
}

/**************************************************************
 * 函数名：uint16_t readTwoBytes(uint8_t h_adr, uint8_t l_adr)
 * 函数功能：给定两个地址 各自读出一个字节
 * 输入参数：in_adr 待读高位数据的地址 in_adr_lo待读低位数据的地址
 * 输出参数：retVal读出的数据
 * 返回值：无
**************************************************************/
uint16_t readTwoBytes(uint8_t h_adr , uint8_t l_adr,I2C_HandleTypeDef IIC)
{
	uint16_t retVal = 0;
  uint8_t low=0,high=0;
	/* Read High Byte */ 
	high = readOneByte(h_adr,IIC);
	high = high & 0x0f;
	/* Read Low Byte */
	low = readOneByte(l_adr,IIC);	
	//将两个八位 合成一个16位
	retVal = high << 8;
	retVal = (retVal | low) ;
	
	return retVal;//输出一个16位
}


/**************************************************************
 * 函数名：uint8_t detectMagnet(void)
 * 函数功能：判断有无磁铁
 * 输入参数：无
 * 输出参数：1有磁铁 0无磁铁
 * 返回值：无
**************************************************************/
uint8_t detectMagnet(I2C_HandleTypeDef IIC)
{
  uint8_t retVal = 0;
  retVal = readOneByte(_stat,IIC);
  
  if(retVal & 0x20)
    retVal = 1; 
	else
		degress=400;
  
  return retVal;
}

/**************************************************************
 * 函数名：float Programe_Run(void)
 * 函数功能：读取传感器角度值
 * 输入参数：无
 * 输出参数：传感器角度值  400无磁铁
 * 返回值：无
**************************************************************/
float Programe_Run(I2C_HandleTypeDef IIC)
{
	if(detectMagnet(IIC))
	{
		data =readTwoBytes(_raw_ang_hi, _raw_ang_lo,IIC);
		/* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */ 
		degress = data*360/4095;
		if(degress < 0)
		{
			degress = degress + 360.0f;
		}
	}
	return degress;
}

