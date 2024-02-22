#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "driveas5600.h"
#include "CRC.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


extern float angle;
float angle1;
uint8_t header[39]={0};
uint8_t Serial_RxPacket[6]={0};
uint8_t Serial_RxFlag;
uint8_t data1[4]={0};


static void Data_Concatenation(void);


void Send_Task(void const * argument)
{						
  /* USER CODE BEGIN Send_Task */
  /* Infinite loop */
  //uint32_t wait_time = xTaskGetTickCount();
  HAL_UART_Receive_IT(&huart3,(uint8_t *)Serial_RxPacket,6);
  for(;;)
  { 

    angle=Programe_Run(hi2c1);
    angle1=Programe_Run(hi2c2);
    Data_Concatenation();
    HAL_UART_Transmit(&huart2, (uint8_t *)(&header), sizeof(header), 50);
    osDelay(500);
  }
  /* USER CODE END Send_Task */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART3)
  {
     static uint8_t RxState=0;
   //接收数据包
 
			if(RxState==0)
      { 
           if(Serial_RxPacket[0]==0x0A)
           {
               RxState=1;
           } 
			}
      else if(RxState==1)
      {  
         data1[0]=Serial_RxPacket[1];
         data1[1]=Serial_RxPacket[2]; 
         data1[2]=Serial_RxPacket[3];
         data1[3]=Serial_RxPacket[4]; 
             RxState=2;
			}
      else if(RxState==2)
      {
          if(Serial_RxPacket[5]==0xFE)
         { RxState=0;}
			}
    }
      HAL_UART_Receive_IT(&huart3,(uint8_t *)Serial_RxPacket,6);
 }
static void Data_Concatenation(void)
{
   static uint8_t seq = 0;
   header[0]=0xA5;
   header[1]=0x1E;
   header[3]=seq++;
   Append_CRC8_Check_Sum((uint8_t *)(&header),5);
   header[5]=0x02;
   header[6]=0x03;
   header[7]=(uint16_t)angle>>8;
   header[8]=(uint16_t)angle;
   header[9]=(uint16_t)angle1>>8;
   header[10]=(uint16_t)angle1;
   header[11]=data1[0];
   header[12]=data1[1];
   header[13]=data1[2];
   header[14]=data1[3];
   Append_CRC16_Check_Sum((uint8_t *)(&header),39);
}