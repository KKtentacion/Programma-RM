#include "drv_as5600.h"
#include "cmsis_os.h"
#include "PID.h"
#include "cmsis_os.h"
#include "judge.h"
#include "rc_potocal.h"
#include "pid.h"
#include "drv_can.h"
#include "CRC.h"
#include "main.h"
#include "usart.h"
extern float angle;
uint8_t Serial_RxPacket[19]={0};

static void Data_Concatenation(void);
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  HAL_UART_Receive_IT(&huart6,(uint8_t *)Serial_RxPacket,19);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收
  for(;;)
  { 
		if(recv_end_flag == 1)  //接收完成标志
		{
			if(rx_len==18)
			{
				remote_data_read(rx_buffer);//	如果和遥控器的18字节相符合，解算遥控器数据，具体函数在remote_data_user()
				can_cnt_2++;
			}
			recv_end_flag = 0;//清除接收结束标志位
			for(uint8_t i=0;i<rx_len;i++)
				{
					rx_buffer[i]=0;//清接收缓存
				}
				//memset(rx_buffer,0,rx_len);
			rx_len = 0;//清除计数
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//重新打开DMA接收
			if(can_cnt_2==50)
				{
					HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);//蓝灯表示接收到了
					can_cnt_2=0;
					remote_flag=1;
				}		
			
		}
		
     osDelay(1);
  }
  /* USER CODE END StartTask03 */
}
/*
拼接函数
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART6)
  {

  }
      HAL_UART_Receive_IT(&huart6,(uint8_t *)Serial_RxPacket,19);
 }