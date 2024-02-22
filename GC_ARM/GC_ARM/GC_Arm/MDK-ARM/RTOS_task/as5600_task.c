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
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ��IDLE�ж�
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //��������
  for(;;)
  { 
		if(recv_end_flag == 1)  //������ɱ�־
		{
			if(rx_len==18)
			{
				remote_data_read(rx_buffer);//	�����ң������18�ֽ�����ϣ�����ң�������ݣ����庯����remote_data_user()
				can_cnt_2++;
			}
			recv_end_flag = 0;//������ս�����־λ
			for(uint8_t i=0;i<rx_len;i++)
				{
					rx_buffer[i]=0;//����ջ���
				}
				//memset(rx_buffer,0,rx_len);
			rx_len = 0;//�������
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//���´�DMA����
			if(can_cnt_2==50)
				{
					HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);//���Ʊ�ʾ���յ���
					can_cnt_2=0;
					remote_flag=1;
				}		
			
		}
		
     osDelay(1);
  }
  /* USER CODE END StartTask03 */
}
/*
ƴ�Ӻ���
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART6)
  {

  }
      HAL_UART_Receive_IT(&huart6,(uint8_t *)Serial_RxPacket,19);
 }