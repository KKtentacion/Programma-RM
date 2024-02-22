/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//�ṹ��
typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_voltage;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
}moto_info_t;

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value  
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

//�궨��
#define MOTOR_MAX_NUM 7		//��������ֽ���
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//Խ���򸳱߽�ֵ
#define FEEDBACK_ID_BASE      0x201
#define FEEDBACK_ID_BASE_6020 0x205
#define CAN_CONTROL_ID_BASE   0x200
#define CAN_CONTROL_ID_EXTEND 0x1ff
#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//ȫ�ֱ���
extern volatile uint8_t rx_len;  //����һ֡���ݵĳ���
extern volatile uint8_t recv_end_flag; //һ֡���ݽ�����ɱ�־
extern uint8_t rx_buffer[100];  //�������ݻ�������
extern uint16_t can_cnt_2;
extern float target_speed[7];//ʵ��������ת��320rpm
extern moto_info_t motor_info[MOTOR_MAX_NUM];		//��������7���ֽ�
extern moto_info_t motor_info_can_2[MOTOR_MAX_NUM];
extern pid_struct_t motor_pid[7];	
extern pid_struct_t motor_pid_can_2[7];	
extern uint8_t can_flag;
extern uint8_t remote_flag;
extern double step; 
extern double r;
extern double sin_sita;
extern double cos_sita;
extern double target_v;
extern int16_t target_int1;
extern int16_t target_int2;//���ڵ�����ת��ֱ��
extern double target_curl;
extern float yuntai_step;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Vcc_as_Pin GPIO_PIN_12
#define Vcc_as_GPIO_Port GPIOB
#define GND_as_Pin GPIO_PIN_13
#define GND_as_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
