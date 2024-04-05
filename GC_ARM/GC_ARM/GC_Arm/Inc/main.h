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
//结构体
typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_voltage;		//发送信息
    uint16_t rotor_angle;		//现在的角度
		uint16_t last_rotor_angle; //previous angle
		uint16_t total_angle;
		uint8_t initflag;
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
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

//宏定义
#define MOTOR_MAX_NUM 7		//最大数据字节数
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//越界则赋边界值
#define FEEDBACK_ID_BASE      0x201
#define FEEDBACK_ID_BASE_6020 0x205
#define CAN_CONTROL_ID_BASE   0x200
#define CAN_CONTROL_ID_EXTEND 0x1ff
#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//全局变量
extern volatile uint8_t rx_len;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag; //一帧数据接收完成标志
extern uint8_t rx_buffer[100];  //接收数据缓存数组
extern uint16_t can_cnt_2;
extern float target_speed[7];//实测最大空载转速320rpm
extern moto_info_t motor_info[MOTOR_MAX_NUM];		//赋予最大的7个字节
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
extern int16_t target_int2;//用于叠加旋转和直行
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
