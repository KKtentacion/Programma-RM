#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
#include "rc_potocal.h"
#include "pid.h"
//extern RC_ctrl_t rc_ctrl;

void CAN1_Init(void);
void CAN2_Init( void );
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);
void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
extern Motor_t MOTOR1_can1;
extern Motor_t MOTOR2_can1;
extern Motor_t MOTOR3_can1;
extern Motor_t MOTOR1_can2;
extern Motor_t MOTOR2_can2;
extern Motor_t MOTOR3_can2;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define P_MIN -12.5		//λ����Сֵ
#define P_MAX 12.5		//λ�����ֵ
#define V_MIN -45			//�ٶ���Сֵ
#define V_MAX 45			//�ٶ����ֵ
#define KP_MIN 0.0		//Kp��Сֵ
#define KP_MAX 500.0	//Kp���ֵ
#define KD_MIN 0.0		//Kd��Сֵ
#define KD_MAX 5.0		//Kd���ֵ
#define T_MIN -18			//ת�����ֵ
#define T_MAX 18			//ת����Сֵ

#endif