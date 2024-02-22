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
#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值

#endif