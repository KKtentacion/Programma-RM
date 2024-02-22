#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "gantry.h"

/*********************************/
// can_1用作龙门架控制
// 电机ID分别为1，2，3，4
// 抬升电机ID是1，2
// 前后电机ID是3，4

/*********************************/


/*************全局变量***************/

float Gantry_PID[5] = {30, 1, 10, 16384, 16384};		//底盘PID在此修改

/*********************************/


/*************函数***************/
static void	Gantry_PID_Init(float *PID);		//PID参数初始化
static void Gantry_Slow();		//电机减速(只有速度环)
static void Gantry_Mode();		//电机控制（只有速度环），请何同学自己改写一下键位并调试
static void Gantry_PID_Calc(); 		//PID计算
static void Gantry_PID_Send(); 		//PID发送

/*********************************/


/*************主任务***************/
void StartGantryTask(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	
	Gantry_PID_Init(Gantry_PID);

  for(;;)
  {
			if(!x_flag  && !v_flag && !g_flag && !b_flag)
			{
				Gantry_Slow();
			}
			else
			{
				Gantry_Mode();			
			}
			Gantry_PID_Calc();
			Gantry_PID_Send();
			osDelay(1);
  
  /* USER CODE END StartTask04 */
	}
}

static void Gantry_PID_Init(float *PID)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid[i], PID[0], PID[1], PID[2], PID[3], PID[4]);
	}
}

static void Gantry_Slow()
{
		for(int i=0;i<4;i++)
		{
			if(motor_info[i].rotor_speed>5||motor_info[i].rotor_speed<-5)//减速+给力矩
			{
				target_speed[i]=0;
				motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);
			}
			else//防止减速时抖电机
			{
				motor_info[i].set_voltage=0;
			}
		}
}

static void Gantry_Mode()
{
		if(v_flag)//0控制抬升,电机ID为1和2
		{
			target_speed[0]=19*45;		//45r/m
			target_speed[1]=-19*45;
		}
		else if(x_flag)
		{
			target_speed[0]=-19*45;		
			target_speed[1]=19*45;
		}	
				
		if(g_flag)//1控制前后，电机ID为3和4
		{
			target_speed[2]=19*45;		//45r/m
			target_speed[3]=-19*45;
		}
		else if(b_flag)
		{
			target_speed[2]=-19*45;		
			target_speed[3]=19*45;
		}	
}

static void Gantry_PID_Calc()
{
	for (uint8_t i = 0; i < 4; i++)
	{			
		motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);
	}
}

static void Gantry_PID_Send()
{
		set_motor_voltage(0, 
                      motor_info[0].set_voltage, 
                      motor_info[1].set_voltage, 
                      motor_info[2].set_voltage, 
                      motor_info[3].set_voltage);
}