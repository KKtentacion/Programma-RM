#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "gantry.h"

/*********************************/
// can_1�������żܿ���
// ���ID�ֱ�Ϊ1��2��3��4
// ̧�����ID��1��2
// ǰ����ID��3��4

/*********************************/


/*************ȫ�ֱ���***************/

float Gantry_PID[5] = {30, 1, 10, 16384, 16384};		//����PID�ڴ��޸�

/*********************************/


/*************����***************/
static void	Gantry_PID_Init(float *PID);		//PID������ʼ��
static void Gantry_Slow();		//�������(ֻ���ٶȻ�)
static void Gantry_Mode();		//������ƣ�ֻ���ٶȻ��������ͬѧ�Լ���дһ�¼�λ������
static void Gantry_PID_Calc(); 		//PID����
static void Gantry_PID_Send(); 		//PID����

/*********************************/


/*************������***************/
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
			if(motor_info[i].rotor_speed>5||motor_info[i].rotor_speed<-5)//����+������
			{
				target_speed[i]=0;
				motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);
			}
			else//��ֹ����ʱ�����
			{
				motor_info[i].set_voltage=0;
			}
		}
}

static void Gantry_Mode()
{
		if(v_flag)//0����̧��,���IDΪ1��2
		{
			target_speed[0]=19*45;		//45r/m
			target_speed[1]=-19*45;
		}
		else if(x_flag)
		{
			target_speed[0]=-19*45;		
			target_speed[1]=19*45;
		}	
				
		if(g_flag)//1����ǰ�󣬵��IDΪ3��4
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