#include "cmsis_os.h"
#include "PID.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "judge.h"
#include "rc_potocal.h"


/*********************************/
// can_2�������̿���
// ���ID�ֱ�Ϊ1��2��3��4

/*********************************/

/*************ȫ�ֱ���***************/

float Chassis_PID[5] = {30, 1, 10, 16384, 16384};		//����PID�ڴ��޸�
volatile int16_t Vx=0,Vy=0,Wz=0;	//��������ϵ3����
volatile int16_t target_speed_can_2[4];		//

/*************���������㷨�ı�������***************/
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;
//���ڼ��̵���
fp32 ramp[6] = {0,0,0,0,0,0};

/*************����***************/
static void Chassis_PID_Init(float *PID);	//PID������ʼ��
static void Chassis_loop_Init(); //������0
static void Chassis_mode();	//ң����ת����,����ģʽ
static void Chassis_motol_speed_calculate(); //���ֵ��̽���
static void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed); 	//��������
static void Chassis_Power_Limit(double Chassis_pidout_target_limit); 	//�������
static void Chassis_current_give();   //���͵���
static void Chassis_keyboard_mode_1();		//����������ʹģʽ
static void Chassis_keyboard_mode_2();		//�Ի�е��Ϊ��ǰ������ʻģʽ
static void Chassis_keyboard_mode_3();		//�ҿ�ȡ��ģʽ
static void speed_ramp(); //����������


/*************������***************/
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	
	Chassis_PID_Init(Chassis_PID);		//PID������ʼ��
	
  for(;;)
  {
		Chassis_loop_Init();	//����������
		Chassis_mode();//ң������Ϣת��Ϊ�����ٶ�Vy,Vx,Wz
		Chassis_motol_speed_calculate();	//�����˶��߼�����
		Motor_Speed_limiting(target_speed_can_2,5000);//��������ٶȣ���������������ٶ�ֵ(ͬ������)
		Chassis_Power_Limit(20000);									//�������Ҫ��Motor_Speed_limiting�������������ٶȵ�4��
		Chassis_current_give();                 //���͵���  
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}


static void Chassis_PID_Init(float *PID)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_can_2[i], PID[0], PID[1], PID[2], PID[3], PID[5]);
	}
}

static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
}

static void Chassis_mode()
{

			//braking to stop quickly
				if( (rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50)
				&& ( !w_flag && !s_flag && !a_flag && !d_flag && !q_flag && !e_flag))
				{
					for(int i=0;i<4;i++)
					{
						if(motor_info_can_2[i].rotor_speed>360||motor_info_can_2[i].rotor_speed<-360)
						{
							target_speed_can_2[i]=0;
							motor_info_can_2[i].set_voltage = pid_calc(&motor_pid_can_2[i], target_speed_can_2[i], motor_info_can_2[i].rotor_speed);
						}
						else
						{
							motor_info_can_2[i].set_voltage=0;
						}
					}
                  	for(int i =0 ;i < 6;i++)
	      {
           ramp[i]=0;
        }
				}
    
	
	
		// moving control by keyboard
		else
		{
		
			if(rc_ctrl.rc.s[0] == 2)		//�Ҳ��������¶ˣ�������ʻģʽ
			{
				Chassis_keyboard_mode_1();			
			}
			else if (rc_ctrl.rc.s[0] == 3)		//�Ҳ������м䣬��е����ǰ��ģʽ
			{
				Chassis_keyboard_mode_2();
			}
			else if (rc_ctrl.rc.s[0] == 1)		//�Ҳ��������϶ˣ��Կ�ȡ��ģʽ��wasd��������ṹ���ƣ�
			{
				Chassis_keyboard_mode_3();
			}
		}
				// moving	control by remote
				//һֱ�����������ƶ�����е����ǰ����

        Vy = -rc_ctrl.rc.ch[3]/660.0*15000-  ramp[2] +  ramp[3];
        Vx = rc_ctrl.rc.ch[2]/660.0*15000 +  ramp[0] -  ramp[1];
        Wz = -rc_ctrl.rc.ch[4]/660.0*15000+  ramp[4] -  ramp[5] ;

}	

static void speed_ramp()
{
fp32 speed_max = 15000;
	uint8_t start = 10;
	//ǰ��
	if(w_flag)
	{
		 ramp[0] = ramp[0] + start;
	}	
	if(s_flag)
	{
		 ramp[1] = ramp[1] + start;
	}

	
	if(a_flag)
	{
		 ramp[2] = ramp[2] + start;
	}
	if(d_flag)
	{
		 ramp[3] = ramp[3] + start;
	}
	if(q_flag)
	{
		 ramp[4] = ramp[4] + start;
	}
	if(e_flag)
	{
		 ramp[5] = ramp[5] + start;
	}
	for(int i =0 ;i < 6;i++)
	{
		if(ramp[i]>speed_max)
		{
			ramp[i] = speed_max;
		}
	}
    if( !w_flag && !s_flag && !a_flag && !d_flag && !q_flag && !e_flag)
  {
        	for(int i =0 ;i < 6;i++)
	      {
           ramp[i]=0;
        }
  }
}



static void Chassis_motol_speed_calculate()
{
	
	 target_speed_can_2[0] =  Vx+Vy+Wz;
    target_speed_can_2[1] =  Vx-Vy+Wz;
    target_speed_can_2[2] =  -Vy-Vx+Wz; 
    target_speed_can_2[3] =  Vy-Vx+Wz;
}

static void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//�����ֵ
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0ת�������ͣ���Ȼ���ܻ�ֱ�ӵ�0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}

static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{	
	//819.2/A�����������Ϊ120W����ô�ܹ�ͨ����������Ϊ5A��ȡһ������ֵ��800.0 * 5 = 4000
	Watch_Power_Max=Klimit;	Watch_Power=Hero_chassis_power;	Watch_Buffer=60;//����ֵ������ֵ����������ֵ����ʼֵ��1��0��0
	//get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//ͨ������ϵͳ�ͱ�����ֵ��ȡ������ֵ��ʵʱ���ʣ�ʵʱ����������

		Chassis_pidout_max=61536;//32768��40��960			15384 * 4��ȡ��4��3508�����������һ������ֵ

		if(Watch_Power>960)	Motor_Speed_limiting(target_speed_can_2,4096);//��������ٶ� ;//5*4*24;������������������ƽ���ı䣬��֪��Ϊɶһ��ʼ�õ�Power>960,���Թ۲������ֵ�������ܲ���ѹե���幦��
	else{
		Chassis_pidout=(
						fabs(target_speed_can_2[0]-motor_info_can_2[0].rotor_speed)+
						fabs(target_speed_can_2[1]-motor_info_can_2[1].rotor_speed)+
						fabs(target_speed_can_2[2]-motor_info_can_2[2].rotor_speed)+
						fabs(target_speed_can_2[3]-motor_info_can_2[3].rotor_speed));//fabs�������ֵ�������ȡ��4�����ӵĲ�ֵ���
		
//	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

		/*�����ͺ�ռ�Ȼ������������ٶ�*/
		if(Chassis_pidout)
		{
		Scaling1=(target_speed_can_2[0]-motor_info_can_2[0].rotor_speed)/Chassis_pidout;	
		Scaling2=(target_speed_can_2[1]-motor_info_can_2[1].rotor_speed)/Chassis_pidout;
		Scaling3=(target_speed_can_2[2]-motor_info_can_2[2].rotor_speed)/Chassis_pidout;	
		Scaling4=(target_speed_can_2[3]-motor_info_can_2[3].rotor_speed)/Chassis_pidout;//�������4��scaling���Ϊ1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*���������ռ�Ȼ�������������ٶ�*/
//		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
//		else{Klimit = 0;}
		Klimit = Chassis_pidout/Chassis_pidout_target_limit;
		
		if(Klimit > 1) Klimit = 1 ;
		else if(Klimit < -1) Klimit = -1;//���ƾ���ֵ���ܳ���1��Ҳ����Chassis_pidoutһ��ҪС��ĳ���ٶ�ֵ�����ܳ���

		/*��������ռ�Ȼ�������Լ��*/
		if(Watch_Buffer<50&&Watch_Buffer>=40)	Plimit=0.9;		//��������һ��������Լ��������Ϊ�˱��ؿ��Ե���Plimit������Ӱ����Ӧ�ٶȣ�
		else if(Watch_Buffer<40&&Watch_Buffer>=35)	Plimit=0.75;
		else if(Watch_Buffer<35&&Watch_Buffer>=30)	Plimit=0.5;
		else if(Watch_Buffer<30&&Watch_Buffer>=20)	Plimit=0.25;
		else if(Watch_Buffer<20&&Watch_Buffer>=10)	Plimit=0.125;
		else if(Watch_Buffer<10&&Watch_Buffer>=0)	Plimit=0.05;
		else {Plimit=1;}
		
		motor_info_can_2[0].set_voltage = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//���ֵ
		motor_info_can_2[1].set_voltage = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_can_2[2].set_voltage = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_can_2[3].set_voltage = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*ͬ�����ŵ���*/

	}

}

static void Chassis_current_give() 
{
		set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
}

static void Chassis_keyboard_mode_1()
{
				speed_ramp();
}

static void Chassis_keyboard_mode_2()
{
	fp32 speed_max = 15000;
	uint8_t start = 10;
	//ǰ��
	if(w_flag)
	{
		 ramp[2] = ramp[2] + start;
	}
	if(s_flag)
	{
		 ramp[3] = ramp[3] + start;
	}
	if(a_flag)
	{
		 ramp[0] = ramp[0] + start;
	}
	if(d_flag)
	{
		 ramp[1] = ramp[1] + start;
	}
	if(q_flag)
	{
		 ramp[4] = ramp[4] + start;
	}
	if(e_flag)
	{
		 ramp[5] = ramp[5] + start;
	}
	for(int i =0 ;i < 6;i++)
	{
		if(ramp[i]>speed_max)
		{
			ramp[i] = speed_max;
		}
	}
  if( !w_flag && !s_flag && !a_flag && !d_flag && !q_flag && !e_flag)
  {
        	for(int i =0 ;i < 6;i++)
	      {
           ramp[i]=0;
        }
  }
	
}

static void Chassis_keyboard_mode_3()
{

}