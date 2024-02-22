#include "cmsis_os.h"
#include "PID.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "judge.h"
#include "rc_potocal.h"


/*********************************/
// can_2用作底盘控制
// 电机ID分别为1，2，3，4

/*********************************/

/*************全局变量***************/

float Chassis_PID[5] = {30, 1, 10, 16384, 16384};		//底盘PID在此修改
volatile int16_t Vx=0,Vy=0,Wz=0;	//底盘坐标系3变量
volatile int16_t target_speed_can_2[4];		//

/*************功率限制算法的变量定义***************/
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;
//用于键盘叠加
fp32 ramp[6] = {0,0,0,0,0,0};

/*************函数***************/
static void Chassis_PID_Init(float *PID);	//PID参数初始化
static void Chassis_loop_Init(); //数据清0
static void Chassis_mode();	//遥控器转底盘,底盘模式
static void Chassis_motol_speed_calculate(); //麦轮底盘解算
static void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed); 	//功率限制
static void Chassis_Power_Limit(double Chassis_pidout_target_limit); 	//输出解算
static void Chassis_current_give();   //发送电流
static void Chassis_keyboard_mode_1();		//正常开车行使模式
static void Chassis_keyboard_mode_2();		//以机械臂为正前方的行驶模式
static void Chassis_keyboard_mode_3();		//兑矿取矿模式
static void speed_ramp(); //底盘软启动


/*************主任务***************/
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	
	Chassis_PID_Init(Chassis_PID);		//PID参数初始化
	
  for(;;)
  {
		Chassis_loop_Init();	//三变量清零
		Chassis_mode();//遥控器信息转换为底盘速度Vy,Vx,Wz
		Chassis_motol_speed_calculate();	//麦轮运动逻辑解算
		Motor_Speed_limiting(target_speed_can_2,5000);//限制最大速度，输入参数是限制速度值(同比缩放)
		Chassis_Power_Limit(20000);									//输入参数要是Motor_Speed_limiting（）函数限制速度的4倍
		Chassis_current_give();                 //发送电流  
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
		
			if(rc_ctrl.rc.s[0] == 2)		//右拨杆置最下端，正常行驶模式
			{
				Chassis_keyboard_mode_1();			
			}
			else if (rc_ctrl.rc.s[0] == 3)		//右拨杆置中间，机械臂正前方模式
			{
				Chassis_keyboard_mode_2();
			}
			else if (rc_ctrl.rc.s[0] == 1)		//右拨杆置最上端，对矿取矿模式（wasd变成其他结构控制）
			{
				Chassis_keyboard_mode_3();
			}
		}
				// moving	control by remote
				//一直保留拨杆能移动（机械臂正前方）

        Vy = -rc_ctrl.rc.ch[3]/660.0*15000-  ramp[2] +  ramp[3];
        Vx = rc_ctrl.rc.ch[2]/660.0*15000 +  ramp[0] -  ramp[1];
        Wz = -rc_ctrl.rc.ch[4]/660.0*15000+  ramp[4] -  ramp[5] ;

}	

static void speed_ramp()
{
fp32 speed_max = 15000;
	uint8_t start = 10;
	//前进
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
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}

static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{	
	//819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
	Watch_Power_Max=Klimit;	Watch_Power=Hero_chassis_power;	Watch_Buffer=60;//限制值，功率值，缓冲能量值，初始值是1，0，0
	//get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

		Chassis_pidout_max=61536;//32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

		if(Watch_Power>960)	Motor_Speed_limiting(target_speed_can_2,4096);//限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
	else{
		Chassis_pidout=(
						fabs(target_speed_can_2[0]-motor_info_can_2[0].rotor_speed)+
						fabs(target_speed_can_2[1]-motor_info_can_2[1].rotor_speed)+
						fabs(target_speed_can_2[2]-motor_info_can_2[2].rotor_speed)+
						fabs(target_speed_can_2[3]-motor_info_can_2[3].rotor_speed));//fabs是求绝对值，这里获取了4个轮子的差值求和
		
//	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

		/*期望滞后占比环，增益个体加速度*/
		if(Chassis_pidout)
		{
		Scaling1=(target_speed_can_2[0]-motor_info_can_2[0].rotor_speed)/Chassis_pidout;	
		Scaling2=(target_speed_can_2[1]-motor_info_can_2[1].rotor_speed)/Chassis_pidout;
		Scaling3=(target_speed_can_2[2]-motor_info_can_2[2].rotor_speed)/Chassis_pidout;	
		Scaling4=(target_speed_can_2[3]-motor_info_can_2[3].rotor_speed)/Chassis_pidout;//求比例，4个scaling求和为1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*功率满输出占比环，车总增益加速度*/
//		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
//		else{Klimit = 0;}
		Klimit = Chassis_pidout/Chassis_pidout_target_limit;
		
		if(Klimit > 1) Klimit = 1 ;
		else if(Klimit < -1) Klimit = -1;//限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

		/*缓冲能量占比环，总体约束*/
		if(Watch_Buffer<50&&Watch_Buffer>=40)	Plimit=0.9;		//近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
		else if(Watch_Buffer<40&&Watch_Buffer>=35)	Plimit=0.75;
		else if(Watch_Buffer<35&&Watch_Buffer>=30)	Plimit=0.5;
		else if(Watch_Buffer<30&&Watch_Buffer>=20)	Plimit=0.25;
		else if(Watch_Buffer<20&&Watch_Buffer>=10)	Plimit=0.125;
		else if(Watch_Buffer<10&&Watch_Buffer>=0)	Plimit=0.05;
		else {Plimit=1;}
		
		motor_info_can_2[0].set_voltage = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//输出值
		motor_info_can_2[1].set_voltage = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_can_2[2].set_voltage = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_can_2[3].set_voltage = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*同比缩放电流*/

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
	//前进
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