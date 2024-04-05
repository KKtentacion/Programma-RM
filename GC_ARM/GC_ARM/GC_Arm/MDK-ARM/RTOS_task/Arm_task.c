#include "cmsis_os.h"
#include "PID.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "tim.h"
#include "bsp_usb.h"

extern UART_HandleTypeDef huart6;
extern Motor_t MOTOR1_can1;
extern Motor_t MOTOR2_can1;
extern Motor_t MOTOR3_can1;
extern Motor_t MOTOR1_can2;
extern Motor_t MOTOR2_can2;
extern Motor_t MOTOR3_can2;

extern float Target_pos1_Can1;
extern float Target_pos2_Can1;
extern float Target_pos1_Can2;
extern float Target_pos2_Can2;
extern Vision_Recv recv;

extern float Max_pos1_Can1;
extern float Min_pos1_Can1;
extern float Max_pos2_Can1;
extern float Min_pos2_Can1;
extern float Max_pos1_Can2;
extern float Min_pos1_Can2;
extern float Max_pos2_Can2;
extern float Min_pos2_Can2;

extern float angle1_can1;
extern float angle1_can2;
extern float angle2_can2;
extern float angle2_can1;

float height=0;
float rotorangle2height=1;

float motor1_can1_zeropos = 0;
float motor2_can1_zeropos = 0;
float motor1_can2_zeropos = 0.8;
float motor2_can2_zeropos = -2.6;

float increment_can1_2=0.003;//0.003
float increment_can1_1=0.002;//0.002
static float increment=0.005;//0.003
float pid_pos1;
float pid_pos2;
float DM_v;

float recycle_angle1_can1_first=1.87;
float recycle_angle2_can1_first=2.13;
float recycle_angle1_can2_first=0.605;
float recycle_angle2_can2_first=-2.65;


float recycle_angle1_can1_second=1.87;
float recycle_angle2_can1_second=2.13;
float recycle_angle1_can2_second=0.605;
float recycle_angle2_can2_second=-2.65;

int16_t target_speed_can_1[4];
int16_t target_speed_can_2[4];
uint8_t start_flag;

uint8_t Mixed_mode=0;
uint8_t Count=0;


pid_struct_t DM_pid_1;
pid_struct_t DM_pid_2;
pid_struct_t DM_pid_3;
pid_struct_t DM_pid_4;
pid_struct_t up_down_pid;
pid_struct_t last_pid;
pid_struct_t up_down_pid_pos;


void DM_Enable(); //??????'??
void Arm_control();  //????????'??????'???

void One_Click_Control();

void Mixed_Control();
void Custom_control();
void Vision_control();

void Recyling();
void Restrict();
void MotorControl(float angle1_can1,float angle2_can1,float angle1_can2,float angle2_can2);
void z_init();
void dji_motor_get();
void dji_motor_control();
void dji_motor_get_pos();
void dji_motor_control_pos();
static void Sucker_Init();

void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  Sucker_Init();
  osDelay(1000);
  DM_Enable();
  osDelay(1);
  pid_init(&DM_pid_1,3,0.01,0.1,10,10);
  pid_init(&DM_pid_2,5,0.01,0.1,10,10);
  pid_init(&DM_pid_3,6,0.01,100,50,50);
  pid_init(&DM_pid_4,20.5,0.1,1,2,2);
  pid_init(&up_down_pid,10,0,0,30000,300000);
  pid_init(&up_down_pid_pos,10,0,0,30000,300000);
  pid_init(&last_pid,10,0.1,0,30000,300000);
	osDelay(1000);
  for(;;)
  {   
			//z_init();
		
			if(rc_ctrl.rc.s[0]==1)
			{
				One_Click_Control();
			}
			if(rc_ctrl.rc.s[0]==3)
			{
				Arm_control();
			}
			if(rc_ctrl.rc.s[0]==2)
			{
				Mixed_Control();
			}
			if(!b_flag&&!g_flag)
			{
				target_speed_can_2[0] = 0;
			}
			if(!shift_flag&&!ctrl_flag)
			{
				target_speed_can_1[0]=0;
			}
			
			osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

static void DM_Enable()
{ 
    osDelay(1);
    Enable_Ctrl2(0x101,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    osDelay(1);
    Enable_Ctrl2(0x102,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    osDelay(1);
    Enable_Ctrl(0x102,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    osDelay(1);
    Enable_Ctrl(0x101,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    osDelay(1);
}

static void Arm_control()
{
		if(q_flag!=0)
		{
			Target_pos1_Can1=Target_pos1_Can1+increment_can1_1;
		}
		else if(e_flag!=0)
		{
			Target_pos1_Can1=Target_pos1_Can1-increment_can1_1;	
		}
		
		if(w_flag!=0)
		{
			Target_pos1_Can2=Target_pos1_Can2+increment;
		}
		else if(s_flag!=0)
		{
			Target_pos1_Can2=Target_pos1_Can2-increment;
		}
		
		if(z_flag!=0)
		{
			Target_pos2_Can2=Target_pos2_Can2+increment;
		}
		else if(c_flag!=0)
		{
			Target_pos2_Can2=Target_pos2_Can2-increment;
		}
		
		if(a_flag!=0)
		{
			Target_pos2_Can1=Target_pos2_Can1+increment_can1_2;
		}
		else if(d_flag!=0)
		{
			Target_pos2_Can1=Target_pos2_Can1-increment_can1_2;
		}
		if (f_flag)
		{
			Recyling();
		} 
		Restrict(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
		MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
		osDelay(1);
		dji_motor_get();
		osDelay(1);
		dji_motor_control();
		osDelay(1);
}

static void Sucker_Init()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//?????????1??PWM???
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
	HAL_Delay(2000);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

void One_Click_Control()
{
		Target_pos1_Can1=0.71;
		Target_pos2_Can1=-0.82;
		Target_pos1_Can2=2.23;
		Target_pos2_Can2=0.59;
		MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
	
//    if(Count==0)
//    {
//        Target_pos1_Can1 = 0.28;
//        Target_pos2_Can1 = -0.36;
//        Target_pos1_Can2 = 2.12;
//        Target_pos2_Can2 = -2.6;
//        MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
//    }
//    if(Count==1)
//    {
//        Target_pos1_Can1 = 0.28;
//        Target_pos2_Can1 = -0.36;
//        Target_pos1_Can2 = 2.12;
//        Target_pos2_Can2 = -2.6;
//        MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
//    }
//    if(Count==2)
//    {
//        Target_pos1_Can1 = 0.28;
//        Target_pos2_Can1 = -0.36;
//        Target_pos1_Can2 = 2.12;
//        Target_pos2_Can2 = -2.6;
//        MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
//    }
//    if(f_flag)
//    {
//        Recyling();
//        Count++;
//    }
}

void Custom_control()
{
		Target_pos1_Can1=angle1_can1;
		Target_pos2_Can1=angle2_can1;
		Target_pos1_Can2=angle1_can2;
		Target_pos2_Can2=angle2_can2;
		
		Restrict(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
			
		MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
	
		dji_motor_get();
		osDelay(1);
		dji_motor_control();
		osDelay(1);
	
		if(q_flag)
    {
        Mixed_mode=0;
    }
		
}

void Vision_control()
{
		if(recv.header!=0xA5)
		{
			start_flag=165;
			USBTransmit(&start_flag,1);
		}
		else 
		{
			height=recv.z;
			
			Target_pos1_Can1=motor1_can1_zeropos-recv.angle1_can1;
			
			Target_pos1_Can2=motor1_can2_zeropos+recv.angle1_can2;
			
			Target_pos2_Can2=motor2_can2_zeropos+recv.angle2_can2;
			
			Target_pos2_Can1=motor2_can1_zeropos-recv.angle2_can1;
			
			dji_motor_get();
			osDelay(1);
			dji_motor_control();
			osDelay(1);
			
			Restrict(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
			
			MotorControl(Target_pos1_Can1,Target_pos2_Can1,Target_pos1_Can2,Target_pos2_Can2);
			
		}
		if(q_flag)
		{
			start_flag=160;
			USBTransmit(&start_flag,1);
			Mixed_mode=0;
		}
}

static void Mixed_Control()
{
	if(r_flag)
	{
		Mixed_mode=1;
	}
	else if(v_flag)
	{
		Mixed_mode=2;
	}
	if(Mixed_mode==1)
		Custom_control();
	if(Mixed_mode==2)
		Vision_control();
}

static void Restrict()
{
	Target_pos1_Can1=Target_pos1_Can1>Max_pos1_Can1?Max_pos1_Can1:Target_pos1_Can1;
	Target_pos1_Can1=Target_pos1_Can1<Min_pos1_Can1?Min_pos1_Can1:Target_pos1_Can1;
	
	Target_pos2_Can1=Target_pos2_Can1>Max_pos2_Can1?Max_pos2_Can1:Target_pos2_Can1;
	Target_pos2_Can1=Target_pos2_Can1<Min_pos2_Can1?Min_pos2_Can1:Target_pos2_Can1;
	
	Target_pos1_Can2=Target_pos1_Can2>Max_pos1_Can2?Max_pos1_Can2:Target_pos1_Can2;
	Target_pos1_Can2=Target_pos1_Can2<Min_pos1_Can2?Min_pos1_Can2:Target_pos1_Can2;
	
	Target_pos2_Can2=Target_pos2_Can2>Max_pos2_Can2?Max_pos2_Can2:Target_pos2_Can2;
	Target_pos2_Can2=Target_pos2_Can2<Min_pos2_Can2?Min_pos2_Can2:Target_pos2_Can2;
}

static void MotorControl(float angle1_can1,float angle2_can1,float angle1_can2,float angle2_can2)
{
	PosSpeed_CtrlMotor2(0x101,angle1_can2,0.5);		//zeropos -1.48
	osDelay(1);
	PosSpeed_CtrlMotor2(0x102,angle2_can2,0.5);		//zeropos -0.5
	osDelay(1);
	PosSpeed_CtrlMotor(0x101,angle1_can1,0.5);    //zeropos 0.46
	osDelay(1);
	PosSpeed_CtrlMotor(0x102,angle2_can1,0.5 );
	osDelay(1);
}

static void Recyling()
{
	Target_pos1_Can1=recycle_angle1_can1_first;
	Target_pos2_Can1=recycle_angle2_can1_first;
	Target_pos1_Can2=recycle_angle1_can2_first;
	Target_pos2_Can2=recycle_angle2_can2_first;
}

void dji_motor_get()
{
	if(ctrl_flag!=0)
	{
		target_speed_can_1[0]=21*45;
	}
	
	if(shift_flag!=0)
	{
		target_speed_can_1[0]=-21*45;
	}
	if(g_flag!=0)
	{
		target_speed_can_2[0]=10*45;
	}
	else if(b_flag!=0)
	{
		target_speed_can_2[0]=-10*45;
	}
	if(rc_ctrl.rc.s[1]==1)
	{
	 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000); 
	}
	if(rc_ctrl.rc.s[1]==3)
	{
	 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000); 
	}
}

void dji_motor_control(void)
{
   motor_info[0].set_voltage=pid_calc(&up_down_pid,target_speed_can_1[0],motor_info[0].rotor_speed);
   motor_info_can_2[0].set_voltage=pid_calc(&last_pid,target_speed_can_2[0],motor_info_can_2[0].rotor_speed);
	
   set_motor_voltage(0,motor_info[0].set_voltage,0,0,0);
	 osDelay(1);
   set_motor_voltage_can_2(0,motor_info_can_2[0].set_voltage,0,0,0);
	 osDelay(1);
}

void dji_motor_get_pos()
{
	if(g_flag!=0)
	{
	
	 target_speed_can_2[0]=19*45;
	}
	else if(b_flag!=0)
	{
	
	 target_speed_can_2[0]=-19*45;
	}
	else 
	{
	target_speed_can_2[0] = 0;

	}

	if(rc_ctrl.rc.s[1]==1)
	{
	 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000); 
	}
	if(rc_ctrl.rc.s[1]==3)
	{
	 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000); 
	}
    if(f_flag!=0)
    {
        Recyling();
    }
}

void dji_motor_control_pos()
{
	motor_info[0].set_voltage=pid_calc(&up_down_pid,pid_calc(&up_down_pid_pos,recv.z,height),motor_info[0].rotor_speed);
	motor_info_can_2[0].set_voltage=pid_calc(&last_pid,target_speed_can_2[0],motor_info_can_2[0].rotor_speed);
	
	set_motor_voltage(0,motor_info[0].set_voltage,0,0,0);
	osDelay(1);
	set_motor_voltage_can_2(0,motor_info_can_2[0].set_voltage,0,0,0);
	osDelay(1);
}

void z_init()
{
	if(!motor_info[0].initflag)
	{
		target_speed_can_1[0]=21*45;
		motor_info[0].set_voltage=pid_calc(&up_down_pid,target_speed_can_1[0],motor_info[0].rotor_speed);
		set_motor_voltage(0,motor_info[0].set_voltage,0,0,0);
	}
	else
	{
		target_speed_can_1[0]=0;
		height=0;
	}
}