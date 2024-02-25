#include "cmsis_os.h"
#include "PID.h"
#include "cmsis_os.h"
#include "judge.h"
#include "rc_potocal.h"
#include "pid.h"
#include "drv_can.h"
#include "tim.h"
#include "as5600_task.h"
#include "usart.h"
extern UART_HandleTypeDef huart6;
extern Motor_t MOTOR1_can1;
extern Motor_t MOTOR2_can1;
extern Motor_t MOTOR3_can1;
extern Motor_t MOTOR1_can2;
extern Motor_t MOTOR2_can2;
extern Motor_t MOTOR3_can2;

extern float Target_pos1_Can1;
extern float Target_pos1_Can2;
extern float Target_pos2_Can2;
extern float Target_pos3_Can2;

extern float angle1_can1;
extern float angle1_can2;
extern float angle2_can2;
extern float angle3_can2;
extern float as_flag;
int as_init_flag=1;

extern float Max_pos1_Can1;
extern float Min_pos1_Can1;
extern float Max_pos1_Can2;
extern float Min_pos1_Can2;
extern float Max_pos2_Can2;
extern float Min_pos2_Can2;
extern float Max_pos3_Can2;
extern float Min_pos3_Can2;

float motor1_can1_zeropos = 0.46;
float motor1_can2_zeropos = -1.48;
float motor2_can2_zeropos = 0; //-0.5;
float motor3_can2_zeropos = -0.015;

float setmotor1_can1;
float setmotor1_can2;
float setmotor2_can2;
float setmotor3_can2;

float increment_can2_3=0.0001;//0.003
float increment_can1_1=0.005;//0.002
static float increment=0.005;//0.003
float pid_pos1;
float pid_pos2;
float DM_v;
float angle1_can1_send;
float angle1_can2_send;
float angle2_can2_send;
float angle3_can2_send;
int16_t target_speed_can_1[4];
int16_t target_speed_can_2[4];


pid_struct_t DM_pid_1;
pid_struct_t DM_pid_2;
pid_struct_t DM_pid_3;
pid_struct_t DM_pid_4;
pid_struct_t up_down_pid;
pid_struct_t last_pid;





static void DM_Enable(); //达妙电机使能
static void DM_Init(); 
static void Arm_control();  //达妙电机初始化到初始位置
void dji_motor_control(void);
static void Sucker_Init();
static void as5600_Control();
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  Sucker_Init();
  HAL_Delay(1000);
  DM_Enable();
  HAL_Delay(1000);
  pid_init(&DM_pid_1,3,0.01,0.1,10,10);
  pid_init(&DM_pid_2,5,0.01,0.1,10,10);
  pid_init(&DM_pid_3,6,0.01,100,50,50);
  pid_init(&DM_pid_4,20.5,0.1,1,2,2);
  pid_init(&up_down_pid,10,0,0,30000,300000);
  pid_init(&last_pid,10,0.1,0,30000,300000);
//  DM_Init();
  for(;;)
  {   
		if(as_flag&&rc_ctrl.rc.s[0]==3)
		{
			if(as_init_flag)
			{
				PosSpeed_CtrlMotor(0x101,0,2);
				osDelay(100);
				PosSpeed_CtrlMotor2(0x101,0,5);
				osDelay(100);
				PosSpeed_CtrlMotor2(0x102,0,5);
				osDelay(100);
				PosSpeed_CtrlMotor2(0x103,0,5);
				osDelay(100);	
				as_init_flag=0;
			}
			as5600_Control();
		
			if(ctrl_flag==0&&shift_flag==0)
			{
				target_speed_can_1[0]=0;
			}
			
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
      
       target_speed_can_2[0]=19*45;
    }
     else if(b_flag!=0)
    {
      
       target_speed_can_2[0]=-19*45;
    }
		else {
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
			
			dji_motor_control();
			
			osDelay(1);
		}
		
		else
		{
			if(rc_ctrl.rc.s[0]==1)
			{
				Arm_control();
			}
			else if(rc_ctrl.rc.s[0]==2)
			{
				Target_pos1_Can1=motor1_can1_zeropos+(-1.43);
				Target_pos1_Can2=motor1_can2_zeropos+(1.5);
				Target_pos2_Can2=motor2_can2_zeropos+(-1.33);
				Target_pos3_Can2=motor3_can2_zeropos-(-0.01/25.f);
			}
			if(shift_flag==0&&ctrl_flag==0)
			{
			 target_speed_can_1[0]=0;
			}
			if(g_flag==0&&b_flag==0)
			{
			 target_speed_can_2[0]=0;
			}
			
			PosSpeed_CtrlMotor(0x101,Target_pos1_Can1,2);    //zeropos 0.46
			HAL_Delay(1);
			PosSpeed_CtrlMotor2(0x101,Target_pos1_Can2,5);		//zeropos -1.48
			HAL_Delay(1);
			PosSpeed_CtrlMotor2(0x102,Target_pos2_Can2,5);		//zeropos -0.5
			HAL_Delay(1);
			//MY_DMPos_Control(&DM_pid_3,0x203,Target_pos3_Can2,MOTOR3_can2.position);
			MIT_CtrlMotor2(0x03,Target_pos3_Can2,3,20,3.5,0 );//zeropos -0.0071
			//PosSpeed_CtrlMotor2(0x103,Target_pos3_Can2,5);
			HAL_Delay(1);
			dji_motor_control();
			uint8_t rxui[8] = "abddf";
			HAL_UART_Transmit(&huart6,rxui,8,0xff);
			osDelay(1);
		}
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
	Enable_Ctrl2(0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
	osDelay(1);
  Enable_Ctrl(0x101,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
}

static void DM_Init()
{        
//Expend Progress	
  MIT_CtrlMotor2(0x01,3.14,0.2,1,0.5,1);
  HAL_Delay(2000);
  MIT_CtrlMotor(0x01,-2.2,0.2,1,0.5,1);
  HAL_Delay(2000);
  MIT_CtrlMotor2(0x03,0.5,0.2,1,0.5,1);
  HAL_Delay(2000);
  MIT_CtrlMotor2(0x02,-5.3,0.2,1,0.5,1);
	HAL_Delay(2000);
		
//Recycle Progress
	MIT_CtrlMotor2(0x02,-1.7,0.01,1,0.5,1);
	HAL_Delay(2000);
	MIT_CtrlMotor2(0x03,-3.5,0.2,1,0.5,1);
	HAL_Delay(2000);
  MIT_CtrlMotor(0x01,-2.2,0.2,1,0.5,1);
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
		Target_pos1_Can1=Target_pos1_Can1>Max_pos1_Can1?Max_pos1_Can1:Target_pos1_Can1;
		Target_pos1_Can1=Target_pos1_Can1<Min_pos1_Can1?Min_pos1_Can1:Target_pos1_Can1;
		
		if(w_flag!=0)
		{
			Target_pos1_Can2=Target_pos1_Can2+increment;
		}
		else if(s_flag!=0)
		{
			Target_pos1_Can2=Target_pos1_Can2-increment;
		}
		Target_pos1_Can2=Target_pos1_Can2>Max_pos1_Can2?Max_pos1_Can2:Target_pos1_Can2;
		Target_pos1_Can2=Target_pos1_Can2<Min_pos1_Can2?Min_pos1_Can2:Target_pos1_Can2;
		
		if(z_flag!=0)
		{
			Target_pos2_Can2=Target_pos2_Can2+increment;
		}
		else if(c_flag!=0)
		{
			Target_pos2_Can2=Target_pos2_Can2-increment;
		}
		Target_pos2_Can2=Target_pos2_Can2>Max_pos2_Can2?Max_pos2_Can2:Target_pos2_Can2;
		Target_pos2_Can2=Target_pos2_Can2<Min_pos2_Can2?Min_pos2_Can2:Target_pos2_Can2;
		
		if(a_flag!=0)
		{
			Target_pos3_Can2=Target_pos3_Can2+increment_can2_3;
		}
		else if(d_flag!=0)
		{
			Target_pos3_Can2=Target_pos3_Can2-increment_can2_3;
		}
		Target_pos3_Can2=Target_pos3_Can2>Max_pos3_Can2?Max_pos3_Can2:Target_pos3_Can2;
		Target_pos3_Can2=Target_pos3_Can2<Min_pos3_Can2?Min_pos3_Can2:Target_pos3_Can2;
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
      
       target_speed_can_2[0]=19*45;
    }
     if(b_flag!=0)
    {
      
       target_speed_can_2[0]=-19*45;
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
   set_motor_voltage(0,motor_info[0].set_voltage,motor_info[1].set_voltage,0,0);
   set_motor_voltage_can_2(0,motor_info_can_2[0].set_voltage,0,0,0);
}

static void Sucker_Init()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//启动定时器1的PWM通道
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
	HAL_Delay(2000);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

static void as5600_Control()
{
	angle1_can1_send=-angle1_can1*3.14f/180.f;
	angle1_can2_send=-angle1_can2*3.14f/180.f;
	angle2_can2_send=-angle2_can2*3.14f/180.f;
	angle3_can2_send=-angle3_can2*3.14f/180.f/25.f;
	
	angle1_can1_send=angle1_can1_send>Max_pos1_Can1?Max_pos1_Can1:angle1_can1_send;
	angle1_can1_send=angle1_can1_send<Min_pos1_Can1?Min_pos1_Can1:angle1_can1_send;
	
	angle1_can2_send=angle1_can2_send>Max_pos1_Can2?Max_pos1_Can2:angle1_can2_send;
	angle1_can2_send=angle1_can2_send<Min_pos1_Can2?Min_pos1_Can2:angle1_can2_send;
	
	angle2_can2_send=angle2_can2_send>Max_pos2_Can2?Max_pos2_Can2:angle2_can2_send;
	angle2_can2_send=angle2_can2_send<Min_pos2_Can2?Min_pos2_Can2:angle2_can2_send;
	
	angle3_can2_send=angle3_can2_send>Max_pos3_Can2?Max_pos3_Can2:angle3_can2_send;
	angle3_can2_send=angle3_can2_send<Min_pos3_Can2?Min_pos3_Can2:angle3_can2_send;
	
	PosSpeed_CtrlMotor(0x101,angle1_can1_send,0.5);
	osDelay(1);
	PosSpeed_CtrlMotor2(0x101,angle1_can2_send,0.5);
	osDelay(1);
	PosSpeed_CtrlMotor2(0x102,angle2_can2_send,5);
	osDelay(1);
	MIT_CtrlMotor2(0x03,angle3_can2_send,3,20,3.5,0);
	
	osDelay(1);	
}