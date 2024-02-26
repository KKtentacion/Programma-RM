#include "drv_can.h"
#include "main.h"
#include "rc_potocal.h"
#include "pid.h"
uint16_t flag_can=0;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//extern RC_ctrl_t rc_ctrl;
extern uint16_t Up_ins_yaw;
Motor_t MOTOR1_can1;
Motor_t MOTOR2_can1;
Motor_t MOTOR3_can1;
Motor_t MOTOR1_can2;
Motor_t MOTOR2_can2;
Motor_t MOTOR3_can2;

int initflag1_Can1=0;
int initflag1_Can2=0;
int initflag2_Can2=0;
int initflag3_Can2=0;

float Max_pos1_Can1=0.42;
float Min_pos1_Can1=-1.5;
float Max_pos1_Can2=0.5;
float Min_pos1_Can2=-1.5;
float Max_pos2_Can2=1.03;
float Min_pos2_Can2=-2.15;
float Max_pos3_Can2=-0.5; //旧电机max=0.56
float Min_pos3_Can2=-5.6;  //旧电机min=-0.16 

float Target_pos1_Can1;
float Target_pos1_Can2;
float Target_pos2_Can2;
float Target_pos3_Can2;
CAN_RxPacketTypeDef ss;
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void CAN1_Init(void)
{
    CAN_FilterTypeDef  can_filter;

    can_filter.FilterBank = 0;                       // filter 0
    can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
    can_filter.FilterIdHigh = 0;//标识符寄存器 
    can_filter.FilterIdLow  = 0;//标识符寄存器 
    can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
    can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
    can_filter.FilterActivation = ENABLE;           // enable can filter
    can_filter.SlaveStartFilterBank  = 14;          
   
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断
    HAL_CAN_Start(&hcan1);//启动can1

}

void CAN2_Init( void )
{
    CAN_FilterTypeDef  can_filter;

    can_filter.FilterBank = 14;                       // filter 14
    can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
    can_filter.FilterIdHigh = 0;//标识符寄存器 
    can_filter.FilterIdLow  = 0;//标识符寄存器 
    can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
    can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
    can_filter.FilterActivation = ENABLE;           // enable can filter
    can_filter.SlaveStartFilterBank  = 14;         
   
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);        // init can filter
   	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	  HAL_CAN_Start(&hcan2);//启动can2
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
	static CAN_RxPacketTypeDef packet;
    // CAN数据接收
    if (canHandle->Instance ==  CAN1)
    {
        if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		//获得接收到的数据头和数据
        {      
            if(packet.payload[0] == 0x01)
           {        //接收MOTOR1数据         
							MOTOR1_can1.p_int=(packet.payload[1]<<8)|packet.payload[2];
              MOTOR1_can1.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
              MOTOR1_can1.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
              MOTOR1_can1.position = uint_to_float(MOTOR1_can1.p_int, P_MIN, P_MAX, 16); 
            	 if(initflag1_Can1==0)
						 {
							Target_pos1_Can1=MOTOR1_can1.position;
							initflag1_Can1=1;
						 }
						 
            }  
               
		           if ((packet.hdr.StdId >= FEEDBACK_ID_BASE)//201-207
              && (packet.hdr.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // 判断标识符，标识符为0x200+ID
							{
								uint8_t index = packet.hdr.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
								motor_info[index].rotor_angle    = ((packet.payload[0] << 8) | packet.payload[1]);
								motor_info[index].rotor_speed    = ((packet.payload[2] << 8) | packet.payload[3]);
								motor_info[index].torque_current = ((packet.payload[4] << 8) | packet.payload[5]);
								motor_info[index].temp           =   packet.payload[6];
							}
//              else if(packet.payload[0] == 0x03){   //接收MOTOR3数据
//							MOTOR3_can1.p_int=(packet.payload[1]<<8)|packet.payload[2];
//              MOTOR3_can1.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
//              MOTOR3_can1.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
//              MOTOR3_can1.position = uint_to_float(MOTOR3_can1.p_int, P_MIN, P_MAX, 16); 
////              MOTOR3_t.position = uint_to_float(MOTOR3_t.p_int, P_MIN, P_MAX, 16); 
////              MOTOR3_t.velocity = uint_to_float(MOTOR3_t.v_int, V_MIN, V_MAX, 12); 
////              MOTOR3_t.torque = uint_to_float(MOTOR3_t.position, T_MIN, T_MAX, 12); 
//            }
            HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收中断
        }
    }
    if (canHandle->Instance ==  CAN2)
    {
        if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		//获得接收到的数据头和数据
        {     
            if(packet.payload[0] == 0x01)
           {        //接收MOTOR1数据     
              ss=packet;    
							MOTOR1_can2.p_int=(packet.payload[1]<<8)|packet.payload[2];
              MOTOR1_can2.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
              MOTOR1_can2.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
              MOTOR1_can2.position = uint_to_float(MOTOR1_can2.p_int, P_MIN, P_MAX, 16); 
//              MOTOR1_t.position = uint_to_float(MOTOR1_t.p_int, P_MIN, P_MAX, 16); 
//              MOTOR1_t.velocity = uint_to_float(MOTOR1_t.v_int, V_MIN, V_MAX, 12);
//              MOTOR1_t.torque = uint_to_float(MOTOR1_t.t_int, T_MIN, T_MAX, 12);
              if(initflag1_Can2==0)
						 {
							Target_pos1_Can2=MOTOR1_can2.position;
							initflag1_Can2=1;
						 } 
            }
             else if(packet.payload[0] == 0x02)
            {  //接收MOTOR2数据
							MOTOR2_can2.p_int=(packet.payload[1]<<8)|packet.payload[2];
              MOTOR2_can2.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
              MOTOR2_can2.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
              MOTOR2_can2.position = uint_to_float(MOTOR2_can2.p_int, P_MIN, P_MAX, 16); 
//              MOTOR2_t.position = uint_to_float(MOTOR2_t.p_int, P_MIN, P_MAX, 16); 
//              MOTOR2_t.velocity = uint_to_float(MOTOR2_t.v_int, V_MIN, V_MAX, 12); 
//              MOTOR2_t.torque = uint_to_float(MOTOR2_t.t_int, T_MIN, T_MAX, 12);
              if(initflag2_Can2==0)
						 {
							Target_pos2_Can2=MOTOR2_can2.position;
							initflag2_Can2=1;
						 } 
 
            }
               else if(packet.payload[0] == 0x03)
            {   //接收MOTOR3数据
							flag_can=1;
							MOTOR3_can2.p_int=(packet.payload[1]<<8)|packet.payload[2];
              MOTOR3_can2.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
              MOTOR3_can2.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
              MOTOR3_can2.position = uint_to_float(MOTOR3_can2.p_int, P_MIN, P_MAX, 16); 
//              MOTOR3_t.position = uint_to_float(MOTOR3_t.p_int, P_MIN, P_MAX, 16); 
//              MOTOR3_t.velocity = uint_to_float(MOTOR3_t.v_int, V_MIN, V_MAX, 12); 
//              MOTOR3_t.torque = uint_to_float(MOTOR3_t.position, T_MIN, T_MAX, 12);
               if(initflag3_Can2==0)
						 {
							 Target_pos3_Can2=MOTOR3_can2.position;//*5.625;   //旧电机两个数据实际位置0.064目标位置0.36存在倍数差值
							initflag3_Can2=1;
						 } 

            }
            else if ((packet.hdr.StdId >= FEEDBACK_ID_BASE)//201-207
              && (packet.hdr.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // 判断标识符，标识符为0x200+ID
							{
								uint8_t index = packet.hdr.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
								motor_info_can_2[index].rotor_angle    = ((packet.payload[0] << 8) | packet.payload[1]);
								motor_info_can_2[index].rotor_speed    = ((packet.payload[2] << 8) | packet.payload[3]);
								motor_info_can_2[index].torque_current = ((packet.payload[4] << 8) | packet.payload[5]);
								motor_info_can_2[index].temp           =   packet.payload[6];
							}
            HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收中断
        }
    }
}


float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
