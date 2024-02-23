#include "PID.h"
uint16_t flag_enable=0;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Motor_t MOTOR1_can1;
extern Motor_t MOTOR2_can1;
extern Motor_t MOTOR3_can1;
extern Motor_t MOTOR1_can2;
extern Motor_t MOTOR2_can2;
extern Motor_t MOTOR3_can2;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)//ref是目标值,fdb是电机解码的速度返回值
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];//err[1]是上一次计算出来的差值
  pid->err[0] = pid->ref - pid->fdb;//err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0是标准值，把这个加到watch1里面
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//防止越界
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//防止越界
  return pid->output;//电机返回的报文有转速和转矩电流，但是只能发电压值(-30000至30000)，有点抽象这个PID
}

void set_motor_voltage(uint8_t id_range, int32_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x2ff);//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

//	tx_data[0] = (v1)&0xff;	//先发di八位		
//  tx_data[1] = (v1>>8)&0xff;
//  tx_data[2] = (v1>>16)&0xff;
//  tx_data[3] = (v1>>24)&0xff;
//  tx_data[4] = (v3>>8)&0xff;
//  tx_data[5] =    (v3)&0xff;
//  tx_data[6] = (v4>>8)&0xff;
//  tx_data[7] =   (v4)&0xff;
	tx_data[0] = (v1>>8)&0xff;	//先发高八位		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

/**
 * @brief  电机命令
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID      
 * @param  data   命令帧
 */
 void Enable_Ctrl(uint16_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7)
 {
    static CAN_TxPacketTypeDef packet;
    
    packet.hdr.StdId = ID;
		packet.hdr.ExtId=0;
		packet.hdr.IDE=0;
		packet.hdr.RTR=0;
    packet.hdr.DLC = 8;
    packet.payload[0] = (uint8_t)data0;
    packet.payload[1] = (uint8_t)data1;
    packet.payload[2] = (uint8_t)data2;
    packet.payload[3] = (uint8_t)data3;
    packet.payload[4] = (uint8_t)data4;
    packet.payload[5] = (uint8_t)data5;
    packet.payload[6] = (uint8_t)data6;
    packet.payload[7] = (uint8_t)data7;

    /*找到空的发送邮箱，把数据发送出去*/
	  if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
 
 void Enable_Ctrl2(uint16_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7)
 {
		flag_enable=1;
	 
    static CAN_TxPacketTypeDef packet;
    
    packet.hdr.StdId = ID;
		packet.hdr.ExtId=0;
		packet.hdr.IDE=0;
		packet.hdr.RTR=0;
    packet.hdr.DLC = 8;
    packet.payload[0] = (uint8_t)data0;
    packet.payload[1] = (uint8_t)data1;
    packet.payload[2] = (uint8_t)data2;
    packet.payload[3] = (uint8_t)data3;
    packet.payload[4] = (uint8_t)data4;
    packet.payload[5] = (uint8_t)data5;
    packet.payload[6] = (uint8_t)data6;
    packet.payload[7] = (uint8_t)data7;

    /*找到空的发送邮箱，把数据发送出去*/
	  if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
 
void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x2ff);//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (v1>>8)&0xff;	//先发高八位		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
	
	
        /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
    return 0;
}

void Speed_CtrlMotor(uint16_t ID, float _vel)
{
    static CAN_TxPacketTypeDef packet;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x04;

    packet.payload[0] = *vbuf;
    packet.payload[1] = *(vbuf+1);
    packet.payload[2] = *(vbuf+2);
    packet.payload[3] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
void Speed_CtrlMotor1(uint16_t ID, float _vel)
{
    static CAN_TxPacketTypeDef packet;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x04;

    packet.payload[0] = *vbuf;
    packet.payload[1] = *(vbuf+1);
    packet.payload[2] = *(vbuf+2);
    packet.payload[3] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

void PosSpeed_CtrlMotor( uint16_t ID, float _pos, float _vel)
{
    static CAN_TxPacketTypeDef packet;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    packet.payload[0] = *pbuf;;
    packet.payload[1] = *(pbuf+1);
    packet.payload[2] = *(pbuf+2);
    packet.payload[3] = *(pbuf+3);
    packet.payload[4] = *vbuf;
    packet.payload[5] = *(vbuf+1);
    packet.payload[6] = *(vbuf+2);
    packet.payload[7] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
void PosSpeed_CtrlMotor2( uint16_t ID, float _pos, float _vel)
{
    static CAN_TxPacketTypeDef packet;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    packet.payload[0] = *pbuf;;
    packet.payload[1] = *(pbuf+1);
    packet.payload[2] = *(pbuf+2);
    packet.payload[3] = *(pbuf+3);
    packet.payload[4] = *vbuf;
    packet.payload[5] = *(vbuf+1);
    packet.payload[6] = *(vbuf+2);
    packet.payload[7] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

void MIT_CtrlMotor(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
    static CAN_TxPacketTypeDef packet;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    
    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(_vel, -45, 45, 12);
    kp_tmp = float_to_uint(_KP, 0, 500, 12);
    kd_tmp = float_to_uint(_KD, 0, 5, 12);
    tor_tmp = float_to_uint(_torq, -18, 18, 12);

    packet.payload[0] = (pos_tmp >> 8);
    packet.payload[1] = pos_tmp;
    packet.payload[2] = (vel_tmp >> 4);
    packet.payload[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    packet.payload[4] = kp_tmp;
    packet.payload[5] = (kd_tmp >> 4);
    packet.payload[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    packet.payload[7] = tor_tmp;

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan1, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

void MIT_CtrlMotor2(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
    static CAN_TxPacketTypeDef packet;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    
    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(_vel, -45, 45, 12);
    kp_tmp = float_to_uint(_KP, 0, 500, 12);
    kd_tmp = float_to_uint(_KD, 0, 5, 12);
    tor_tmp = float_to_uint(_torq, -18, 18, 12);

    packet.payload[0] = (pos_tmp >> 8);
    packet.payload[1] = pos_tmp;
    packet.payload[2] = (vel_tmp >> 4);
    packet.payload[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    packet.payload[4] = kp_tmp;
    packet.payload[5] = (kd_tmp >> 4);
    packet.payload[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    packet.payload[7] = tor_tmp;

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan2, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

//Add Integral calculate
void MY_MIT_CtrlMotor(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
	switch (ID)
	{
		case 1:
		{
			while(Abs(MOTOR1_can1.position-_pos)>=0.005)
			{
				MIT_CtrlMotor(ID,_pos,_vel,_KP,_KD,_torq);
				_pos=_pos+0.05;
			}
		}
		case 2:
		{
			while(Abs(MOTOR2_can1.position-_pos)>=0.005)
			{
				MIT_CtrlMotor(ID,_pos,_vel,_KP,_KD,_torq);
				_pos=_pos+0.05;
			}
		}
		case 3:
		{
			while(Abs(MOTOR3_can1.position-_pos)>=0.005)
			{
				MIT_CtrlMotor(ID,_pos,_vel,_KP,_KD,_torq);
				_pos=_pos+0.05;
			}
		}
		break;
	}	
}

void MY_MIT_CtrlMotor2(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
	switch (ID)
	{
		case 1:
		{
			while(Abs(MOTOR1_can2.position-_pos)>=0.005)
			{
				MIT_CtrlMotor2(ID,_pos,_vel,_KP,_KD,_torq);
				_pos=_pos+0.05;
			}
		}
		case 2:
		{
			while(Abs(MOTOR2_can2.position-_pos)>=0.005)
			{
				MIT_CtrlMotor2(ID,_pos,_vel,_KP,_KD,_torq);
				_pos=_pos+0.05;
			}
		}
		case 3:
		{
			while(Abs(MOTOR3_can2.position-_pos)>=0.005)
			{
				MIT_CtrlMotor2(ID,_pos,_vel,_KP,_KD,_torq);
				_pos=_pos+0.05;
			}
		}
		break;
	}	
}

float Abs(float num)
{
	return num>=0?num:-num;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{ 
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
