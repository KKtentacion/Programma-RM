#include "drv_can.h"
#include "main.h"
#include "rc_potocal.h"
#include "chassis_control.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
uint16_t can_cnt_1=0;
extern uint16_t Up_ins_yaw;

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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if ((rx_header.StdId >= FEEDBACK_ID_BASE)//201-207
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // 判断标识符，标识符为0x200+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
  }
  }
	 if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
		if ((rx_header.StdId >= FEEDBACK_ID_BASE)//201-207
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // 判断标识符，标识符为0x200+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
  }

}

