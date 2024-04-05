/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "drv_can.h"

Vision_Recv recv;

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
		
    CDC_Transmit_FS(buffer, len); // 发送
}

void JOINT()
{
		if (bsp_usb_rx_buffer[0] != 0xA5)
			return;
		if (bsp_usb_rx_buffer[0] == 0xA5)
		{
			uint8_t *rx_buff = bsp_usb_rx_buffer;
			
			recv.header = rx_buff[0];
			memcpy(&recv.angle1_can1,&rx_buff[1],4);
			memcpy(&recv.angle2_can1,&rx_buff[5],4);
			memcpy(&recv.angle2_can2,&rx_buff[9],4);
			memcpy(&recv.angle1_can2,&rx_buff[13],4);
//			memcpy(&recv2.z,&rx_buff[5],4);
//			memcpy(&recv2.angle_2006,&rx_buff[5],4);
		}
}
