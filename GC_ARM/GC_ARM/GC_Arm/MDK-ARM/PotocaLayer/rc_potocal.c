#include "rc_potocal.h"
#include "drv_can.h"
#include "main.h"
#include "rc_potocal.h"
#include "pid.h"

RC_ctrl_t rc_ctrl;//定义结构体变量用来存储遥控器数据


//遥控器标志位
uint8_t w_flag;
uint8_t s_flag;
uint8_t a_flag;
uint8_t d_flag;
uint8_t q_flag;
uint8_t e_flag;
uint8_t shift_flag;
uint8_t ctrl_flag;
uint8_t r_flag;
uint8_t f_flag;
uint8_t g_flag;
uint8_t z_flag;
uint8_t x_flag;
uint8_t c_flag;
uint8_t v_flag;
uint8_t b_flag;


void remote_data_read(uint8_t rx_buffer[])
{
	  rc_ctrl.rc.ch[0] = (rx_buffer[0] | (rx_buffer[1] << 8)) & 0x07ff;        //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
    rc_ctrl.rc.ch[1] = (((rx_buffer[1] >> 3)&0xff) | (rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = (((rx_buffer[2] >> 6)&0xff) | (rx_buffer[3] << 2) |          //!< Channel 2
                         (rx_buffer[4] << 10)) &0x07ff;
    rc_ctrl.rc.ch[3] = (((rx_buffer[4] >> 1)&0xff) | (rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((rx_buffer[5] >> 4) & 0x0003);                  //!< Switch left！！！这尼玛是右
    rc_ctrl.rc.s[1] = ((rx_buffer[5] >> 4) & 0x000C) >> 2;    		//!< Switch right！！！这才是左
    rc_ctrl.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse.press_l = rx_buffer[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = rx_buffer[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key.v = rx_buffer[14] | (rx_buffer[15] << 8);                    //!< KeyBoard value	
    rc_ctrl.rc.ch[4] = rx_buffer[16] | (rx_buffer[17] << 8);                 //NULL
	
		w_flag=(rx_buffer[14]&0x01);
		s_flag=(rx_buffer[14]&0x02);
		a_flag=(rx_buffer[14]&0x04);
		d_flag=(rx_buffer[14]&0x08);
		q_flag=(rx_buffer[14]&0x40);
		e_flag=(rx_buffer[14]&0x80);
		shift_flag=(rx_buffer[14]&0x10);
		ctrl_flag=(rx_buffer[14]&0x20);
		
		r_flag=(rx_buffer[15]&0x01);
		f_flag=(rx_buffer[15]&0x02);
		g_flag=(rx_buffer[15]&0x04);
		z_flag=(rx_buffer[15]&0x08);
		x_flag=(rx_buffer[15]&0x10);
		c_flag=(rx_buffer[15]&0x20);
		v_flag=(rx_buffer[15]&0x40);
		b_flag=(rx_buffer[15]&0x80);
}

