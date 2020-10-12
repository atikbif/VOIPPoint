/*
 * uart1_protocol.c
 *
 *  Created on: 24 ��� 2019 �.
 *      Author: User
 */


#include "uart1_protocol.h"
#include "uart.h"
#include "main.h"
#include "din.h"

extern uint8_t button1;
extern uint8_t button2;
extern uint8_t limit_switch;
extern uint16_t discrete_state;

extern struct dinput di1,di2;

uint8_t di2_type = 0;


void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt==3 /*&& rx_ptr[0]+rx_ptr[1]==rx_ptr[2]*/) {
		switch(rx_ptr[0]) {
		case 0x41:
			if(rx_ptr[1]==0x0F) button1 = 1;else button1 = 0;
			break;
		case 0x42:
			if(rx_ptr[1]==0x0F) button2 = 1;else button2 = 0;
			break;
		case 0x43:	// ��������
			if(rx_ptr[1]==0x0F) {
				limit_switch = 1;
				discrete_state |= 1<<2;
			}else {
				limit_switch = 0;
				discrete_state &= ~(1<<2);
			}
			break;
		case 0x91:
			di2_type = rx_ptr[1] - 1;
			break;
		case 0x31:	// ���� 1
			if(di1.en_flag) {
				if(rx_ptr[1]==0x03) {	// �����
					discrete_state |= 1<<9;
					discrete_state &= ~(1<<10);
					discrete_state &= ~(1<<8);
					di1.state = LINE_BREAK;
					di1.tmr = 0;
				}else if(rx_ptr[1]==0x0C) {		// ���������
					di1.state = OFF;
					if(di1.tmr>=di1.tmr_limit) {
						discrete_state &= ~(1<<8);
						discrete_state &= ~(1<<9);
						discrete_state &= ~(1<<10);
					}

				}else if(rx_ptr[1]==0x30) {		// �������
					di1.state = ON;
					//if(di1.tmr>=di1.tmr_limit) {
					discrete_state |= 1<<8;
					discrete_state &= ~(1<<9);
					discrete_state &= ~(1<<10);
					//}
					di1.tmr = 0;
				}else if(rx_ptr[1]==0xC0) {		// ��
					discrete_state &= ~(1<<8);
					discrete_state &= ~(1<<9);
					discrete_state |= 1<<10;
					di1.state = SHORT_CIRC;
					di1.tmr = 0;
				}
			}else {
				di1.state = UNUSED;
				discrete_state &= ~(1<<8);
				discrete_state &= ~(1<<9);
				discrete_state &= ~(1<<10);
			}

			break;
		case 0x32:	// ���� 2
			if(di2.en_flag) {
				if(rx_ptr[1]==0x03) {	// �����
					discrete_state |= 1<<12;
					discrete_state &= ~(1<<13);
					discrete_state &= ~(1<<11);
					di2.state = LINE_BREAK;
					di2.tmr = 0;
				}else if(rx_ptr[1]==0x0C) {		// ���������
						di2.state = OFF;
						if(di2.tmr>=di2.tmr_limit) {
						discrete_state &= ~(1<<11);
						discrete_state &= ~(1<<12);
						discrete_state &= ~(1<<13);
					}
				}else if(rx_ptr[1]==0x30) {		// �������
					di2.state = ON;
					//if(di2.tmr>=di2.tmr_limit) {
						discrete_state |= 1<<11;
						discrete_state &= ~(1<<12);
						discrete_state &= ~(1<<13);
						di2.state = SHORT_CIRC;
						di2.tmr = 0;
					//}
				}else if(rx_ptr[1]==0xC0) {		// ��
					discrete_state &= ~(1<<11);
					discrete_state &= ~(1<<12);
					discrete_state |= 1<<13;
				}
			}else {
				di2.state = UNUSED;
				discrete_state &= ~(1<<11);
				discrete_state &= ~(1<<12);
				discrete_state &= ~(1<<13);
			}
			break;
		}
	}
}
