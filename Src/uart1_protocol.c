/*
 * uart1_protocol.c
 *
 *  Created on: 24 ��� 2019 �.
 *      Author: User
 */


#include "uart1_protocol.h"
#include "uart.h"
#include "main.h"

extern uint8_t button1;
extern uint8_t button2;
extern uint8_t limit_switch;
extern uint16_t discrete_state;


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
		case 0x31:	// ���� 1
			if(rx_ptr[1]==0x03) {	// �����
				discrete_state |= 1<<9;
				discrete_state &= ~(1<<10);
				discrete_state &= ~(1<<8);
			}else if(rx_ptr[1]==0x0C) {		// ���������
				discrete_state &= ~(1<<8);
				discrete_state &= ~(1<<9);
				discrete_state &= ~(1<<10);
			}else if(rx_ptr[1]==0x30) {		// �������
				discrete_state |= 1<<8;
				discrete_state &= ~(1<<9);
				discrete_state &= ~(1<<10);
			}else if(rx_ptr[1]==0xC0) {		// ��
				discrete_state &= ~(1<<8);
				discrete_state &= ~(1<<9);
				discrete_state |= 1<<10;
			}
			break;
		case 0x32:	// ���� 2
			if(rx_ptr[1]==0x03) {	// �����
				discrete_state |= 1<<12;
				discrete_state &= ~(1<<13);
				discrete_state &= ~(1<<11);
			}else if(rx_ptr[1]==0x0C) {		// ���������
				discrete_state &= ~(1<<11);
				discrete_state &= ~(1<<12);
				discrete_state &= ~(1<<13);
			}else if(rx_ptr[1]==0x30) {		// �������
				discrete_state |= 1<<11;
				discrete_state &= ~(1<<12);
				discrete_state &= ~(1<<13);
			}else if(rx_ptr[1]==0xC0) {		// ��
				discrete_state &= ~(1<<11);
				discrete_state &= ~(1<<12);
				discrete_state |= 1<<13;
			}
			break;
		}
	}
}
