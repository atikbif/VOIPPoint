/*
 * uart1_protocol.c
 *
 *  Created on: 24 мая 2019 г.
 *      Author: User
 */


#include "uart1_protocol.h"
#include "uart.h"
#include "main.h"

extern uint8_t button1;
extern uint8_t button2;
extern uint8_t button3;
extern uint16_t discrete_state;


void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt==3 /*&& rx_ptr[0]+rx_ptr[1]==rx_ptr[2]*/) {
		switch(rx_ptr[0]) {
		case 0x41:
			if(rx_ptr[1]==0x0F) button1 = 1;else button1 = 0;
			break;
		case 0x42:
			if(rx_ptr[1]==0x0F) button2 = 1;else button2 = 0;
		case 0x43:
			if(rx_ptr[1]==0x0F) button3 = 1;else button3 = 0;
			break;
		case 0x31:
			if(rx_ptr[1]==0x03) {	// обрыв
				discrete_state |= 1<<9;
				discrete_state &= ~(1<<10);
				discrete_state &= ~(1<<8);
			}else if(rx_ptr[1]==0x0C) {		// разомкнут
				discrete_state &= ~(1<<8);
				discrete_state &= ~(1<<9);
				discrete_state &= ~(1<<10);
			}else if(rx_ptr[1]==0x30) {		// замкнут
				discrete_state |= 1<<8;
				discrete_state &= ~(1<<9);
				discrete_state &= ~(1<<10);
			}else if(rx_ptr[1]==0xC0) {		// кз
				discrete_state &= ~(1<<8);
				discrete_state &= ~(1<<9);
				discrete_state |= 1<<10;
			}
			break;
		case 0x32:
			if(rx_ptr[1]==0x03) {	// обрыв
				discrete_state |= 1<<12;
				discrete_state &= ~(1<<13);
				discrete_state &= ~(1<<11);
			}else if(rx_ptr[1]==0x0C) {		// разомкнут
				discrete_state &= ~(1<<11);
				discrete_state &= ~(1<<12);
				discrete_state &= ~(1<<13);
			}else if(rx_ptr[1]==0x30) {		// замкнут
				discrete_state |= 1<<11;
				discrete_state &= ~(1<<12);
				discrete_state &= ~(1<<13);
			}else if(rx_ptr[1]==0xC0) {		// кз
				discrete_state &= ~(1<<11);
				discrete_state &= ~(1<<12);
				discrete_state |= 1<<13;
			}
			break;
		}
	}
}
