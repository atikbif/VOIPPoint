/*
 * uart1_protocol.c
 *
 *  Created on: 24 мая 2019 г.
 *      Author: User
 */


#include "uart1_protocol.h"
#include "uart.h"
#include "main.h"

extern unsigned short device_id;
extern uint8_t button1;
extern uint8_t button2;

void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt==3 /*&& rx_ptr[0]+rx_ptr[1]==rx_ptr[2]*/) {
		switch(rx_ptr[0]) {
		case 0x41:
			if(rx_ptr[1]==0x0F) button1 = 1;else button1 = 0;
			break;
		case 0x42:
			if(rx_ptr[1]==0x0F) button2 = 1;else button2 = 0;
			break;
		case 0x50:
			device_id = rx_ptr[1];
			break;
		}
	}
}
