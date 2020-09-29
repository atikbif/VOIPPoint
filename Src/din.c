/*
 * din.c
 *
 *  Created on: 23 сент. 2020 г.
 *      Author: User
 */

#include "din.h"

struct dinput di1,di2;

void init_dinputs(uint64_t v) {
	v = v>>3;
	if(v&(0x0001<<8)) di1.en_flag = 1;else di1.en_flag = 0;
	di1.tmr_limit = v&0x0F;
	if(di1.tmr_limit>10) di1.tmr_limit=10;
	di1.state = OFF;
	di1.tmr = 0;

	if(v&(0x0001<<9)) di2.en_flag = 1;else di2.en_flag = 0;
	di2.tmr_limit = (v>>4)&0x0F;
	if(di2.tmr_limit>10) di2.tmr_limit=10;
	di2.state = OFF;
	di2.tmr = 0;
}
