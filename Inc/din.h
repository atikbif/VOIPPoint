/*
 * din.h
 *
 *  Created on: 23 сент. 2020 г.
 *      Author: User
 */

#ifndef DIN_H_
#define DIN_H_

#include <stdint.h>

enum DI_STATE{ON,OFF,SHORT_CIRC,LINE_BREAK,UNUSED};

struct dinput{
	uint8_t tmr;
	enum DI_STATE state;
	uint8_t en_flag;
	uint8_t tmr_limit;
};

void init_dinputs(uint64_t v);

#endif /* DIN_H_ */
