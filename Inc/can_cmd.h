/*
 * can_cmd.h
 *
 *  Created on: 23 рту. 2019 у.
 *      Author: User
 */

#ifndef CAN_CMD_H_
#define CAN_CMD_H_

#include <stdint.h>

#define		UNKNOWN_TYPE	0
#define		PC_TO_ALL		1
#define		PC_TO_GROUP		2
#define		PC_TO_POINT		3
#define		POINT_TO_ALL	4
#define		POINT_TO_PC		5
#define		UNUSED_TYPE		6

#define		AUDIO_PACKET	1
#define		FIND_NEXT_POINT	2
#define		SCAN_GROUP		3
#define		POINT_STATE		4
#define		SET_OUTS		5
#define		LAST_POINT		6

#define		FIND_REQUEST	1
#define		FIND_ANSWER		2

typedef struct
{
 uint32_t param: 8;
 uint32_t cmd: 4;
 uint32_t group_addr: 7;
 uint32_t point_addr: 7;
 uint32_t type: 3;
 uint32_t unused_bits : 3;
} id_field;

void send_point_state(uint8_t can_num);
void next_point(uint8_t t);	// t -type (1 - request, 2 - answer)
void last_point();

#endif /* CAN_CMD_H_ */
