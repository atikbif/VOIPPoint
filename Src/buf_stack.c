/*
 * buf_stack.c
 *
 *  Created on: 20 рту. 2019 у.
 *      Author: User
 */

#include "buf_stack.h"
#include <cmsis_gcc.h>
#include "main.h"

#define EMPTY_PACKET	0
#define BUSY_PACKET		1
#define READY_PACKET	2

static uint8_t is_stack_full(buf_stack *stack) {
	uint8_t cnt = 0;
	uint8_t i = 0;
	for(i=0;i<STACK_SIZE;i++) {
		if(stack->coils[i].state!=EMPTY_PACKET) cnt++;
	}
	if(cnt==STACK_SIZE) return 1;
	return 0;
}

static uint16_t atomicWrPosIncrement(uint16_t * ptr)
{
	uint16_t oldValue, newValue;
	uint8_t try_num = 0;
	do
	{
		oldValue = __LDREXH(ptr);
		try_num++;
		if(try_num>3) break;
		newValue = oldValue + 1;
		if(newValue>=STACK_SIZE) newValue = 0;
	}while(__STREXH(newValue, ptr));
	return oldValue;
}

void init_buf_stack(buf_stack *stack, uint8_t *data_ptr,uint16_t max_coil_length) {
	uint16_t i=0;
	uint16_t offset = 0;
	stack->rd_pos = 0;
	stack->wr_pos = 0;
	stack->max_coil_data_length = max_coil_length;
	stack->coils->data = data_ptr;
	for(i=0;i<STACK_SIZE;i++) {
		stack->coils[i].state = EMPTY_PACKET;
		stack->coils[i].length = 0;
		stack->coils[i].data = data_ptr + offset;
		offset += max_coil_length;
	}
}

uint8_t add_data_to_stack(buf_stack *stack, uint8_t *data, uint16_t length) {
	uint16_t i = 0;
	uint16_t wr_pos = 0;
	uint8_t try_num = 0;
	if(length>stack->max_coil_data_length) length = stack->max_coil_data_length;
	wr_pos = atomicWrPosIncrement(&(stack->wr_pos));
	//do{
	if(stack->coils[wr_pos].state!=EMPTY_PACKET && is_stack_full(stack)) {
		stack->rd_pos = 0;
		stack->wr_pos = 0;
		for(i=0;i<STACK_SIZE;i++) {
			stack->coils[i].state = EMPTY_PACKET;
			stack->coils[i].length = 0;
		}
	}
		try_num++;if(try_num>3) return 0;
		__LDREXB(&(stack->coils[wr_pos].state));
		stack->coils[wr_pos].state = BUSY_PACKET;
		for(i=0;i<length;++i) stack->coils[wr_pos].data[i] = data[i];
		stack->coils[wr_pos].length = length;
		stack->coils[wr_pos].state = READY_PACKET;
	//}while(__STREXB(READY_PACKET, &(stack->coils[wr_pos].state)));
	return 1;
}

uint16_t get_data_from_stack(buf_stack *stack, uint8_t *ptr) {
	uint16_t i = 0;
	uint16_t res = 0;
	uint16_t rd_pos = 0;
	uint8_t try_num = 0;

	if(stack->coils[stack->rd_pos].state==READY_PACKET) {
		rd_pos = atomicWrPosIncrement(&(stack->rd_pos));
		//do{
			try_num++;if(try_num>3) return 0;
			__LDREXB(&(stack->coils[rd_pos].state));
			stack->coils[rd_pos].state = BUSY_PACKET;
			res = stack->coils[rd_pos].length;
			for(i=0;i<res;++i) ptr[i] = stack->coils[rd_pos].data[i];
			stack->coils[rd_pos].state = EMPTY_PACKET;
		//}while(__STREXB(EMPTY_PACKET, &(stack->coils[rd_pos].state)));
		return res;
	}
	return 0;
}
