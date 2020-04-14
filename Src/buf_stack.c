/*
 * buf_stack.c
 *
 *  Created on: 20 авг. 2019 г.
 *      Author: User
 */

#include "buf_stack.h"
#include <cmsis_gcc.h>
#include "main.h"
#include <string.h>

#define EMPTY_PACKET	0
#define BUSY_PACKET		1
#define READY_PACKET	2

static uint8_t is_stack_full(buf_stack *stack) {
	uint8_t cnt = 0;
	uint8_t i = 0;
	for(i=0;i<STACK_SIZE;i++) {
		if(stack->state[i]!=EMPTY_PACKET) cnt++;
	}
	if(cnt==STACK_SIZE) return 1;
	return 0;
}

/*static uint16_t atomicWrPosIncrement(uint16_t * ptr)
{
	uint16_t oldValue, newValue;
//	uint8_t try_num = 0;
//	do
//	{
//		oldValue = __LDREXH(ptr);
//		newValue = oldValue + 1;
//		if(newValue>=STACK_SIZE) newValue = 0;
//		try_num++;
//		if(try_num>100) break;
//	}while(__STREXH(newValue, ptr));
//	if(oldValue>=STACK_SIZE) oldValue = 0;
	newValue = oldValue = *ptr;
	newValue++;
	if(newValue>=STACK_SIZE) newValue = 0;
	if(oldValue>=STACK_SIZE) oldValue = 0;
	(*ptr)=newValue;
	return oldValue;
}*/

void init_buf_stack(buf_stack *stack, uint8_t *data_ptr,uint16_t length_limit) {
	uint16_t i=0;
	//uint16_t offset = 0;
	stack->rd_pos = 0;
	stack->wr_pos = 0;
	stack->length_limit = length_limit;
	stack->ptr = data_ptr;
	for(i=0;i<STACK_SIZE;i++) {
		stack->state[i] = EMPTY_PACKET;
		stack->length[i] = 0;
	}
}

uint8_t add_data_to_stack(buf_stack *stack, uint8_t *data, uint16_t length) {
	uint16_t i = 0;
	uint16_t wr_pos = 0;
	uint16_t start_pos = 0;
	uint16_t end_pos = 0;
	uint16_t limit_pos = 0;

	wr_pos = stack->wr_pos;
	// проверка допустимости индекса записи
	if(wr_pos>=STACK_SIZE) {
		stack->wr_pos=0;
		return 0;
	}
	// проверка заполненности стэка
	if(stack->state[wr_pos]!=EMPTY_PACKET && is_stack_full(stack)) {
		stack->rd_pos = 0;
		stack->wr_pos = 1;
		for(i=0;i<STACK_SIZE;i++) {
			stack->state[i] = EMPTY_PACKET;
			stack->length[i] = 0;
		}
		wr_pos=0;
	}
	if(length>stack->length_limit) return 0; // проверка допустимой длины
	// проверка вместимости данных в стэк
	start_pos = wr_pos*stack->length_limit;
	end_pos = start_pos + length;
	limit_pos = ((uint16_t)STACK_SIZE) * (stack->length_limit);
	if((start_pos>=limit_pos) || (end_pos>limit_pos)) return 0;

	// запись данных
	stack->state[wr_pos] = BUSY_PACKET;
	stack->length[wr_pos] = length;
	memcpy(&(stack->ptr[start_pos]),&data[0],length);
	stack->state[wr_pos] = READY_PACKET;
	wr_pos++;
	if(wr_pos>=STACK_SIZE) wr_pos=0;
	stack->wr_pos = wr_pos;
	return 1;
}

uint16_t get_data_from_stack(buf_stack *stack, uint8_t *ptr) {
	uint16_t res = 0;
	uint16_t start_pos = 0;
	uint16_t end_pos = 0;
	uint16_t limit_pos = 0;
	uint16_t rd_pos = stack->rd_pos;
	if(rd_pos>=STACK_SIZE) {
		stack->rd_pos=0;
		return 0;
	}
	if(stack->state[rd_pos]==READY_PACKET) {
		start_pos = rd_pos*stack->length_limit;
		end_pos = start_pos + stack->length[rd_pos];
		limit_pos = ((uint16_t)STACK_SIZE) * (stack->length_limit);
		if((start_pos>=limit_pos) || (end_pos>limit_pos) || ((stack->length[rd_pos])>stack->length_limit)) {stack->length[rd_pos]=0;}
		stack->state[rd_pos] = BUSY_PACKET;
		res = stack->length[rd_pos];
		memcpy(&(ptr[0]),&(stack->ptr[start_pos]),res);
		stack->state[rd_pos] = EMPTY_PACKET;
		rd_pos++;
		if(rd_pos>=STACK_SIZE) rd_pos = 0;
		stack->rd_pos=rd_pos;
		return res;
	}
	return 0;
}
