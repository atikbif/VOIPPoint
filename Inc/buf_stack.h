/*
 * buf_stack.h
 *
 *  Created on: 20 рту. 2019 у.
 *      Author: User
 */

#ifndef BUF_STACK_H_
#define BUF_STACK_H_

#include <stdint.h>

#define STACK_SIZE		8

typedef struct  __attribute__((packed, aligned(4))){
	uint16_t state[STACK_SIZE];
	uint16_t length[STACK_SIZE];
	uint16_t wr_pos;
	uint16_t rd_pos;
	uint16_t length_limit;
	uint8_t* ptr;
} buf_stack;

void init_buf_stack(buf_stack *stack, uint8_t *data_ptr,uint16_t length_limit);
uint8_t add_data_to_stack(buf_stack *stack, uint8_t *data, uint16_t length);
uint16_t get_data_from_stack(buf_stack *stack, uint8_t *ptr);

#endif /* BUF_STACK_H_ */
