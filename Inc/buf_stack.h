/*
 * buf_stack.h
 *
 *  Created on: 20 рту. 2019 у.
 *      Author: User
 */

#ifndef BUF_STACK_H_
#define BUF_STACK_H_

#include <stdint.h>

#define STACK_SIZE		6

typedef struct {
	uint8_t* data;
	uint8_t state;
	uint16_t length;
}stack_coil;

typedef struct {
	stack_coil coils[STACK_SIZE];
	uint16_t wr_pos;
	uint16_t rd_pos;
	uint16_t max_coil_data_length;
} buf_stack;

void init_buf_stack(buf_stack *stack, uint8_t *data_ptr,uint16_t max_coil_length);
uint8_t add_data_to_stack(buf_stack *stack, uint8_t *data, uint16_t length);
uint16_t get_data_from_stack(buf_stack *stack, uint8_t *ptr);

#endif /* BUF_STACK_H_ */
