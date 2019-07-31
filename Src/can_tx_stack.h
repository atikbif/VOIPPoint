/*
 * can_tx_stack.h
 *
 *  Created on: 30 èþë. 2019 ã.
 *      Author: User
 */

#ifndef CAN_TX_STACK_H_
#define CAN_TX_STACK_H_

#include <stdint.h>

#define CAN_TX_DATA_SIZE		8
#define CAN_TX_STACK_LENGTH		64

#define EMPTY_PACKET	0
#define BUSY_PACKET		1
#define READY_PACKET	2

#define LOW_PACKET_PRIORITY		1
#define HIGH_PACKET_PRIORITY	2

typedef struct {
	uint8_t data[CAN_TX_DATA_SIZE];
	uint32_t id;
	uint8_t length;
	uint8_t state;
	uint8_t priority;
} tx_stack_data;

typedef struct {
	tx_stack_data packet[CAN_TX_STACK_LENGTH];
	uint16_t read_position;
	uint16_t write_position;
} tx_stack;

void init_can_tx_stack(tx_stack *stack);
void add_tx_can_packet(tx_stack *stack,tx_stack_data *packet);
uint8_t get_tx_can_packet(tx_stack *stack, tx_stack_data *packet);

#endif /* CAN_TX_STACK_H_ */
