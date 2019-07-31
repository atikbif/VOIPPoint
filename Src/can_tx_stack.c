#include "can_tx_stack.h"
#include <cmsis_gcc.h>


static uint16_t atomicWrPosIncrement(uint16_t * ptr)
{
	uint16_t oldValue, newValue;
	do
	{
		oldValue = __LDREXH(ptr);
		newValue = oldValue + 1;
		if(newValue>=CAN_TX_STACK_LENGTH) newValue = 0;
	}while(__STREXH(newValue, ptr));
	return oldValue;
}

void init_can_tx_stack(tx_stack *stack) {
	uint8_t i = 0;
	stack->read_position = 0;
	stack->write_position = 0;
	for(i=0;i<CAN_TX_STACK_LENGTH;i++) {
		stack->packet[i].length = 0;
		stack->packet[i].state = EMPTY_PACKET;
	}
}

void add_tx_can_packet(tx_stack *stack,tx_stack_data *packet) {
	uint8_t i = 0;
	uint16_t wr_pos = atomicWrPosIncrement(&(stack->write_position));
	stack->packet[wr_pos].state = BUSY_PACKET;
	for(i=0;i<packet->length;++i) stack->packet[wr_pos].data[i] = packet->data[i];
	stack->packet[wr_pos].id = packet->id;
	stack->packet[wr_pos].priority = packet->priority;
	stack->packet[wr_pos].length = packet->length;
	stack->packet[wr_pos].state = READY_PACKET;
}

uint8_t get_tx_can_packet(tx_stack *stack, tx_stack_data *packet) {
	uint8_t i = 0;
	if(stack->packet[stack->read_position].state==READY_PACKET) {
		packet->id = stack->packet[stack->read_position].id;
		packet->length = stack->packet[stack->read_position].length;
		packet->priority = stack->packet[stack->read_position].priority;
		for(i=0;i<packet->length;++i) packet->data[i] = stack->packet[stack->read_position].data[i];
		stack->packet[stack->read_position].state = EMPTY_PACKET;
		stack->read_position++;
		if(stack->read_position>=CAN_TX_STACK_LENGTH) stack->read_position = 0;
		return 1;
	}
	return 0;
}
