#include "can_tx_stack.h"
#include <cmsis_gcc.h>

#define FULL_STACK_CHECK 	0
#define ATOMIC_ENABLE		0

#if FULL_STACK_CHECK

static uint8_t is_stack_full(tx_stack *stack) {
	uint8_t cnt = 0;
	uint8_t i = 0;
	for(i=0;i<CAN_TX_STACK_LENGTH;i++) {
		if(stack->packet[i].state!=EMPTY_PACKET) cnt++;
	}
	if(cnt==CAN_TX_STACK_LENGTH) return 1;
	return 0;
}

#endif

#if ATOMIC_ENABLE
static uint16_t atomicWrPosIncrement(uint16_t * ptr)
{
	uint16_t oldValue, newValue;
//	uint8_t try_num = 0;
//	do
//	{
//		oldValue = __LDREXH(ptr);
//		newValue = oldValue + 1;
//		if(newValue>=CAN_TX_STACK_LENGTH) newValue = 0;
//		try_num++;
//		if(try_num>100) break;
//	}while(__STREXH(newValue, ptr));
//	if(oldValue>=CAN_TX_STACK_LENGTH) oldValue = 0;
//	return oldValue;
	newValue = oldValue = *ptr;
	newValue++;
	if(newValue>=CAN_TX_STACK_LENGTH) newValue = 0;
	if(oldValue>=CAN_TX_STACK_LENGTH) oldValue = 0;
	(*ptr)=newValue;
	return oldValue;
}
#endif

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
	uint16_t wr_pos = stack->write_position;
	//wr_pos = atomicWrPosIncrement(&(stack->wr_pos));
	if(wr_pos>=CAN_TX_STACK_LENGTH) {
		return;
	}
#if FULL_STACK_CHECK
	if(stack->packet[wr_pos].state!=EMPTY_PACKET && is_stack_full(stack)) {
		init_can_tx_stack(stack);
		stack->write_position = 1;
		wr_pos = 0;
	}
#endif
	if(packet->length > CAN_TX_DATA_SIZE) return;
	stack->packet[wr_pos].state = BUSY_PACKET;
	for(i=0;i<packet->length;++i) stack->packet[wr_pos].data[i] = packet->data[i];
	stack->packet[wr_pos].id = packet->id;
	stack->packet[wr_pos].priority = packet->priority;
	stack->packet[wr_pos].length = packet->length;
	stack->packet[wr_pos].state = READY_PACKET;
	wr_pos++;
	if(wr_pos>=CAN_TX_STACK_LENGTH) wr_pos=0;
	stack->write_position = wr_pos;
}

uint8_t get_tx_can_packet(tx_stack *stack, tx_stack_data *packet) {
	uint8_t i = 0;
	uint16_t rd_pos = stack->read_position;
	if(rd_pos>=CAN_TX_STACK_LENGTH) {
		return 0;
	}
	if(stack->packet[rd_pos].state==READY_PACKET) {
		packet->id = stack->packet[rd_pos].id;
		packet->length = stack->packet[rd_pos].length;
		if(packet->length > CAN_TX_DATA_SIZE) {
			stack->packet[rd_pos].state = EMPTY_PACKET;
			rd_pos++;
			if(rd_pos>=CAN_TX_STACK_LENGTH) rd_pos = 0;
			stack->read_position=rd_pos;
			return 0;
		}
		packet->priority = stack->packet[rd_pos].priority;
		for(i=0;i<packet->length;++i) packet->data[i] = stack->packet[rd_pos].data[i];
		stack->packet[rd_pos].state = EMPTY_PACKET;
		rd_pos++;
		if(rd_pos>=CAN_TX_STACK_LENGTH) rd_pos = 0;
		stack->read_position=rd_pos;
		return 1;
	}
	return 0;
}
