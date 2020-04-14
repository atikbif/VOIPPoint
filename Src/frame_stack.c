#include "frame_stack.h"
#include "main.h"
#include "buf_stack.h"

// буфера для аудиопакетов и для данных зашифрованных опусом, полученных из кан

static uint8_t empty_can_buf[] = {0x08,0x0B,0xE4,0xB9,0x9C,0x34,0x04}; // закодированный в опус нулевой сигнал (тишина)
static int16_t empty_audio_buf[AUDIO_BUF_SIZE];

static volatile uint8_t audio_stack_can[(STACK_SIZE+1)*CAN_BUF_SIZE]  __attribute__((aligned(4)));
static volatile uint8_t audio_stack_stream[(STACK_SIZE+1)*AUDIO_BUF_SIZE*2]  __attribute__((aligned(4)));
static buf_stack can_stack;
static buf_stack audio_stack;

void init_audio_frames() {
	uint16_t i = 0;
	for(i=0;i<AUDIO_BUF_SIZE;i++) empty_audio_buf[i] = 0;
	init_buf_stack(&audio_stack,(uint8_t*)audio_stack_stream,(uint16_t)(AUDIO_BUF_SIZE)*2);
}

void init_can_frames() {
	init_buf_stack(&can_stack,(uint8_t*)audio_stack_can,CAN_BUF_SIZE);
}


void add_audio_frame(int16_t *ptr, unsigned short length) {
	if(length>AUDIO_BUF_SIZE) return;
	add_data_to_stack(&audio_stack,(uint8_t*)ptr,length*2);
}

void add_can_frame(uint8_t *ptr, unsigned short length) {
	if(length>CAN_BUF_SIZE) return;
	add_data_to_stack(&can_stack,ptr,length);
}

void add_empty_can_frame() {
	add_data_to_stack(&can_stack,empty_can_buf,sizeof(empty_can_buf));
}

void add_empty_audio_frame() {
	add_data_to_stack(&audio_stack,(uint8_t*)empty_audio_buf,sizeof(empty_audio_buf));
}

uint16_t get_audio_frame(int16_t *ptr) {
	uint16_t res = get_data_from_stack(&audio_stack,(uint8_t*)ptr);
	return res/2;
}

uint16_t get_can_frame(uint8_t *ptr) {
	uint16_t res = get_data_from_stack(&can_stack,ptr);
	return res;
}
