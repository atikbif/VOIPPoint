
#ifndef FRAME_STACK_H_
#define FRAME_STACK_H_

#include <stdint.h>

#define AUDIO_FRAME_COUNT			6
#define CAN_FRAME_COUNT				6

#define AUDIO_BUF_SIZE		160
#define CAN_BUF_SIZE		64

// буфера для хранения CAN и аудиопакетов на базе buf_stack

void init_audio_frames();
void add_audio_frame(int16_t *ptr, uint16_t length);
void add_empty_audio_frame();
uint16_t get_audio_frame(int16_t *ptr);

void init_can_frames();
void add_can_frame(uint8_t *ptr, uint16_t length);
void add_empty_can_frame();
unsigned short get_can_frame(uint8_t *ptr);

#endif /* FRAME_STACK_H_ */
