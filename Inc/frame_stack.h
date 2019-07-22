
#ifndef FRAME_STACK_H_
#define FRAME_STACK_H_

#include <stdint.h>

#define AUDIO_FRAME_COUNT			5
#define CAN_FRAME_COUNT				5

#define AUDIO_BUF_SIZE		160
#define CAN_BUF_SIZE		40

typedef struct {
	int16_t buf[AUDIO_BUF_SIZE];
	unsigned short length;
	unsigned char ready;
} big_frame;

typedef struct {
	uint8_t buf[CAN_BUF_SIZE];
	unsigned short length;
	unsigned char ready;
} small_frame;

void init_audio_frames();
void add_audio_frame(int16_t *ptr, unsigned short length);
void add_empty_audio_frame();
unsigned short get_audio_frame(int16_t *ptr);

void init_can_frames();
void add_can_frame(uint8_t *ptr, unsigned short length);
void add_empty_can_frame();
unsigned short get_can_frame(uint8_t *ptr);

#endif /* FRAME_STACK_H_ */
