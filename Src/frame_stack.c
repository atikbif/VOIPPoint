#include "frame_stack.h"
#include "main.h"

big_frame audio_frames[AUDIO_FRAME_COUNT];
small_frame can_frames[CAN_FRAME_COUNT];

uint32_t add_cnt = 0;
uint32_t rem_cnt = 0;

uint8_t audio_ready = 0;
//uint8_t can_ready = 0;

unsigned char empty_can_buf[] = {0x08,0x0B,0xE4,0xB9,0x9C,0x34,0x04};

static unsigned char audio_wr_pos = 0;
static unsigned char can_wr_pos = 0;


static unsigned char audio_buzy = 0;
static unsigned char can_buzy = 0;

void init_audio_frames() {
	static unsigned short i = 0;
	static unsigned short j = 0;
	for(i=0;i<AUDIO_FRAME_COUNT;i++) {
		audio_frames[i].ready = 0;
		for(j=0;j<AUDIO_BUF_SIZE;j++) {
			audio_frames[i].buf[j] = 0;
		}
		audio_frames[i].length = AUDIO_BUF_SIZE;
	}
	audio_wr_pos = 0;
}

void init_can_frames() {
	static unsigned short i = 0;
	static unsigned short j = 0;
	for(i=0;i<CAN_FRAME_COUNT;i++) {
		can_frames[i].ready = 0;
		for(j=0;j<sizeof(empty_can_buf);j++) {
			can_frames[i].buf[j] = empty_can_buf[j];
		}
		can_frames[i].length = sizeof(empty_can_buf);
	}
	can_wr_pos = 0;
}


void add_audio_frame(int16_t *ptr, unsigned short length) {
	static unsigned short i = 0;
	if(length==0) return;

	add_cnt++;

	audio_buzy = 1;
	if(audio_wr_pos>=AUDIO_FRAME_COUNT) {
		init_audio_frames();
		audio_wr_pos = 0;
		//return;
	}
	for(i=0;i<length;i++) {
		audio_frames[audio_wr_pos].buf[i] = ptr[i];
	}
	audio_frames[audio_wr_pos].length = length;
	audio_frames[audio_wr_pos].ready=1;
	audio_wr_pos++;if(audio_wr_pos>=2) audio_ready = 1;
	audio_buzy = 0;
	//HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
}

void add_can_frame(uint8_t *ptr, unsigned short length) {
	static unsigned short i = 0;
	if(length==0) return;
	can_buzy = 1;
	if(can_wr_pos>=CAN_FRAME_COUNT) {
		init_can_frames();
		can_wr_pos = 0;
		//return;
	}
	for(i=0;i<length;i++) {
		can_frames[can_wr_pos].buf[i] = ptr[i];
	}
	can_frames[can_wr_pos].length = length;
	can_frames[can_wr_pos].ready=1;
	can_wr_pos++;
	can_buzy = 0;
}

void add_empty_can_frame() {
	static unsigned short i = 0;
	can_buzy = 1;
	if(can_wr_pos>=CAN_FRAME_COUNT) {
		init_can_frames();
		can_wr_pos = 0;
		//return;
	}
	for(i=0;i<sizeof(empty_can_buf);i++) {
		can_frames[can_wr_pos].buf[i] = empty_can_buf[i];
	}
	can_frames[can_wr_pos].length = sizeof(empty_can_buf);
	can_frames[can_wr_pos].ready=1;
	can_wr_pos++;
	can_buzy = 0;
}

void add_empty_audio_frame() {
	static unsigned short i = 0;

	add_cnt++;

	audio_buzy = 1;
	if(audio_wr_pos>=AUDIO_FRAME_COUNT) {
		init_audio_frames();
		audio_wr_pos = 0;
		//return;
	}
	for(i=0;i<AUDIO_BUF_SIZE;i++) {
		audio_frames[audio_wr_pos].buf[i] = 0;
	}
	audio_frames[audio_wr_pos].length = AUDIO_BUF_SIZE;
	audio_frames[audio_wr_pos].ready=1;
	audio_wr_pos++;if(audio_wr_pos>=2) audio_ready = 1;
	audio_buzy = 0;
}

unsigned short get_audio_frame(int16_t *ptr) {
	static unsigned short i = 0;
	static unsigned short j = 0;
	unsigned short length = 0;
	if(audio_ready==0) return 0;
	rem_cnt++;
	if(/*audio_wr_pos<AUDIO_FRAME_COUNT && */audio_frames[0].ready) {
		for(i=0;i<audio_frames[0].length;i++) ptr[i] = audio_frames[0].buf[i];
		length = audio_frames[0].length;
		for(i=0;i<AUDIO_FRAME_COUNT-1;i++) {
			audio_frames[i].ready = audio_frames[i+1].ready;
			audio_frames[i].length = audio_frames[i+1].length;
			for(j=0;j<AUDIO_BUF_SIZE;j++) {
				audio_frames[i].buf[j] = (uint16_t)audio_frames[i+1].buf[j];//+(int16_t)32768;
			}
		}
		for(i=0;i<AUDIO_BUF_SIZE;i++) audio_frames[AUDIO_FRAME_COUNT-1].buf[i] = 0;
		audio_frames[AUDIO_FRAME_COUNT-1].length = AUDIO_BUF_SIZE;
		if(audio_wr_pos) audio_wr_pos--;
	}
	//HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	return length;
}

unsigned short get_can_frame(uint8_t *ptr) {
	static unsigned short i = 0;
	static unsigned short j = 0;
	unsigned short length = 0;

	if(/*can_wr_pos<CAN_FRAME_COUNT && */can_frames[0].ready) {
		for(i=0;i<can_frames[0].length;i++) ptr[i] = can_frames[0].buf[i];
		length = can_frames[0].length;
		for(i=0;i<CAN_FRAME_COUNT-1;i++) {
			can_frames[i].ready = can_frames[i+1].ready;
			can_frames[i].length = can_frames[i+1].length;
			for(j=0;j<CAN_BUF_SIZE;j++) {
				can_frames[i].buf[j] = can_frames[i+1].buf[j];
			}
		}
		for(i=0;i<sizeof(empty_can_buf);i++) can_frames[CAN_FRAME_COUNT-1].buf[i] = empty_can_buf[i];
		can_frames[CAN_FRAME_COUNT-1].length = sizeof(empty_can_buf);
		if(can_wr_pos) can_wr_pos--;
	}
	return length;
}
