/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dfsdm.h"
#include "dma.h"
#include "iwdg.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "arm_math.h"
#include "wave_example.h"
#include "frame_stack.h"
#include "opus.h"
#include "can_tx_stack.h"
#include "uart.h"
#include <stdlib.h>
#include <arm_math.h>
#include "can_cmd.h"
#include "eeprom.h"
#include "eeprom2.h"
#include "flash_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	FRAME_SIZE	160
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define FFT_SIZE 128//указываем размер FFT (быстрое преобразование Фурье)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PAGEError = 0;

volatile int32_t		RecBuf[2][FRAME_SIZE];
uint16_t 	conv[2][FRAME_SIZE];

volatile uint32_t	DmaMicrophoneHalfBuffCplt  = 0;
volatile uint32_t    DmaMicrophoneBuffCplt      = 0;

volatile uint32_t	DmaDacHalfBuffCplt  = 0;
volatile uint32_t    DmaDacBuffCplt      = 0;

extern DAC_HandleTypeDef hdac1;
extern CAN_HandleTypeDef hcan1;

OpusDecoder *dec;
OpusEncoder *enc;
static int error = 0;
char microphone_encoded_data[2][256];
int8_t encoded_length = 0;
int16_t	audio_stream[1024];

#define OPUS_PACKET_MAX_LENGTH	64

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox1=0;
static uint32_t              TxMailbox2=0;
static uint8_t               TxData[8];
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];
static uint8_t				 can_frame[OPUS_PACKET_MAX_LENGTH];
static uint8_t 				 can_priority_frame[OPUS_PACKET_MAX_LENGTH];
static uint32_t				 can_caught_id = 0;

uint8_t group_id = 0;	// номер группы/шлюза к которой принадлежит точка
uint16_t packet_tmr = 0;	// таймер активности звукового выхода
uint8_t can_pckt_length = 0;

unsigned char encoded_micr_ready_buf_num = 0;

uint32_t sin_offset = 0;
uint32_t call_offset = 0;
uint32_t call_offset2 = 0;

tx_stack can1_tx_stack;
tx_stack can2_tx_stack;
tx_stack can1_tx_stack_pr;	// буфера с повышенным приоритетом для передачи
tx_stack can2_tx_stack_pr;

uint8_t button1 = 0;
uint8_t button2 = 0;
// остояние концевика
uint8_t limit_switch = 0;

// лаг внешнего вызова
uint8_t call_flag = 0;
uint16_t call_tmr = 0;

// группа переменных для проверки исправности микрофона и динамиков

float32_t in_fft[FFT_SIZE] = {0};
float32_t out_fft[FFT_SIZE] = {0};
arm_rfft_fast_instance_f32 S;

float32_t base_2_5_kHz_level;		// уровень фонового сигнала микрофона на 2.5 кГц
float32_t test_2_5_kHz_level;		// уровень сигнала микрофона на 2.5 кГц во время проверки динамиков

volatile uint16_t test_2_5_kHz_tmr = 0;
uint8_t test_2_5_kHz_res = 0;
uint8_t test_2_5_kHz_state = 0;
uint8_t test_2_5_kHz_check_enable = 0;
uint8_t check_cmd = 0;


uint8_t pos_in_group = 1;			// номер точки в цепочке шлюза
extern uint8_t search_next_try;		// номер попытки поиска следующей точки

uint16_t discrete_state = 0;	// битовое состояние точки
// младший байт:
// бит 0 - результат проверки микрофона/динамиков
// бит 1  - была ли проверка
// бит 2 - состояние концевика
//старший байт:
// бит 0 - вход 1 замкнут
// бит 1 - вход 1 обрыв
// бит 2 - вход 1 кз
// бит 3 - вход 2 замкнут
// бит 4 - вход 2 обрыв
// бит 5 - вход 2 кз
// бит 6 - выход 1
// бит 7 - выход 2

// авария точки (если вход 1 разомкнут или по нему есть обрыв или кз)
uint8_t alarm_flag = 0;

// флаг и таймер для отправления данных компьютеру а не всем точкам при вызове от компьютера
uint8_t point_flag = 0;
uint16_t point_tmr = 0;


// переменные для ацп
uint16_t adc_data[2] = {0,0};	// непосредственно данные
uint32_t sum_adc[2] ={0,0};		// массив для усреднения
uint16_t adc_filter_tmr = 0;

// данные пересчитанные в десятые доли вольт
uint8_t prev_pow_data[2] = {0,0};
uint8_t pow_data[2] = {0,0};

uint64_t gain = 0; // текущий уровень громкости
// 0 - максимальная
// 1 - ослабление в 2 раза
// 2 - в 4 раза
// 3 - в 8 раз

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// проверка необходимо ли воспроизводить пришедший аудиопакет
uint8_t static check_id_priority(uint32_t packet_id) {
	id_field *input_id = (id_field*)(&packet_id);
	id_field *cur_id = (id_field*)(&can_caught_id);
	if(input_id->type==POINT_TO_PC) return 0; // точка компьютер игнорируем
	if(input_id->type==AUDIO_INFO) {
		if(cur_id->type!=AUDIO_INFO || ((cur_id->type==AUDIO_INFO) && (cur_id->group_addr==input_id->group_addr))) {
			*cur_id = *input_id;
			point_flag = 0;
			return 1;
		}
	}
	if(cur_id->type==AUDIO_INFO) return 0;
	if(input_id->type==PC_TO_ALL) { // компьютер ко всем
		*cur_id = *input_id;
		point_flag = 0;
		return 1;
	}
	if(input_id->type==PC_TO_POINT) { // компьютер точка
		if((input_id->group_addr == group_id) && (input_id->point_addr == pos_in_group)) { //совпадает адрес группы и адрес точки
			*cur_id = *input_id;
			point_tmr = 0;
			point_flag = 1;
			return 1;
		}
		return 0;
	}
	if(input_id->type==PC_TO_GROUP) { // компьютер группа
		if(input_id->group_addr == group_id) { //совпадает адрес группы
			*cur_id = *input_id;
			point_flag = 0;
			return 1;
		}
		return 0;
	}
	if(input_id->type==POINT_TO_ALL || input_id->type==POINT_CALL) { // точка все
		if(cur_id->type==UNKNOWN_TYPE) {	// пакеты не захвачены
			*cur_id = *input_id;
			point_flag = 0;
			return 1;
		}
		if(cur_id->type!=PC_TO_ALL && cur_id->type!=PC_TO_GROUP && cur_id->type!=PC_TO_POINT)  {
			// ранее захваченный источник
			if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
				*cur_id = *input_id;
				point_flag = 0;
				return 1;
			}
			// активный пакет не из родной группы, новый запрос из родной группы, перехватить
			if(cur_id->group_addr != group_id && input_id->group_addr == group_id) {
				*cur_id = *input_id;
				point_flag = 0;
				return 1;

			}
		}
		return 0;
	}
	return 0;
}

// обработка входящих кан пакетов (can_num (1-входящий, 2-исходящий))
static void check_can_rx(uint8_t can_num) {
	CAN_HandleTypeDef *hcan;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t cur_num = 0;
	uint8_t cnt = 0;
	tx_stack_data packet;
	id_field *p_id;
	if(can_num==1) hcan = &hcan1; else hcan = &hcan2;
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			p_id = (id_field*)(&(RxHeader.ExtId));
			if(p_id->cmd==AUDIO_PACKET) { // аудиопоток

				if(button1==0 && button2==0 && check_id_priority(RxHeader.ExtId)) {
					packet_tmr = 0;
					if(p_id->type==POINT_CALL) {
						call_flag = 1;
						call_tmr=0;
						//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
					}else {
						cur_num = p_id->param & 0x0F;
						cnt = (p_id->param & 0xFF)>> 4;
						if(cur_num) {
							if(cur_num==cnt) {
							  if(p_id->type==POINT_TO_ALL || p_id->type==PC_TO_ALL || p_id->type==PC_TO_POINT || p_id->type==PC_TO_GROUP || p_id->type==AUDIO_INFO) { // точка все
								  j = (cur_num-1)*8;
								  for(i=0;i<RxHeader.DLC;i++) {
									  if(j+i<OPUS_PACKET_MAX_LENGTH) can_priority_frame[j+i]=RxData[i];
								  }
								  can_pckt_length = (cnt-1)*8+RxHeader.DLC;
								  if(can_pckt_length>OPUS_PACKET_MAX_LENGTH) can_pckt_length = OPUS_PACKET_MAX_LENGTH;
								  for(i=0;i<can_pckt_length;i++) {
									  can_frame[i]=can_priority_frame[i];
								  }
								  add_can_frame(&can_frame[0],can_pckt_length);
							  }
							}else {
							  j = (cur_num-1)*8;
							  for(i=0;i<8;i++) { if(j+i<OPUS_PACKET_MAX_LENGTH) can_priority_frame[j+i]=RxData[i]; }
							}
						}
					}
				}
			}else if(p_id->cmd==LAST_POINT) {
				if(p_id->group_addr == group_id) {
					search_next_try = 0;	// ответ от следующей в цепочке точки
				}
			}else if(p_id->cmd==FIND_NEXT_POINT) {
				if(p_id->param==FIND_REQUEST) {	// запрос от предыдущей в цепочке точки
					pos_in_group = RxData[0] + 1;
					next_point(FIND_ANSWER);	// отправить ответ
				}else if(p_id->param==FIND_ANSWER) {
					search_next_try = 0;	// ответ от следующей в цепочке точки
				}
			}else if(p_id->cmd==SCAN_GROUP) {
				if(can_num==1) {	// транслировать запрос сканирования дальше
					check_cmd = 1;	// запустить проверку точки
				}
			}else if(p_id->cmd==SET_ALL_OUTS) {	// установить выход
				if(p_id->group_addr == group_id) {
					if(RxData[0]==1) {
						if(RxData[1]) discrete_state |= 0x4000;else discrete_state &= ~(0x4000);
					}else if(RxData[0]==2) {
						if(RxData[1]) discrete_state |= 0x8000;else discrete_state &= ~(0x8000);
					}
				}
			}else if(p_id->cmd==GET_POINTS_STATE) {
				group_id = p_id ->group_addr;
				send_point_state(1);
			}else if(p_id->cmd==GATE_STATE) {
				if(can_num==1) {
					group_id = p_id ->group_addr;
					if(RxData[1]&(1<<3)) {discrete_state |= 0x4000;}else{discrete_state &= ~(0x4000);}
					if(RxData[1]&(1<<4)) {discrete_state |= 0x8000;}else{discrete_state &= ~(0x8000);}
				}
			}else if(p_id->point_addr == pos_in_group && p_id->group_addr == group_id && p_id->cmd==BOOT) {
				if(p_id->type==BOOT_SWITCH) {
					HAL_FLASH_Unlock();
					EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
					EraseInitStruct.Banks       = GetBank((uint32_t)0x08080000);
					EraseInitStruct.Page        = GetPage((uint32_t)0x08080000);
					EraseInitStruct.NbPages     = 1;
					HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
					HAL_NVIC_SystemReset();
				}
			}
			else if(p_id->point_addr == pos_in_group && p_id->group_addr == group_id && p_id->cmd==POINT_CONFIG) {
				HAL_FLASH_Unlock();
				if(RxData[0]>4) RxData[0]=0;
				gain = RxData[0];
				write_var2(RxData[0]);
				send_point_state(1);
				send_point_state(1);
			}
			if(p_id->group_addr == group_id) {
				if(p_id->cmd==FIND_NEXT_POINT) return;
				if(limit_switch && p_id->cmd==LAST_POINT && can_num==2) return;
			}
			// ретрансляция пакетов
			if(!(p_id->point_addr == pos_in_group && p_id->group_addr == group_id)) {
				packet.id = RxHeader.ExtId;
				if(p_id->cmd==AUDIO_PACKET) {packet.priority = LOW_PACKET_PRIORITY;}
				else packet.priority = HIGH_PACKET_PRIORITY;
				packet.length = RxHeader.DLC;
				for(i=0;i<packet.length;++i) packet.data[i] = RxData[i];
				if(can_num==1)	{
					if(packet.priority == LOW_PACKET_PRIORITY) add_tx_can_packet(&can2_tx_stack,&packet);
					else add_tx_can_packet(&can2_tx_stack_pr,&packet);
				}
				else {
					if(packet.priority == LOW_PACKET_PRIORITY) add_tx_can_packet(&can1_tx_stack,&packet);
					else add_tx_can_packet(&can1_tx_stack_pr,&packet);
				}
			}
		}

	}
}

// отправка в сеть пакетов из tx буферов с двумя приоритетами
void can_write_from_stack() {
	tx_stack_data packet;
	uint8_t i = 0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=0) {
		if(get_tx_can_packet(&can1_tx_stack_pr,&packet)) {
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
			continue;
		}
		if(get_tx_can_packet(&can1_tx_stack,&packet)) {
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
		}else break;
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)!=0) {
		if(get_tx_can_packet(&can2_tx_stack_pr,&packet)) {
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox2);
			continue;
		}
		if(get_tx_can_packet(&can2_tx_stack,&packet)) {
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox2);
		}else break;
	}
}

static void initCANFilter() {
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
}

// разбивка кан буфера на пакеты
static void send_full_frame(uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);

	while(i) {
		pckt_cnt++;
		if(i<=8) {i=0;break;}
		i-=8;
	}
	i=1;
	while(cur_pckt<=pckt_cnt) {
		if(button2) p_id->type = POINT_CALL;
		else {
			if(point_flag==0) p_id->type = POINT_TO_ALL;
			else p_id->type = POINT_TO_PC;
		}

		p_id->point_addr = pos_in_group;
		p_id->group_addr = group_id;
		p_id->cmd = AUDIO_PACKET;
		p_id->param = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4);
		packet.priority = LOW_PACKET_PRIORITY;
		if(cur_pckt==pckt_cnt) { // last packet
			packet.length = len;
			for(i=0;i<len;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			cur_pckt++;
			len=0;
		}else {
			packet.length = 8;
			for(i=0;i<8;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			cur_pckt++;
			len-=8;
		}
	}
}

static void can_work() {
  if(encoded_micr_ready_buf_num) {
	  if(encoded_length>0) {
		  if(button1 || button2)
		  {
			  // отправка аудиоданных в сеть
			  send_full_frame(encoded_length,(unsigned char*)&microphone_encoded_data[encoded_micr_ready_buf_num-1]);
		  }
		  encoded_length = 0;
	  }
	  encoded_micr_ready_buf_num=0;
  }
}

// преобразование кан пакетов в звуковые
static void decode_work() {
	static uint8_t can_buf[40];
	int res = 0;
	uint16_t i = 0;
	unsigned short can_length = 0;
	if(button1) return;

	can_length = get_can_frame(can_buf);
	if((test_2_5_kHz_state==1) || (test_2_5_kHz_state==2)) {
		for(i=0;i<FRAME_SIZE;i++) {
			audio_stream[i] = sin_ex[sin_offset++]+0x8000;
			if(sin_offset>=sizeof(sin_ex)/2) sin_offset = 0;
			add_audio_frame(audio_stream,FRAME_SIZE);
		}
	}else if(button2 || call_flag) {
		for(i=0;i<FRAME_SIZE;i++) {
			audio_stream[i] = call_ex[call_offset2];
			if(call_flag==0) audio_stream[i]*=0.1;
			call_offset2++;
			if(call_offset2>=sizeof(call_ex)/2) call_offset2 = 0;
		}
		add_audio_frame(audio_stream,FRAME_SIZE);

	}else if(can_length) {
		res = opus_decode(dec,(unsigned char*)&can_buf[0],can_length,&audio_stream[0],1024,0);
		if(res!=FRAME_SIZE) {add_empty_audio_frame();}
		else {add_audio_frame(audio_stream,FRAME_SIZE);}
	}
}

// кодирование данных полученных от микрофона
static void encode_work() {
	uint16_t i = 0;
	static int16_t tmp_frame[FRAME_SIZE];
	if(test_2_5_kHz_state) return;
	if(DmaMicrophoneBuffCplt) {

	  if(button1) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i]  = SaturaLH((RecBuf[1][i] >>8), -32768, 32767);
		  }
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[1][0], 256);
		  if(encoded_length>0) {
			  encoded_micr_ready_buf_num = 2;
		  }

	  }else if(button2) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i] = call_ex[call_offset];
			  call_offset++;
			  if(call_offset>=sizeof(call_ex)/2) call_offset = 0;
		  }
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[1][0], 256);
		  if(encoded_length>0) {
			  encoded_micr_ready_buf_num = 2;
		  }
	  }

	  DmaMicrophoneBuffCplt = 0;
	}else if(DmaMicrophoneHalfBuffCplt) {

	  if(button1) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i]  = SaturaLH((RecBuf[0][i] >>8), -32768, 32767);
		  }
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[0][0], 256);
		  if(encoded_length>0) {
			  encoded_micr_ready_buf_num = 1;
		  }
	  }else if(button2) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i] = call_ex[call_offset];
			  call_offset++;
			  if(call_offset>=sizeof(call_ex)/2) call_offset = 0;
		  }
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[0][0], 256);
		  if(encoded_length>0) {
			  encoded_micr_ready_buf_num = 1;
		  }
	  }

	  DmaMicrophoneHalfBuffCplt = 0;
	}
}

// преобразование звука в сигнал для ЦАП
static void convert(uint8_t num) {
	static int16_t tmp_frame[FRAME_SIZE];
	uint16_t i = 0;
	if(get_audio_frame(tmp_frame)==0) {
		for(i=0;i<FRAME_SIZE;i++) conv[num][i] = 32768;
	}else {
		for(i=0;i<FRAME_SIZE;i++) {
			conv[num][i] = ((tmp_frame[i]>>gain) + 32768);
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint16_t i = 0;
	uint16_t led_cnt = 0;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  HAL_Delay(400);	// задержка на стабилизацию питания
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  init_eeprom();
  write_var(0);
  init_eeprom2();
  gain = read_var2();
  if(gain>3) {
	  gain=0;
	  write_var2(0);
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_DFSDM1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_6);
  LL_USART_EnableIT_RXNE(USART1);

  HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_RESET);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &adc_data,2);

  enc = opus_encoder_create(8000, 1, OPUS_APPLICATION_RESTRICTED_LOWDELAY, &error);
  opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(1));
  //opus_encoder_ctl(enc, OPUS_SET_BITRATE(8000));
  dec = opus_decoder_create(8000,1,&error);

  HAL_TIM_Base_Start(&htim6);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t*)&RecBuf[0][0], 160*2);

  init_can_frames();
  init_audio_frames();
  initCANFilter();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	Error_Handler();
  }

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t*)conv,FRAME_SIZE*2,DAC_ALIGN_12B_L);

  init_can_tx_stack(&can1_tx_stack);
  init_can_tx_stack(&can2_tx_stack);
  init_can_tx_stack(&can1_tx_stack_pr);
  init_can_tx_stack(&can2_tx_stack_pr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  arm_rfft_fast_init_f32(&S, FFT_SIZE);

  DmaMicrophoneBuffCplt=0;
  while(DmaMicrophoneBuffCplt==0);

  send_point_state(1);

  //MX_IWDG_Init();

  while (1)
  {
	  if((discrete_state & (1<<9)) || (discrete_state & (1<<10)) || ((discrete_state & (1<<8))==0)) alarm_flag=1;
	  else alarm_flag=0;
	  // проверка микрофона динамиков по запросу
	  if(check_cmd && (test_2_5_kHz_state==0)) {
		  test_2_5_kHz_state = 1;
		  test_2_5_kHz_tmr = 0;
		  test_2_5_kHz_res = 0;
		  test_2_5_kHz_check_enable = 1;
	  }

	  if(test_2_5_kHz_state==1) {
		  // определение фонового уровня
		  if(test_2_5_kHz_check_enable) {
			  for(i=0;i<FFT_SIZE;i++) in_fft[i] = RecBuf[0][i];
			  arm_rfft_fast_f32(&S, in_fft,out_fft,0);
			  arm_cmplx_mag_f32(out_fft,out_fft, FFT_SIZE/2);
			  base_2_5_kHz_level = out_fft[40];
		  }
		  test_2_5_kHz_state = 2;
	  }
	  if((test_2_5_kHz_state==2) && (test_2_5_kHz_tmr>=1000)) {
		  // определение уровня при генерации динамиками частоты
		  if(test_2_5_kHz_check_enable) {
			  for(i=0;i<FFT_SIZE;i++) in_fft[i] = RecBuf[01][i];
			  arm_rfft_fast_f32(&S, in_fft,out_fft,0);
			  arm_cmplx_mag_f32(out_fft,out_fft, FFT_SIZE/2);
			  test_2_5_kHz_level = out_fft[40];
			  if(test_2_5_kHz_level/base_2_5_kHz_level>=32) test_2_5_kHz_res = 1;
			  else test_2_5_kHz_res = 0;
			  test_2_5_kHz_check_enable = 0;
			  if(check_cmd) {	// был запрос на сканирование от шлюза
				  if(test_2_5_kHz_res) {
					  discrete_state |= 0x0001;
				  }else {
					  discrete_state &= ~(0x0001);
				  }
				  check_cmd = 0;
				  discrete_state |= 0x0002;	// была выполнена проверка
				  send_point_state(1);
			  }
		  }

		  test_2_5_kHz_state = 3;
	  }
	  if((test_2_5_kHz_state==3) && (test_2_5_kHz_tmr>=2000)) test_2_5_kHz_state = 0;

	  if(DmaDacBuffCplt) {
		  decode_work();
		  DmaDacBuffCplt = 0;
	  }else if(DmaDacHalfBuffCplt) {
		  decode_work();
		  DmaDacHalfBuffCplt=0;
	  }

	  encode_work();
	  can_work();
	  uart1_scan();
	  if(packet_tmr>=500) {
		  can_caught_id = 0;
		  if(button2 || test_2_5_kHz_state || call_flag) HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_SET);
		  else HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_RESET);
	  }else HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_SET);

	  sum_adc[0]+=adc_data[0];
	  sum_adc[1]+=adc_data[1];
	  adc_filter_tmr++;
	  if(adc_filter_tmr>=32) {
		  adc_filter_tmr=0;
		  pow_data[0] = (double)sum_adc[0]*33/1016/32+0.5;
		  pow_data[1] = (double)sum_adc[1]*33/1016/32+6.5;	// добавка 0.6 В для компенсации падения на стабилизаторе
		  sum_adc[0]=0;
		  sum_adc[1]=0;
	  }


	  if(button2) {
		  point_tmr = 0;
		  point_flag = 0;
	  }else {
		  if(call_flag==0) {
			  call_offset = 0;
			  call_offset2 = 0;
		  }
	  }
	  if(led_cnt<100) HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	  else HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	  led_cnt++;if(led_cnt>=65000) {led_cnt=0;}
	  if(call_tmr<65000) call_tmr++;else {call_flag=0;}
	  HAL_IWDG_Refresh(&hiwdg);
	  // спящий режим
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	  HAL_IWDG_Refresh(&hiwdg);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 43;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	check_can_rx(1);
	check_can_rx(2);
}



void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	DmaMicrophoneHalfBuffCplt = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	DmaMicrophoneBuffCplt = 1;
}




void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	DmaDacBuffCplt = 1;
	convert(1);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	DmaDacHalfBuffCplt = 1;
	convert(0);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
