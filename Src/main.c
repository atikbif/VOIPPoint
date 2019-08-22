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
#include "can.h"
#include "dac.h"
#include "dfsdm.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fit_processing.h"
#include "arm_math.h"
#include "wave_example.h"
#include "frame_stack.h"
#include "opus.h"
#include "can_tx_stack.h"
#include "uart.h"
#include <stdlib.h>
#include <arm_math.h> //подключаем библиотеку

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	FRAME_SIZE	160
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define FFT_SIZE 128//указываем размер FFT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int32_t		RecBuf[2][FRAME_SIZE];
uint16_t 	conv[2][FRAME_SIZE];

volatile uint32_t	DmaMicrophoneHalfBuffCplt  = 0;
volatile uint32_t    DmaMicrophoneBuffCplt      = 0;

volatile uint32_t	DmaDacHalfBuffCplt  = 0;
volatile uint32_t    DmaDacBuffCplt      = 0;

uint32_t ex_offset = 0;

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
//static uint8_t				 can_frame_id[256][40];


#define		UNKNOWN_TYPE	0
#define		PC_TO_ALL		1
#define		PC_TO_GROUP		2
#define		PC_TO_POINT		3
#define		POINT_TO_ALL	4
#define		POINT_TO_PC		5

static uint8_t 				 can_priority_frame[OPUS_PACKET_MAX_LENGTH];
static uint32_t				 can_caught_id = 0;
uint16_t can_tmr = 0;
uint8_t group_id = 1;
unsigned short device_id = 1;
unsigned short gate_id = 0xFE;
unsigned short point_to_point_tmr = 0x00;
uint16_t packet_tmr = 0;
unsigned char to_id = 0xFF;
uint8_t can_pckt_length = 0;
uint16_t p_cnt = 0;

unsigned char encoded_micr_ready_buf_num = 0;

uint32_t wav_offset = 0;
uint32_t wav_offset2 = 0;

uint32_t sin_offset = 0;

tx_stack can1_tx_stack;
tx_stack can2_tx_stack;

uint8_t button1 = 0;
uint8_t button2 = 0;

float32_t in_fft[FFT_SIZE] = {0};
float32_t out_fft[FFT_SIZE] = {0};
arm_rfft_fast_instance_f32 S;
//float32_t maxvalue;
//uint32_t maxindex;

float32_t base_2_5_kHz_level;
float32_t test_2_5_kHz_level;

volatile uint16_t test_2_5_kHz_tmr = 0;
uint8_t test_2_5_kHz_res = 0;
uint8_t test_2_5_kHz_state = 0;
uint8_t test_2_5_kHz_check_enable = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
 uint32_t param: 8;
 uint32_t cmd: 4;
 uint32_t group_addr: 7;
 uint32_t point_addr: 7;
 uint32_t type: 3;
 uint32_t unused_bits : 3;
} id_field;



uint8_t static check_id_priority(uint32_t packet_id) {
	id_field *input_id = (id_field*)(&packet_id);
	id_field *cur_id = (id_field*)(&can_caught_id);
	if(input_id->type==POINT_TO_PC) return 0; // точка компьютер игнорируем
	if(input_id->type==PC_TO_ALL) { // компьютер ко всем
		*cur_id = *input_id;
		return 1;
	}
	if(input_id->type==PC_TO_POINT) { // компьютер точка
		if((input_id->group_addr == group_id) && (input_id->point_addr == device_id)) { //совпадает адрес группы и адрес точки
			*cur_id = *input_id;
			return 1;
		}
		return 0;
	}
	if(input_id->type==PC_TO_GROUP) { // компьютер группа
		if(input_id->group_addr == group_id) { //совпадает адрес группы
			*cur_id = *input_id;
			return 1;
		}
		return 0;
	}
	if(input_id->type==POINT_TO_ALL) { // точка все
		if(cur_id->type==UNKNOWN_TYPE) {	// пакеты не захвачены
			*cur_id = *input_id;
			return 1;
		}
		if(cur_id->type!=PC_TO_ALL && cur_id->type!=PC_TO_GROUP && cur_id->type!=PC_TO_POINT)  {
			// ранее захваченный источник
			if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
				*cur_id = *input_id;
				return 1;
			}
			// активный пакет не из родной группы, новый запрос из родной группы, перехватить
			if(cur_id->group_addr != group_id && input_id->group_addr == group_id) {
				*cur_id = *input_id;
				return 1;

			}
		}
		return 0;
	}
	return 0;
}

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
			if(p_id->cmd==1) { // аудиопоток
				if(check_id_priority(RxHeader.ExtId)) {
					packet_tmr = 0;
					cur_num = p_id->param & 0x0F;
					cnt = (p_id->param & 0xFF)>> 4;
					if(cur_num) {
						if(cur_num==cnt) {
						  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
						  if(p_id->type==POINT_TO_ALL || p_id->type==PC_TO_ALL) { // точка все
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
		}
		if(!(p_id->point_addr == device_id && p_id->group_addr == group_id)) {
			packet.id = RxHeader.ExtId;
			packet.priority = LOW_PACKET_PRIORITY;
			packet.length = RxHeader.DLC;
			for(i=0;i<packet.length;++i) packet.data[i] = RxData[i];
			if(can_num==1)	add_tx_can_packet(&can2_tx_stack,&packet);
			else add_tx_can_packet(&can1_tx_stack,&packet);
		}
	}
}

void can_write_from_stack() {
	tx_stack_data packet;
	uint8_t i = 0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=0) {

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
		p_id->unused_bits = 0;
		p_id->type = POINT_TO_ALL;
		p_id->point_addr = device_id;
		p_id->group_addr = group_id;
		p_id->cmd = 1;
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
		  if(button1)
		  {
			  send_full_frame(encoded_length,(unsigned char*)&microphone_encoded_data[encoded_micr_ready_buf_num-1]);
		  }
		  encoded_length = 0;
	  }
	  encoded_micr_ready_buf_num=0;
  }
  //if(point_to_point_tmr) point_to_point_tmr--;
  //else to_id = 0xFF; // send data to all
}

static void decode_work() {
	static uint8_t can_buf[40];
	int res = 0;
	uint16_t i = 0;
	unsigned short can_length = 0;
	if(button1) return;

	can_tmr++;

	//HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	can_length = get_can_frame(can_buf);
	//if(button2 || start_test_2_5_kHz) {
	if((test_2_5_kHz_state==1) || (test_2_5_kHz_state==2)) {
		can_tmr = 0;
		for(i=0;i<FRAME_SIZE;i++) {
			audio_stream[i] = sin_ex[sin_offset++]+0x8000;
			if(sin_offset>=sizeof(sin_ex)/2) sin_offset = 0;
			add_audio_frame(audio_stream,FRAME_SIZE);
		}
	}
	else if(can_length) {
		can_tmr = 0;
		res = opus_decode(dec,(unsigned char*)&can_buf[0],can_length,&audio_stream[0],1024,0);
		if(res!=FRAME_SIZE) {add_empty_audio_frame();}
		else {add_audio_frame(audio_stream,FRAME_SIZE);}
	}
	// no audio stream
	if(can_tmr>=30) {
		//add_empty_audio_frame();can_tmr = 0;
	}
	//HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
}

static void encode_work() {
	uint16_t i = 0;
	static int16_t tmp_frame[FRAME_SIZE];
	if(DmaMicrophoneBuffCplt) {

	  if(button1) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i]  = SaturaLH((RecBuf[1][i] >>8), -32768, 32767);
			  //tmp_frame[i] = wav_ex[wav_offset]+0x8000;
			  //wav_offset++;
			  //if(wav_offset>=sizeof(wav_ex)/2) wav_offset = 0;
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
			  //tmp_frame[i] = wav_ex[wav_offset]+0x8000;
			  //wav_offset++;
			  //if(wav_offset>=sizeof(wav_ex)/2) wav_offset = 0;
		  }
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[0][0], 256);
		  if(encoded_length>0) {
			  encoded_micr_ready_buf_num = 1;
		  }
	  }
	  DmaMicrophoneHalfBuffCplt = 0;
	}
}

static void convert(uint8_t num) {
	static int16_t tmp_frame[FRAME_SIZE];
	uint16_t i = 0;
	if(get_audio_frame(tmp_frame)==0) {
		for(i=0;i<FRAME_SIZE;i++) conv[num][i] = 32768;
	}else {
		//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		for(i=0;i<FRAME_SIZE;i++) {
			conv[num][i] = tmp_frame[i] + 32768;
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

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

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
  /* USER CODE BEGIN 2 */

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_6);
  LL_USART_EnableIT_RXNE(USART1);

  HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_SET);

  //FIR_Init();

  enc = opus_encoder_create(8000, 1, OPUS_APPLICATION_VOIP , &error);
  opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(2));
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  arm_rfft_fast_init_f32(&S, FFT_SIZE);

  DmaMicrophoneBuffCplt=0;
  while(DmaMicrophoneBuffCplt==0);

  while (1)
  {
	  if(button2 && (test_2_5_kHz_state==0)) {
		  test_2_5_kHz_state = 1;
		  test_2_5_kHz_tmr = 0;
		  test_2_5_kHz_res = 0;
		  //test_2_5_kHz_check_enable = 1;
	  }

	  if(test_2_5_kHz_state==1) {
		  if(test_2_5_kHz_check_enable) {
			  for(i=0;i<FFT_SIZE;i++) in_fft[i] = RecBuf[0][i];
			  arm_rfft_fast_f32(&S, in_fft,out_fft,0);
			  arm_cmplx_mag_f32(out_fft,out_fft, FFT_SIZE/2);
			  base_2_5_kHz_level = out_fft[40];
		  }
		  test_2_5_kHz_state = 2;
	  }
	  if((test_2_5_kHz_state==2) && (test_2_5_kHz_tmr>=1000)) {
		  if(test_2_5_kHz_check_enable) {
			  for(i=0;i<FFT_SIZE;i++) in_fft[i] = RecBuf[01][i];
			  arm_rfft_fast_f32(&S, in_fft,out_fft,0);
			  arm_cmplx_mag_f32(out_fft,out_fft, FFT_SIZE/2);
			  test_2_5_kHz_level = out_fft[40];
			  if(test_2_5_kHz_level/base_2_5_kHz_level>=32) test_2_5_kHz_res = 1;
			  //if(test_2_5_kHz_res) HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			  else test_2_5_kHz_res = 0;
			  test_2_5_kHz_check_enable = 0;
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
		  packet_tmr=0;
		  can_caught_id = 0;
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 43;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
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
