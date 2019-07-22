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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fit_processing.h"
#include "arm_math.h"
#include "wave_example.h"
#include "frame_stack.h"
#include "opus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	FRAME_SIZE	160

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float32_t inp_buf[FRAME_SIZE*6];
float32_t out_buf[FRAME_SIZE*6];

int32_t		RecBuf[2][FRAME_SIZE];
uint16_t 	conv[2][FRAME_SIZE*6];

uint32_t	DmaMicrophoneHalfBuffCplt  = 0;
uint32_t    DmaMicrophoneBuffCplt      = 0;

uint32_t	DmaDacHalfBuffCplt  = 0;
uint32_t    DmaDacBuffCplt      = 0;

uint32_t ex_offset = 0;

extern DAC_HandleTypeDef hdac1;
extern CAN_HandleTypeDef hcan1;

OpusDecoder *dec;
OpusEncoder *enc;
static int error = 0;
char microphone_encoded_data[2][256];
//unsigned char encoded_micr_ready_buf_num = 0;
int8_t encoded_length = 0;
int16_t	audio_stream[1024];

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox=0;
static uint8_t               TxData[8];
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];
static uint8_t				 can_frame[40];
static uint8_t				 can_frame_id[256][40];
uint16_t can_tmr = 0;
unsigned short device_id = 0x01;
unsigned short gate_id = 0xFE;
unsigned short point_to_point_tmr = 0x00;
unsigned char to_id = 0xFF;
uint8_t can_pckt_length = 0;
uint16_t p_cnt = 0;

unsigned char encoded_micr_ready_buf_num = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
}

static void send_full_frame(uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;
	uint16_t wait_delay = 0;

	while(i) {
		pckt_cnt++;
		if(i<=6) {i=0;break;}
		else i-=7;
		if(i==0) pckt_cnt++;
	}
	i=1;

	while(cur_pckt<=pckt_cnt) {
		TxHeader.StdId = device_id;
		TxHeader.ExtId = 0;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
		if(cur_pckt==pckt_cnt) { // last packet
			TxHeader.DLC = 2+len;
			TxData[0] = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4); // current packet number and packets cnt
			TxData[1] = to_id;
			for(i=0;i<len;i++) TxData[i+2] = ptr[(cur_pckt-1)*7+i];
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {wait_delay++;if(wait_delay>=1000) {NVIC_SystemReset();return;} HAL_Delay(1);}
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			cur_pckt++;
			len=0;
		}else {
			TxHeader.DLC = 0x08;
			TxData[0] = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4); // current packet number and packets cnt
			for(i=0;i<7;i++) TxData[i+1] = ptr[(cur_pckt-1)*7+i];
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {wait_delay++;if(wait_delay>=1000) {NVIC_SystemReset();return;}HAL_Delay(1);}
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			cur_pckt++;
			len-=7;
		}
	}
}

static void can_work() {
  if(encoded_micr_ready_buf_num) {
	  if(encoded_length>0) {
		  send_full_frame(encoded_length,(unsigned char*)&microphone_encoded_data[encoded_micr_ready_buf_num-1]);
		  encoded_length = 0;
	  }
	  encoded_micr_ready_buf_num=0;
  }
  if(point_to_point_tmr) point_to_point_tmr--;
  else to_id = 0xFF; // send data to all
}

static void convert(uint8_t num) {
	static int16_t tmp_frame[FRAME_SIZE];
	//static uint32_t wav_offset=0;
	uint16_t i = 0;
	float32_t k = 0.04;
	if(get_audio_frame(tmp_frame)==0) {
		//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		for(i=0;i<FRAME_SIZE;i++) tmp_frame[i] = 0;
	}

	for(i=0;i<FRAME_SIZE;i++) {
		//tmp_frame[i] = wav_ex[wav_offset]+0x8000;
		//wav_offset++;
		//if(wav_offset>=sizeof(wav_ex)/2) wav_offset = 0;
		inp_buf[i*6] = tmp_frame[i];
		inp_buf[i*6+1] = tmp_frame[i];
		inp_buf[i*6+2] = tmp_frame[i];
		inp_buf[i*6+3] = tmp_frame[i];
		inp_buf[i*6+4] = tmp_frame[i];
		inp_buf[i*6+5] = tmp_frame[i];
		//inp_buf[i] = (uint16_t)(SaturaLH((RecBuf[num][i] >>8), -32768, 32767)) + (int16_t)32768;
	}
	FIR_PROCESSING_F32Process(inp_buf,out_buf);
	for(i=0;i<FRAME_SIZE*6;i++) {
		//conv[num][i] = inp_buf[i]*k+(float32_t)32768;
		conv[num][i] = (out_buf[i]+32768.5)*k;
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

  static int16_t tmp_frame[FRAME_SIZE];
  uint32_t wav_offset = 0;
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
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_SET);

  FIR_Init();

  init_can_frames();
  initCANFilter();
  HAL_CAN_Start(&hcan1);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  	Error_Handler();
  }

  enc = opus_encoder_create(8000, 1, OPUS_APPLICATION_AUDIO , &error);
  opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(1));
  opus_encoder_ctl(enc, OPUS_SET_BITRATE(8000));
  dec = opus_decoder_create(8000,1,&error);

  HAL_TIM_Base_Start(&htim6);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t*)&RecBuf[0][0], 160*2);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t*)conv,FRAME_SIZE*6*2,DAC_ALIGN_12B_L);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(DmaDacBuffCplt) {
		  convert(1);
		  DmaDacBuffCplt = 0;
	  }else if(DmaDacHalfBuffCplt) {
		  convert(0);
		  DmaDacHalfBuffCplt=0;
	  }

	  if(DmaMicrophoneBuffCplt) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i] = wav_ex[wav_offset]+0x8000;
			  wav_offset++;
			  if(wav_offset>=sizeof(wav_ex)/2) wav_offset = 0;
		  }
		  //add_audio_frame(&tmp_frame[0],FRAME_SIZE);
		  //add_empty_audio_frame();
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[0][0], 256);
		  if(encoded_length) {
			  int res = opus_decode(dec,(unsigned char*)&microphone_encoded_data[0][0],encoded_length,&audio_stream[0],1024,0);
			  if(res==FRAME_SIZE) {
				  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				  add_audio_frame(&audio_stream[0],FRAME_SIZE);
			  }
		  }

		  DmaMicrophoneBuffCplt = 0;
	  }else if(DmaMicrophoneHalfBuffCplt) {
		  for(i=0;i<FRAME_SIZE;i++) {
			  tmp_frame[i] = wav_ex[wav_offset]+0x8000;
			  wav_offset++;
			  if(wav_offset>=sizeof(wav_ex)/2) wav_offset = 0;
		  }
		  //add_audio_frame(&tmp_frame[0],FRAME_SIZE);
		  //add_empty_audio_frame();
		  encoded_length = opus_encode(enc, (opus_int16*)tmp_frame, FRAME_SIZE,(unsigned char*) &microphone_encoded_data[0][0], 256);
		  if(encoded_length) {
			  int res = opus_decode(dec,(unsigned char*)&microphone_encoded_data[0][0],encoded_length,&audio_stream[0],1024,0);
			  if(res==FRAME_SIZE) {
				  add_audio_frame(&audio_stream[0],FRAME_SIZE);
			  }
		  }
		  DmaMicrophoneHalfBuffCplt = 0;
	  }
	  can_work();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_DFSDM1;
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
	static uint8_t i = 0;
	static uint8_t cur_num = 0;
	static uint8_t cnt = 0;
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {
	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		  cur_num = RxData[0] & 0x0F;
		  cnt = RxData[0] >> 4;
		  if(cur_num==cnt) {
			  //HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			  if(RxHeader.StdId != device_id) {
				  if(RxData[1]==device_id) {
					  to_id = RxHeader.StdId;
					  point_to_point_tmr = 3000;
					  for(i=0;i<RxHeader.DLC-2;i++) can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+2];
					  can_pckt_length = (cnt-1)*7+RxHeader.DLC-2;
					  for(i=0;i<can_pckt_length;i++) can_frame[i]=can_frame_id[RxHeader.StdId][i];
					  add_can_frame(&can_frame[0],can_pckt_length);
					  //frame_ready=1;
				  }else if(RxData[1]==0xFF) {
					  for(i=0;i<RxHeader.DLC-2;i++) can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+2];
					  can_pckt_length = (cnt-1)*7+RxHeader.DLC-2;
					  for(i=0;i<can_pckt_length;i++) can_frame[i]=can_frame_id[RxHeader.StdId][i];
					  to_id = 0xFF;
					  p_cnt++;
					  add_can_frame(&can_frame[0],can_pckt_length);

					  //frame_ready=1;
				  }
			  }

		  }else {
			  for(i=0;i<7;i++) can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+1];
		  }
	  }
  }
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
	//convert(1);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	DmaDacHalfBuffCplt = 1;
	//convert(0);
	//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
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
