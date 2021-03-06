/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "can_cmd.h"
#include "rng.h"
#include "din.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern void can_write_from_stack();	// �������� � ��� ���� ������� ����� ������������ � �������

extern void convert(uint8_t num); // �������������� ����� � ������ ��� ��� � ������ ������������ ����������
// �������� ����� ������ �� ����� ������������

extern volatile uint32_t	DmaDacHalfBuffCplt;
extern volatile uint32_t    DmaDacBuffCplt;

extern uint8_t rx1_buf[UART_BUF_SISE];
extern uint8_t tx1_buf[UART_BUF_SISE];
extern uint16_t rx1_cnt;					// ������� �������� �� UART ����
extern uint16_t rx1_tmr;					// ������ ������ ��� ����������� ����� ������ �������� �� UART ������

// ������ ����������� ��������� ����������� ��� ���������������
// ��� ����������� ������������ � 0, ����� ��������� ������ ������������
extern uint16_t packet_tmr;
extern volatile uint16_t test_2_5_kHz_tmr;	// ��������������� ������ ��� �������� ���������
extern uint8_t test_2_5_kHz_state;			// ���� �������� ���������

extern uint16_t discrete_state;				// ������� ��������� �����
uint16_t prev_discr_state = 0;				// ������� ��������� ��� ������������ ���������

uint16_t search_next_tmr = 0;				// ������ ��� ������������� �������� ������� ��������� � ���� �����
uint8_t search_next_try = 0;				// ����� ����������� ������� ������ ��������� � ���� �����

extern uint8_t alarm_flag;					// ������ �����

extern uint16_t point_tmr;					// ������ ��� �������� ����� ���� ���� ���� ���������� ����� � �����������
extern uint8_t point_flag;					// ���� ������� ����� � �����������
// ��� ������ � ���������� ����� ���������� ��� ���� ��� ������ ������������� ����������, � �� ���� ������ � ����
// ��� ������������ ������� (��������� �� �������� ����� � ������� ���������� �������)
// ���������� ����� �������� ���� �� ��� ����

extern uint8_t button1;				// ������ ������ ���������� ����� � ����

extern uint8_t prev_pow_data[2];	// ���������� �������� ���������� ��� ������������ ���������
extern uint8_t pow_data[2];			// ���������� ������������ � ������� 1 �� - 0.1�

extern RNG_HandleTypeDef hrng;		// random number generator ��� ���������� � ��������� ����� �� ��������� ��������
// ����� ��������� ������� �������� �� ����

extern uint8_t limit_switch;		// ��������� ��������� ������������� (����� ��������� � ����)

extern uint8_t call_flag;			// ���� �������� ������
extern uint16_t call_tmr;			// ������ ��� ����������� ����������� �������� ������

extern uint16_t adc_tmr;			// ������ ��� ���������� ������ ���
extern uint16_t led_cnt;			// ������� ��� ������������ �����������

extern struct dinput di1,di2;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_dfsdm1_flt0;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  break;
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
	  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  break;
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
	  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  break;
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
	  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  break;
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	//HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);

  static uint16_t i=0;
  static uint8_t state = 0;
  static uint16_t sec_tmr=0;
  static uint32_t upd_tmr = 0;

  //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_SET);

  // ������������� �������� ��������� ����� �� ��������� ���������� 0 - 15 ������
  if(upd_tmr==0) {
	  upd_tmr = HAL_RNG_GetRandomNumber(&hrng) & 0x0F;
	  send_point_state(1);
  }

  // �������� ����� ������������ � ������� ��� ������� � ����
  can_write_from_stack();

  if(test_2_5_kHz_state) test_2_5_kHz_tmr++;

  search_next_tmr++;
  if(search_next_tmr>=100) {
	  search_next_tmr = 0;
	  if(search_next_try>=10 && limit_switch==0) { last_point(); }	// ��������� ����� � ����, �������� ��������
	  next_point(FIND_REQUEST);	// ����� ��������� ����� ������ 100 ��
	  if(search_next_try<10) search_next_try++;
	  sec_tmr++;
	  if(sec_tmr>=10) {	// ����������� ��� � �������
		  if(upd_tmr) upd_tmr--;
		  if(limit_switch) last_point();
		  sec_tmr = 0;
		  // ������ ������� ���������� � ����������� ������������� ������ ���� ����� �� ����������� ������ � ����
		  if(button1==0) point_tmr++;
		  if(point_tmr>=600) {
			  point_tmr = 0;
			  point_flag = 0;
		  }
		  // �������� ������ �� ��������� ��� � ������ ������ (��������� ���������� ������ �� ������� ����������� ����������)
		  if(prev_pow_data[0]!=pow_data[0] || prev_pow_data[1]!=pow_data[1] || alarm_flag) {
			  send_point_state(1);
		  }
		  prev_pow_data[0] = pow_data[0];
		  prev_pow_data[1] = pow_data[1];

	  }
  }
  if(packet_tmr<500) packet_tmr++;
  // �������� ������ �� ��������� �������� ��������� �����
  if(prev_discr_state != discrete_state) {
	  send_point_state(1);
  }
  prev_discr_state = discrete_state;

  // ������� ��� ������� � ������ ������ � ��������� �� UART
  i++;
  if(i==25) {	// ��������� ����������� ����� 1
	  tx1_buf[0]=0x39;
	  tx1_buf[1]=0xAA;
	  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
	  send_data_to_uart1(tx1_buf,3);
  }
  if(i>=50) {
	  i = 0;
	  switch(state) {
	  case 0:	// ��������� �����
		  tx1_buf[0]=0xDE;
		  tx1_buf[1]=0xDD;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 1:	// ������ 1
		  tx1_buf[0]=0x49;
		  tx1_buf[1]=0xAA;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 2:	// ������ 2
	      tx1_buf[0]=0x4A;
	      tx1_buf[1]=0xAA;
	      tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
	      send_data_to_uart1(tx1_buf,3);
	      break;
	  case 3:// ����� ���� 1
		  tx1_buf[0]=0x39;
		  tx1_buf[1]=0xAA;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 4:// ����� ���� 2
		  tx1_buf[0]=0x3A;
		  tx1_buf[1]=0xAA;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 5:// ���� 1
		  tx1_buf[0]=0x61;
		  if((discrete_state>>8)&0x40 && alarm_flag==0) tx1_buf[1]=0x0F;
		  else tx1_buf[1]=0xF0;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 6:// ���� 2
		  tx1_buf[0]=0x62;
		  if((discrete_state>>8)&0x80) tx1_buf[1]=0x0F;
		  else tx1_buf[1]=0xF0;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 7:// ��������
		  tx1_buf[0]=0x4B;
		  tx1_buf[1]=0xAA;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  case 8:// ��� ������� �������
		  tx1_buf[0]=0x9A;
		  tx1_buf[1]=0xAA;
		  tx1_buf[2]=tx1_buf[0] + tx1_buf[1];
		  send_data_to_uart1(tx1_buf,3);
		  break;
	  default:
		  break;
	  }
	  state++;
	  if(state>=10)  {
		  if(di1.tmr<di1.tmr_limit && di1.state == OFF) di1.tmr++;
		  if(di2.tmr<di2.tmr_limit && di2.state == OFF) di2.tmr++;
		  state=0;
	  }
  }

  if(rx1_cnt) {rx1_tmr++;}else rx1_tmr=0;

  if(call_tmr<100) call_tmr++;else {call_flag=0;}

  adc_tmr++;

  led_cnt++;if(led_cnt>=1000) {led_cnt=0;}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_RESET);

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
  if(LL_DMA_IsActiveFlag_TE3(DMA1) == 1)
  {
	//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	LL_DMA_ClearFlag_TE3(DMA1);
  }
  if(LL_DMA_IsActiveFlag_TC3(DMA1) == 1)
  {
	  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
  	LL_DMA_ClearFlag_TC3(DMA1);
  	DmaDacBuffCplt = 1;
  }
  if(LL_DMA_IsActiveFlag_HT3(DMA1) == 1)
  {
	LL_DMA_ClearFlag_HT3(DMA1);
    DmaDacHalfBuffCplt = 1;
  }


  /* USER CODE END DMA1_Channel3_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dfsdm1_flt0);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */



  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

	if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
	{
		//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		rx1_buf[rx1_cnt++] = LL_USART_ReceiveData8(USART1);
		if(rx1_cnt>=UART_BUF_SISE) rx1_cnt = 0;
		rx1_tmr = 0;
	};

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	if(LL_DAC_IsActiveFlag_DMAUDR1(DAC1) != 0)
	{
	    LL_DAC_ClearFlag_DMAUDR1(DAC1);
	}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel6 global interrupt.
  */
void DMA2_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel6_IRQn 0 */

	if(LL_DMA_IsActiveFlag_TC6(DMA2))
	{
	LL_DMA_ClearFlag_GI6(DMA2);
		/* Call function Transmission complete Callback */
		LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_6);
	}
	else if(LL_DMA_IsActiveFlag_TE6(DMA2))
	{
	 /* Call Error function */
		LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_6);
	}

  /* USER CODE END DMA2_Channel6_IRQn 0 */
  
  /* USER CODE BEGIN DMA2_Channel6_IRQn 1 */

  /* USER CODE END DMA2_Channel6_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupt.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
