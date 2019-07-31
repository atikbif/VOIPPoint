/*
 * uart.c
 *
 *  Created on: 24 мая 2019 г.
 *      Author: User
 */

#include "uart.h"
#include "main.h"

#define UART_RX_READY_TIME_MS		10

uint8_t rx1_buf[UART_BUF_SISE];
uint8_t tx1_buf[UART_BUF_SISE];
uint16_t rx1_cnt = 0;
uint16_t rx1_tmr = 0;

__weak void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {

}

__weak void rx2_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {

}

void uart1_scan(void) {

	if(rx1_cnt && rx1_tmr>=UART_RX_READY_TIME_MS) {

		rx1_callback(rx1_buf,rx1_cnt);
		rx1_cnt = 0;
		rx1_tmr = 0;
	}
}

void send_data_to_uart1(uint8_t *ptr, uint16_t cnt) {
	uint16_t i=0;
	for(i=0;i<cnt;i++) tx1_buf[i] = ptr[i];
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_6,
		(uint32_t)tx1_buf,
		LL_USART_DMA_GetRegAddr(USART1,LL_USART_DMA_REG_DATA_TRANSMIT),
		LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_6));
	LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_6, cnt);
	LL_USART_EnableDMAReq_TX(USART1);
	LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_6);
}

