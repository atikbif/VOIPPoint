/*
 * uart.h
 *
 *  Created on: 24 мая 2019 г.
 *      Author: User
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#define UART_BUF_SISE	32

void uart1_scan(void);
void send_data_to_uart1(uint8_t *ptr, uint16_t cnt);

#endif /* UART_H_ */
