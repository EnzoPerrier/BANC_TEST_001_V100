/*
 * rs232_418.h
 *
 *  Created on: Jun 29, 2025
 *      Author: katia
 */

#ifndef INC_RS232_418_H_
#define INC_RS232_418_H_

#include "main.h"

extern UART_HandleTypeDef huart1;

extern uint8_t message_complete1;

void MX_USART1_UART_Init(void);

void RS232_418_Init(void);
void send_UART1(const char *msg);
void process_UART1_data(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_RS232_418_H_ */
