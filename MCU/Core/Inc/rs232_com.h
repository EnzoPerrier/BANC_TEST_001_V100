/*
 * rs232_com.h
 *
 *  Created on: Jun 29, 2025
 *      Author: katia
 */

#ifndef INC_RS232_COM_H_
#define INC_RS232_COM_H_

#include "main.h"

extern UART_HandleTypeDef huart3;
extern uint8_t message_complete3;

void MX_USART3_UART_Init(void);

void RS232_COM_Init(void);
void send_UART3(const char *msg);
void process_UART3_data(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_RS232_COM_H_ */
