/*
 * rs232_com.h
 *
 *  Created on: Jun 24, 2025
 *      Author: Enzo Perrier
 */

#ifndef INC_RS232_COM_H_
#define INC_RS232_COM_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

// Taille buffer
#define RX_BUFFER3_SIZE 100

// Variables
UART_HandleTypeDef huart3;
uint8_t rx_char3;
uint8_t rx_buffer3[RX_BUFFER3_SIZE];
uint8_t message_complete3;

// Fonctions
void RS232_COM_Init(void);
void send_UART3(const char *msg);
void process_UART3_data(void);


#endif /* INC_RS232_COM_H_ */
