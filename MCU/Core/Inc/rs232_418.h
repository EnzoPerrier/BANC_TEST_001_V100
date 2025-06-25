/*
 * rs232_418.h
 *
 *  Created on: Jun 24, 2025
 *      Author: Enzo Perrier
 */

#ifndef INC_RS232_418_H_
#define INC_RS232_418_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

// Taille buffer
#define RX_BUFFER1_SIZE 100

// Variables externes
extern UART_HandleTypeDef huart1;
extern uint8_t rx_char1;
extern uint8_t rx_buffer1[RX_BUFFER1_SIZE];
extern uint8_t message_complete1;

// Fonctions
void RS232_418_Init(void);
void send_UART1(const char *msg);
void process_UART1_data(void);


#endif /* INC_RS232_418_H_ */
