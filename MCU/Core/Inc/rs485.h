/*
 * rs485.h
 *
 *  Created on: Jun 24, 2025
 *      Author: Enzo Perrier
 */

#ifndef INC_RS485_H_
#define INC_RS485_H_

#include "main.h"
#include "cmsis_os.h"

// Taille du buffer
#define RS485_RX_BUFFER_SIZE 100

// Fonctions accessibles
void RS485_Init(void);
void RS485_Send(const char* msg);
void RS485_Process(void);


#endif /* INC_RS485_H_ */
