/*
 * uart.h
 *
 *  Created on: Jun 30, 2025
 *      Author: Enzo Perrier
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include <stdbool.h>

#define RX_BUFFER1_SIZE 100
#define RX_BUFFER2_SIZE 100
#define RX_BUFFER3_SIZE 100


//RS232_418
extern UART_HandleTypeDef huart1;

extern uint8_t message_complete1;
extern uint8_t rx_char1;
extern uint8_t rx_buffer1[RX_BUFFER1_SIZE];

void MX_USART1_UART_Init(void);

void RS232_418_Init(void);
void send_UART1(const char *msg);
void process_UART1_data(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//---------------------------------- STRUCTURE PARSE TRAME STS
typedef struct
{ // Struct Trame de  retour STS
  char ver[32];
  char crc[32];
  char lan[16];
  float acc;
  float bat;
  float cel_val;
  char cel_mode;
  char lum;
  bool dips[8]; // true = ON, false = OFF
  bool inps[3]; // true = ON, false = OFF (si jamais ça change)
} TrameDataSTS;


// RS232_COM
extern UART_HandleTypeDef huart3;

extern uint8_t message_complete3;
extern uint8_t rx_char3;
extern uint8_t rx_buffer3[RX_BUFFER3_SIZE];

void MX_USART3_UART_Init(void);

void RS232_COM_Init(void);
void send_UART3(const char *msg);
void process_UART3_data(void);

//RS485
extern UART_HandleTypeDef huart2;

extern uint8_t message_complete2;
extern uint8_t rx_char2;
extern uint8_t rx_buffer2[RX_BUFFER2_SIZE];

void MX_USART2_UART_Init(void);

void RS485_Init(void);
void send_UART2(const char *msg);
void process_UART2_data(void);


#endif /* INC_UART_H_ */
