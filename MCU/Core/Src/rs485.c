#include "rs485.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2; // USART2 utilisé pour le RS485
extern osMutexId uartMutexHandle; // Optionnel si tu veux utiliser un mutex

static uint8_t rx_char_rs485;
static uint8_t rx_buffer_rs485[RS485_RX_BUFFER_SIZE];
static uint16_t rx_index_rs485 = 0;
static uint8_t message_complete_rs485 = 0;

// Pour réception interrupt
void RS485_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_char_rs485, 1);
}

// Envoi sur RS485
void RS485_Send(const char* msg)
{
    // Active la ligne RTS (optionnel selon le driver RS485 utilisé)
    HAL_GPIO_WritePin(RTS_485_GPIO_Port, RTS_485_Pin, GPIO_PIN_SET); // Enable TX
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RTS_485_GPIO_Port, RTS_485_Pin, GPIO_PIN_RESET); // Disable TX
}

// Callback de réception UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (rx_char_rs485 != '\n' && rx_index_rs485 < RS485_RX_BUFFER_SIZE - 1)
        {
            rx_buffer_rs485[rx_index_rs485++] = rx_char_rs485;
        }
        else
        {
            rx_buffer_rs485[rx_index_rs485] = '\0';
            message_complete_rs485 = 1;
            rx_index_rs485 = 0;
        }

        HAL_UART_Receive_IT(&huart2, &rx_char_rs485, 1);
    }
}

// Traitement de la trame reçue (à appeler dans une tâche FreeRTOS)
void RS485_Process(void)
{
    if (message_complete_rs485)
    {
        message_complete_rs485 = 0;

        // Exemple : afficher ou traiter la trame reçue
        char response[120];
        sprintf(response, "RS485 received: %s\r\n", rx_buffer_rs485);
        RS485_Send(response);
    }
}
