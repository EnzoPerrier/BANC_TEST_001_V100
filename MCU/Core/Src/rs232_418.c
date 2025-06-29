/*
 * rs232_418.c
 *
 *  Created on: Jun 29, 2025
 *      Author: Enzo Perrier
 */

/*Add commentMore actions
 * rs232_com.c
 *
 *  Created on: Jun 24, 2025
 *      Author: Enzo Perrier
 */

#include "rs232_418.h"

#define RX_BUFFER1_SIZE 100

uint8_t rx_char1;
uint8_t rx_buffer1[RX_BUFFER1_SIZE];
uint16_t rx_index1 = 0;
uint8_t message_complete1 = 0;

UART_HandleTypeDef huart1;

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}



void RS232_418_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_char1, 1);
}

void send_UART1(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void process_UART1_data(void)
{
    // A appeler dans la machine à états
    if (message_complete1)
    {
        message_complete1 = 0;
        // Traitement...
    }
}

// Callback à appeler depuis main.c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_char1 != '\n' && rx_index1 < RX_BUFFER1_SIZE - 1)
        {
            rx_buffer1[rx_index1++] = rx_char1;
        }
        else
        {
            rx_buffer1[rx_index1] = '\0';
            message_complete1 = 1;
            rx_index1 = 0;
        }
        HAL_UART_Receive_IT(&huart1, &rx_char1, 1);
    }
}



