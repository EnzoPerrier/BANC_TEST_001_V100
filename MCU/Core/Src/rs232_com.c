/*Add commentMore actions
 * rs232_com.c
 *
 *  Created on: Jun 24, 2025
 *      Author: Enzo Perrier
 */

#include "rs232_com.h"

#define RX_BUFFER3_SIZE 100

uint8_t rx_char3;
uint8_t rx_buffer3[RX_BUFFER3_SIZE];
uint16_t rx_index3 = 0;
uint8_t message_complete3 = 0;

UART_HandleTypeDef huart3;
/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void RS232_COM_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
}

void send_UART3(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void process_UART3_data(void)
{
    // A appeler dans la machine à états
    if (message_complete3)
    {
        message_complete3 = 0;
        // Traitement...
    }
}

// Callback à appeler depuis main.c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_char3 != '\n' && rx_index3 < RX_BUFFER3_SIZE - 1)
        {
            rx_buffer3[rx_index3++] = rx_char3;
        }
        else
        {
            rx_buffer3[rx_index3] = '\0';
            message_complete3 = 1;
            rx_index3 = 0;Add commentMore actions
        }
        HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
    }
}
