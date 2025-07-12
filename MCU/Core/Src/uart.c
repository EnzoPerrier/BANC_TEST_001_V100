/*
 * uart.c
 *
 *  Created on: Jun 30, 2025
 *      Author: Enzo Perrier
 */

#include "uart.h"
#include "main.h"

#include <string.h>

//RS232_418
UART_HandleTypeDef huart1;

uint8_t message_complete1 = 0;
uint8_t rx_char1 = 0;
uint8_t rx_buffer1[RX_BUFFER1_SIZE] = {0};
uint16_t rx_index1 = 0;

// RS232_COM
UART_HandleTypeDef huart3;

uint8_t message_complete3 = 0;
uint8_t rx_char3 = 0;
uint8_t rx_buffer3[RX_BUFFER3_SIZE] = {0};
uint16_t rx_index3 = 0;

//RS485
UART_HandleTypeDef huart2;

uint8_t message_complete2 = 0;
uint8_t rx_char2 = 0;
uint8_t rx_buffer2[RX_BUFFER2_SIZE] = {0};
uint16_t rx_index2 = 0;


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
  HAL_UART_Receive_IT(&huart1, &rx_char1, 1);

  /* USER CODE END USART1_Init 2 */

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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, &rx_char2, 1);

  /* USER CODE END USART2_Init 2 */

}


void send_UART2(const char *msg)
{
	HAL_GPIO_WritePin(RTS_485_GPIO_Port, RTS_485_Pin, GPIO_PIN_SET); // DEBUG
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void process_UART2_data(void)
{
    // A appeler dans la machine à états
    if (message_complete2)
    {
        message_complete2 = 0;
        // Traitement...
    }
}





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
  //HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
  /* USER CODE END USART3_Init 2 */

}

void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart3);  // Appel à HAL pour gérer l'interruption
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


// Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(OUT4_GPIO_Port, OUT4_Pin); //DEBUG
	// RS232_418
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

    //RS232_COM
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
                rx_index3 = 0;
            }
            HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
            send_UART3("RECU"); // DEBUG
        }

    //RS485
        if (huart->Instance == USART2)
            {
                if (rx_char2 != '\n' && rx_index2 < RX_BUFFER2_SIZE - 1)
                {
                    rx_buffer2[rx_index2++] = rx_char2;
                }
                else
                {
                    rx_buffer2[rx_index2] = '\0';
                    message_complete2 = 1;
                    rx_index2 = 0;
                }
                HAL_UART_Receive_IT(&huart2, &rx_char2, 1);
            }

}





