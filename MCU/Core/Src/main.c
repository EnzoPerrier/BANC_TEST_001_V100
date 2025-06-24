/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_BUFFER1_SIZE 100
#define RX_BUFFER3_SIZE 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId State_machineHandle;
osThreadId Update_screenHandle;
osThreadId Update_measureHandle;
osThreadId LED_VIEHandle;
/* USER CODE BEGIN PV */

uint8_t state; // Etat pour la machine à état

uint8_t rx_char1, rx_char3; // UART1 = 232_418 et UART3 = 232_COM
uint8_t rx_buffer1[RX_BUFFER1_SIZE];
uint8_t rx_buffer3[RX_BUFFER3_SIZE];
uint16_t rx_index1 = 0, rx_index3 = 0;
uint8_t message_complete1 = 0;
uint8_t message_complete3 = 0;
char per_value[8] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void Update_StateMachine(void const *argument);
void Update_Screen(void const *argument);
void Update_Measures(void const *argument);
void LED_VIE_App(void const *argument);

/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void send_UART1(const char *msg);
void send_UART3(const char *msg);
void start_UART_Reception(void);
void TEST_STATE_MACHINE(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  start_UART_Reception(); // Initialise la réception UART1 et 3

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of State_machine */
  osThreadDef(State_machine, Update_StateMachine, osPriorityNormal, 0, 128);
  State_machineHandle = osThreadCreate(osThread(State_machine), NULL);

  /* definition and creation of Update_screen */
  osThreadDef(Update_screen, Update_Screen, osPriorityIdle, 0, 128);
  Update_screenHandle = osThreadCreate(osThread(Update_screen), NULL);

  /* definition and creation of Update_measure */
  osThreadDef(Update_measure, Update_Measures, osPriorityIdle, 0, 128);
  Update_measureHandle = osThreadCreate(osThread(Update_measure), NULL);

  /* definition and creation of LED_VIE */
  osThreadDef(LED_VIE, LED_VIE_App, osPriorityLow, 0, 128);
  LED_VIEHandle = osThreadCreate(osThread(LED_VIE), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
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

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
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

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
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

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT8_Pin | BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RTS_485_Pin | LED_CEL_Pin | OUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDR_Pin | LEDG_Pin | LEDY_Pin | RELAIS_ALIM_418_Pin | OUT1_Pin | OUT3_Pin | OUT4_Pin | OUT5_Pin | OUT6_Pin | OUT7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUT8_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = OUT8_Pin | BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP1_Pin */
  GPIO_InitStruct.Pin = DIP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIP1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP2_Pin DIP3_Pin BP2_Pin BP3_Pin
                           BP4_Pin */
  GPIO_InitStruct.Pin = DIP2_Pin | DIP3_Pin | BP2_Pin | BP3_Pin | BP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RTS_485_Pin LED_CEL_Pin OUT2_Pin */
  GPIO_InitStruct.Pin = RTS_485_Pin | LED_CEL_Pin | OUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDR_Pin LEDG_Pin LEDY_Pin RELAIS_ALIM_418_Pin
                           OUT1_Pin OUT3_Pin OUT4_Pin OUT5_Pin
                           OUT6_Pin OUT7_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin | LEDG_Pin | LEDY_Pin | RELAIS_ALIM_418_Pin | OUT1_Pin | OUT3_Pin | OUT4_Pin | OUT5_Pin | OUT6_Pin | OUT7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BP1_IRQ_Pin */
  GPIO_InitStruct.Pin = BP1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BP1_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void TEST_STATE_MACHINE(void) {
    static uint8_t action_done = 0;

    //--------------------------- TRANSITIONS
    switch (state) {
        case 0:
        case 2:
        case 5:
        case 6:
        case 7:
            if (HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) == GPIO_PIN_RESET) {
                state++;
                action_done = 0;
            } else if (HAL_GPIO_ReadPin(BP3_GPIO_Port, BP3_Pin) == GPIO_PIN_RESET && state > 0) {
                state--;
                action_done = 0;
            }
            break;
        case 1:
            if (message_complete1) {
                message_complete1 = 0;
                char expected_response[20];
                sprintf(expected_response, "PER=%s", per_value);
                if (strstr((char *)rx_buffer1, expected_response) == (char *)rx_buffer1) {
                    send_UART3("PER VALIDE --> Etape suivante\n");
                    HAL_Delay(500);
                    state++;
                    action_done = 0;
                } else {
                    send_UART3("Valeur differente. Entrez a nouveau:\n");
                }
            }
            break;
        case 3:
            if (message_complete1) {
                message_complete1 = 0;
                TrameDataSTS data = {0};
                parse_data_STS(rx_buffer1, &data);
                if ((data.acc >= 8.5 && data.acc <= 10) && (data.bat >= 11.5 && data.bat <= 13) && (data.dips[0] == 1 && data.dips[1] == 1 && data.dips[2] == 1 && data.dips[3] == 1 && data.dips[4] == 1 && data.dips[5] == 1 && data.dips[6] == 1 && data.dips[7] == 1)) {
                    send_UART3("STS OK --> Etape suivante\n");
                    HAL_Delay(500);
                    state++;
                }
            }
            break;
        case 4:
            if (message_complete1) {
                message_complete1 = 0;
                TrameDataSTS data = {0};
                parse_data_STS(rx_buffer1, &data);
                if (data.inps[0] == 1 && data.inps[1] == 1 && data.inps[2] == 1) {
                    send_UART3("Entrees OK --> Etape suivante\n");
                    HAL_Delay(500);
                    state++;
                    action_done = 0;
                } else {
                    send_UART3("Erreur défaut entrée:");
                    for (int i = 0; i < 3; i++) {
                        char msg[50];
                        sprintf(msg, "Entree num %d : %d\n", i, data.inps[i]);
                        send_UART3(msg);
                    }
                }
            }
            break;
        case 8:
            if (HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) == GPIO_PIN_RESET) {
                state = 0;
                action_done = 0;
            }
            break;
    }

    //--------------------------- ACTIONS
    switch (state) {
        case 0:
            if (!action_done) {
                send_UART3("Appuyer sur le bouton pour commencer\n");
                action_done = 1;
            }
            break;
        case 1:
            if (!action_done) {
                send_UART3("Entrez les PER (juste la valeur sur 8 digits)\n");
                if (message_complete3) {
                    message_complete3 = 0;
                    if (strlen((char *)rx_buffer3) == MAX_PER_LENGTH) {
                        strcpy(per_value, (char *)rx_buffer3);
                        char per_command[20];
                        sprintf(per_command, "PER=%s\n", per_value);
                        send_UART1(per_command);
                        HAL_Delay(500);
                        send_UART1("PER=\n");
                    } else {
                        send_UART3("Format invalide. Le PER est sur 8 digits, recommencez...\n");
                    }
                }
                action_done = 1;
            }
            break;
        case 2:
            if (!action_done) {
                send_UART3("Mettez tous les DIPs sur ON, une fois fait appuyez sur le bouton\n");
                action_done = 1;
            }
            break;
        case 3:
            if (!action_done) {
                send_UART3("Test STS en cours...\n");
                send_UART1("STS\n");
                send_UART3(rx_buffer1);
                action_done = 1;
            }
            break;
        case 4:
            if (!action_done) {
                send_UART3("Test entrees en cours...\n");
                // Activation de toutes les entrées
                HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
                HAL_Delay(300);
                send_UART1("STS\n");
                action_done = 1;
            }
            break;
        case 5:
            send_UART3("Test du décompteur...\n Veuillez valider en appuyant sur le BP si toutes les leds s'allument correctement et dans le bon ordre sur le décompteur");
            send_UART1("TST=1\n");
            break;
        case 6:
            send_UART1("TST=0");
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
            send_UART3("Test des ampoules ...\n Vérifiez que les ampoules s'éteignent et se rallument et que le défaut sur l'écran LCD de la carte corresponde bien à la bonne ampoule");
            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
            HAL_Delay(1500);
            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
            HAL_Delay(1500);
            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
            HAL_Delay(1500);
            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
            break;
        case 7:
            send_UART3("Test de l'infrarouge...\n Veuillez valider en appuyant sur le BP si la télécommande fonctionne en émission et réception");
            break;
        case 8:
            HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_SET);
            send_UART3("Test de l'accu...\n Veuillez vérifier que vous avez bien le message suppression batterie qui s'affiche à l'écran, si le cas validez");
            break;
        default:
            break;
    }
}

//----------------------------------------------------------------------------------- CALLBACK RECEPTION DE DONNEES USART1
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
  else if (huart->Instance == USART3)
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
  }
}

//----------------------------------------------------------------------------------- FONCTIONS DE TRANSMISSION DES DONNES
void send_UART1(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void send_UART3(const char *msg)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

//----------------------------------------------------------------------------------- Start réception
void start_UART_Reception()
{
  HAL_UART_Receive_IT(&huart1, &rx_char1, 1);
  HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
}

//----------------------------------------------------------------------------------- FONCTION DE PARSING STS

void parse_data_STS(const char *trame, TrameDataSTS *data)
{
  char line[128];
  const char *ptr = trame;

  while (*ptr)
  {
    // Extraire une ligne complète
    int len = 0;
    while (*ptr && *ptr != '\n' && len < sizeof(line) - 1)
    {
      line[len++] = *ptr++;
    }
    if (*ptr == '\n')
      ptr++; // Sauter '\n'
    line[len] = '\0';

    // Analyse ligne par ligne
    if (sscanf(line, "VER = %31s", data->ver) == 1)
      continue;
    if (sscanf(line, "CRC = %31s", data->crc) == 1)
      continue;
    if (sscanf(line, "LAN = %15s", data->lan) == 1)
      continue;
    if (sscanf(line, "ACC = %f v", &data->acc) == 1)
      continue;
    if (sscanf(line, "BAT = %f v", &data->bat) == 1)
      continue;
    if (sscanf(line, "CEL = %f v", &data->cel_val) == 1)
      continue;
    if (sscanf(line, "CEL = %c", &data->cel_mode) == 1)
      continue;
    if (sscanf(line, "LUM = %c", &data->lum) == 1)
      continue;

    // DIP (1 à 8)
    if (strncmp(line, "DIP =", 5) == 0)
    {
      for (int i = 0; i < 8; i++) // 8  DIPs
      {
        char pattern[8];
        sprintf(pattern, "%d:ON", i + 1); // Exemple j'ai 1:ON  2:OFF--> 10
        data->dips[i] = strstr(line, pattern) != NULL;
      }

      // INP (1 à 3)
      if (strncmp(line, "INP =", 5) == 0) // 3 entrées
      {
        for (int i = 0; i < 3; i++)
        {
          char pattern[8];
          sprintf(pattern, "%d:ON", i + 1);
          data->inps[i] = strstr(line, pattern) != NULL;
        }
        continue;
      }
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Update_StateMachine */
/**
 * @brief  Function implementing the State_machine thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Update_StateMachine */
void Update_StateMachine(void const *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    TEST_STATE_MACHINE();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Update_Screen */
/**
 * @brief Function implementing the Update_screen thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Update_Screen */
void Update_Screen(void const *argument)
{
  /* USER CODE BEGIN Update_Screen */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END Update_Screen */
}

/* USER CODE BEGIN Header_Update_Measures */
/**
 * @brief Function implementing the Update_measure thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Update_Measures */
void Update_Measures(void const *argument)
{
  /* USER CODE BEGIN Update_Measures */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END Update_Measures */
}

/* USER CODE BEGIN Header_LED_VIE_App */
/**
 * @brief Function implementing the LED_VIE thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LED_VIE_App */
void LED_VIE_App(void const *argument)
{
  /* USER CODE BEGIN LED_VIE_App */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(OUT4_GPIO_Port, OUT4_Pin);
    osDelay(500);
  }
  /* USER CODE END LED_VIE_App */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
