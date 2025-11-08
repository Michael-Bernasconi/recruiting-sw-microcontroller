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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   // necessario per rand()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USER_BUTTON_Pin       GPIO_PIN_13  //USER BUTTON PIN NUMBER
#define USER_BUTTON_GPIO_Port GPIOC        //PORT FOR USER BOTTON
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;     // ADC handle
DMA_HandleTypeDef hdma_adc1; // DMA handle for ADC (not used in main loop)
UART_HandleTypeDef huart2;   // UART handle

/* USER CODE BEGIN PV */
#define MA_SIZE 150           // BUFFER FOR MOVING AVERAGE AND INDEX
#define POT_THRESHOLD 2000    // Threshold for WARNING
#define POT_THRESHOLDERROR 3000  // Threshold for ERROR
uint32_t adcBuffer[MA_SIZE] = {0};  // Circular buffer for MA
uint16_t bufferIndex = 0;        // Index in circular buffer
uint8_t buttonPrevState = GPIO_PIN_SET; // Previous button state, default not pressed

// Seleziona il tipo di filtro: RAW, MOVING_AVERAGE, RANDOM_NOISE
typedef enum { RAW, MOVING_AVERAGE, RANDOM_NOISE } FilterMode;
FilterMode currentMode = RAW;  // RAW default


// -------------------- STATE MACHINE --------------------
typedef enum {
    STATE_INIT,
    STATE_WAIT_REQUEST,
    STATE_LISTENING,
    STATE_PAUSE,
    STATE_WARNING,
    STATE_ERROR
} SystemState;
SystemState currentState = STATE_INIT;// Current system state
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FUNCTION TO COMPUTE MOVING AVERAGE
uint32_t computeMovingAverage(uint32_t newSample) {
    static uint64_t sum = 0;
    sum -= adcBuffer[bufferIndex];     // SUBTRACT THE OLD VALUE FROM SUM
    adcBuffer[bufferIndex] = newSample;      // ADD THE NEW VALUE INTO BUFFER AND SUM
    sum += newSample;
    bufferIndex = (bufferIndex + 1) % MA_SIZE;      // ADVANCE BUFFER INDEX IN CIRCULAR MANNER
    return (uint32_t)(sum / MA_SIZE);
}

// FUNCTION TO ADD RANDOM NOISE
uint32_t addRandomNoise(uint32_t value) {
    int32_t noise = (rand()%40)+250;  // Random noise between 250 and 289
    int32_t result = (int32_t)value + noise;
    if (result < 0) result = 0;
    if (result > 4095) result = 4095;
    return (uint32_t)result;
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Initialize LEDs */
  BSP_LED_Init(LED_GREEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    // --- READ BUTTON AND DETECT FALLING EDGE ---
    static uint8_t buttonPrevState = GPIO_PIN_SET; // initially not pressed
    uint8_t buttonState = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

    if(buttonPrevState == GPIO_PIN_SET && buttonState == GPIO_PIN_RESET) // CHECK IF BUTTON WAS JUST PRESSED
    {
        if(currentState == STATE_WAIT_REQUEST)
            currentState = STATE_LISTENING;
        else if(currentState == STATE_LISTENING)
            currentState = STATE_PAUSE;
        else if(currentState == STATE_PAUSE)
            currentState = STATE_LISTENING;
        else if(currentState == STATE_WARNING)
            currentState = STATE_WAIT_REQUEST;
        else if(currentState == STATE_ERROR)
            NVIC_SystemReset(); // RESET MCU IN CASE OF ERROR
    }
    buttonPrevState = buttonState; // UPDATE BUTTON STATE

    // --- STATE MACHINE HANDLER ---
    switch(currentState)
    {
        case STATE_INIT:
            currentState = STATE_WAIT_REQUEST;  // IMMEDIATELY TRANSITION TO WAIT_REQUEST
            break;

        case STATE_WAIT_REQUEST:
            BSP_LED_Off(LED_GREEN); // LED OFF
            HAL_Delay(100);
            break;

        case STATE_LISTENING:
       {
    BSP_LED_On(LED_GREEN); // LED ON

    //START ADC CONVERSION
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t rawValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t maValue = computeMovingAverage(rawValue);
    uint32_t noiseValue = addRandomNoise(rawValue);
    // CONVERT TO MILLIVOLTS
    uint32_t raw_mv   = (rawValue   * 3300) / 4095;
    uint32_t ma_mv    = (maValue    * 3300) / 4095;
    uint32_t noise_mv = (noiseValue * 3300) / 4095;

    char msg[100];

    // SEND RAW VALUE OVER UART
    sprintf(msg, "Mode=RAW | ADC=%lu -> %lu mV\r\n", rawValue, raw_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // SEND MOVING AVERAGE VALUE OVER UART
    sprintf(msg, "Mode=MA | ADC=%lu -> %lu mV\r\n", maValue, ma_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // SEND NOISE VALUE OVER UART
    sprintf(msg, "Mode=NOISE | ADC=%lu -> %lu mV\r\n", noiseValue, noise_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // CHECK THRESHOLDS
    static uint32_t highStart = 0;
    if(rawValue > POT_THRESHOLDERROR)
    {
        currentState = STATE_ERROR; // TRANSITION TO ERROR
    }
    else if(rawValue > POT_THRESHOLD)
    {
        if(highStart == 0) highStart = HAL_GetTick();
        else if(HAL_GetTick() - highStart >= 5000)
        {
            currentState = STATE_WARNING;  // TRANSITION TO WARNING
            highStart = 0;
        }
    }
    else highStart = 0;

    HAL_Delay(100);
}
break;

        case STATE_PAUSE:
            BSP_LED_Toggle(LED_GREEN); // LED BLINKING
            HAL_Delay(500); // 50% duty

            break;

        case STATE_WARNING:
            BSP_LED_Off(LED_GREEN); //LED OFF
            HAL_UART_Transmit(&huart2, (uint8_t*)"WARNING\r\n", 9, HAL_MAX_DELAY);
            HAL_Delay(200);
            break;

        case STATE_ERROR:
            BSP_LED_Toggle(LED_GREEN);
            HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\r\n", 7, HAL_MAX_DELAY);
            HAL_Delay(200);
            break;
    }
}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/* ------------------------ STM32 GENERATED FUNCTIONS ------------------------ */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();
     GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ACTIVE clock GPIO
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure USER BUTTON (PC13) as input
    GPIO_InitStruct.Pin = USER_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

    // Configure LED (PA5) as output
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Optional: printf("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif