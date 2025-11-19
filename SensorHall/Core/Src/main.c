/* USER CODE BEGIN Header */
/**

---

* @file           : main.c
* @brief          : Main program body

---

* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*

---

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef enum { RAW, MOVING_AVERAGE, RANDOM_NOISE } FilterMode;
typedef enum {
STATE_INIT,
STATE_WAIT_REQUEST,
STATE_LISTENING,
STATE_PAUSE,
STATE_WARNING,
STATE_ERROR
} SystemState;

/* Private define ------------------------------------------------------------*/
#define USER_BUTTON_Pin       GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC

#define LED_GREEN_Pin         GPIO_PIN_5
#define LED_GREEN_GPIO_Port   GPIOA

#define MA_SIZE 150
#define POT_THRESHOLD 2000
#define POT_THRESHOLDERROR 3000

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart2;

uint32_t adcBuffer[MA_SIZE] = {0};
uint16_t bufferIndex = 0;

volatile uint8_t buttonPressed = 0;
FilterMode currentMode = RAW;
SystemState currentState = STATE_INIT;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PV */

/* Compute Moving Average */
uint32_t computeMovingAverage(uint32_t newSample) {
static uint64_t sum = 0;
sum -= adcBuffer[bufferIndex];
adcBuffer[bufferIndex] = newSample;
sum += newSample;
bufferIndex = (bufferIndex + 1) % MA_SIZE;
return (uint32_t)(sum / MA_SIZE);
}

/* Add Random Noise */
uint32_t addRandomNoise(uint32_t value) {
int32_t noise = (rand() % 40) + 250;
int32_t result = value + noise;
if (result < 0) result = 0;
if (result > 4095) result = 4095;
return (uint32_t)result;
}

/* USER CODE END PV */

/**

* @brief  The application entry point.
* @retval int
  */
  int main(void)
  {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  uint8_t rx;
  uint8_t buttonPrevState = GPIO_PIN_SET;

  while (1)
  {
  /* UART receive -> change filter mode */
  if (HAL_UART_Receive(&huart2, &rx, 1, 10) == HAL_OK)
  {
  if (rx == 'R') currentMode = RAW;
  else if (rx == 'M') currentMode = MOVING_AVERAGE;
  else if (rx == 'N') currentMode = RANDOM_NOISE;
  }

  /* Button polling */
  uint8_t buttonState = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

  if (buttonPrevState == GPIO_PIN_SET && buttonState == GPIO_PIN_RESET)
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
          NVIC_SystemReset();
  }
  buttonPrevState = buttonState;

  switch(currentState)
  {
      case STATE_INIT:
          currentState = STATE_WAIT_REQUEST;
          break;

      case STATE_WAIT_REQUEST:
          HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
          HAL_Delay(100);
          break;

      case STATE_LISTENING:
      {
          HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

          /* Read ADC */
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
          uint32_t rawValue = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);

          uint32_t maValue    = computeMovingAverage(rawValue);
          uint32_t noiseValue = addRandomNoise(rawValue);

          uint32_t raw_mv   = (rawValue   * 3300) / 4095;
          uint32_t ma_mv    = (maValue    * 3300) / 4095;
          uint32_t noise_mv = (noiseValue * 3300) / 4095;

          char msg[80];

          /* Send ONLY selected mode */
          switch(currentMode)
          {
              case RAW:
                  sprintf(msg, "RAW | %lu mV\r\n", raw_mv);
                  break;

              case MOVING_AVERAGE:
                  sprintf(msg, "MA | %lu mV\r\n", ma_mv);
                  break;

              case RANDOM_NOISE:
                  sprintf(msg, "NOISE | %lu mV\r\n", noise_mv);
                  break;
          }
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

          /* Threshold logic */
          static uint32_t highStart = 0;

          if(rawValue > POT_THRESHOLDERROR)
          {
              currentState = STATE_ERROR;
          }
          else if(rawValue > POT_THRESHOLD)
          {
              if(highStart == 0) highStart = HAL_GetTick();
              else if (HAL_GetTick() - highStart >= 5000)
              {
                  highStart = 0;
                  currentState = STATE_WARNING;
              }
          }
          else highStart = 0;

          HAL_Delay(100);
      }
      break;

      case STATE_PAUSE:
          HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
          HAL_Delay(500);
          break;

      case STATE_WARNING:
          HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
          HAL_UART_Transmit(&huart2, (uint8_t*)"WARNING\r\n", 9, HAL_MAX_DELAY);
          HAL_Delay(200);
          break;

      case STATE_ERROR:
          HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
          HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\r\n", 7, HAL_MAX_DELAY);
          HAL_Delay(200);
          break;
  }
  }
  }

/**

* @brief System Clock Configuration
  */
  void SystemClock_Config(void)
  {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|
  RCC_CLOCKTYPE_SYSCLK|
  RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
  }

/**

* @brief ADC1 Initialization Function
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  }

/**

* @brief USART2 Initialization
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
  HAL_UART_Init(&huart2);
  }

/**

* Enable DMA controller clock
  */
  static void MX_DMA_Init(void)
  {
  __HAL_RCC_DMA1_CLK_ENABLE();
  }

/**

* @brief GPIO Initialization
  */
  static void MX_GPIO_Init(void)
  {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* USER BUTTON */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LED */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI IRQ */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  }

/* Interrupt Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if (GPIO_Pin == USER_BUTTON_Pin)
buttonPressed = 1;
}

void Error_Handler(void)
{
__disable_irq();
while (1) {}
}
