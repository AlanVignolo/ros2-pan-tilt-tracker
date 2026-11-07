/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t rx_byte;                 // el byte que acaba de llegar
static char rx_buf[32];                 // la línea que se está armando
static uint8_t rx_len = 0;              // cuántos caracteres lleva
static char linea[32];                  // la última línea completa
static volatile uint8_t linea_lista = 0;

#define SERVO_US_MIN   500   // pulso para 0°
#define SERVO_US_MAX   2000   // pulso para 180°
#define TILT_MIN       0      // décimas de grado
#define TILT_MAX       1800

#define PASOS_POR_VUELTA  1600    // 200 pasos x 8 micropasos
#define PAN_MIN           -900   // décimas de grado
#define PAN_MAX           900

static volatile int32_t pan_pos = 0;   // pasos dados desde el arranque
static volatile int32_t pan_obj = 0;   // pasos a los que hay que ir
static uint8_t pul_alto = 0;
static int8_t dir_actual = 0;

#define FC_ACTIVO   GPIO_PIN_RESET   // botones NO: apretado = 0
static uint8_t bloqueo_pos = 0;   // se tocó FC_POS, no avanzar hacia +
static uint8_t bloqueo_neg = 0;   // se tocó FC_NEG, no avanzar hacia −

// Devuelve 1 si el interruptor está en ON (pin a GND = 0), 0 si está en OFF.
// Es una macro: el compilador reemplaza HABILITADO() por la lectura del pin
// en cada lugar donde aparece. Evita repetir la misma línea larga 4 veces.
#define HABILITADO()  (HAL_GPIO_ReadPin(HABILITAR_GPIO_Port, HABILITAR_Pin) == GPIO_PIN_RESET)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart ->Instance == USART2)
	{
		if (rx_byte == '\r' || rx_byte == '\n')
		{
			if (rx_len > 0 && !linea_lista)
			{
				rx_buf[rx_len] = '\0';
				memcpy(linea, rx_buf, rx_len +1);
				linea_lista = 1;
			}
			rx_len = 0;
		}
		else if (rx_len < sizeof(rx_buf) -1)
		{
			rx_buf[rx_len++] = rx_byte;
		}
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
}

static void servo_tilt(long decimas)
{
	uint32_t us = SERVO_US_MIN + (uint32_t) decimas * (SERVO_US_MAX - SERVO_US_MIN) / 1800;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance != TIM2) return;

	if (pul_alto)
	{
	HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
	pul_alto = 0;
	return;
	}

	// Interruptor en OFF: no se da ningún paso.
	// Además se cancela el objetivo, para que al volver a ON
	// el motor no salte a una orden vieja que quedó pendiente.
	if (!HABILITADO())
	{
	pan_obj = pan_pos;
	return;
	}

	int32_t error = pan_obj - pan_pos;
	if (error == 0) return;

	int8_t dir = (error > 0) ? 1 : -1;

	if (dir > 0 && HAL_GPIO_ReadPin(FC_POS_GPIO_Port, FC_POS_Pin) == FC_ACTIVO) bloqueo_pos = 1;
	if (dir < 0 && HAL_GPIO_ReadPin(FC_NEG_GPIO_Port, FC_NEG_Pin) == FC_ACTIVO) bloqueo_neg = 1;

	if ((dir > 0 && bloqueo_pos) || (dir < 0 && bloqueo_neg))
	{
	pan_obj = pan_pos;
	return;
	}

	if (dir != dir_actual)
	{
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
					  (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	dir_actual = dir;
	return;
	}

	HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET);
	pul_alto = 1;
	pan_pos += dir;

	if (dir > 0) bloqueo_neg = 0;
	if (dir < 0) bloqueo_pos = 0;
}

static void procesar_linea(const char *s)
{
	char resp[64];
	long pan, tilt;

	if (strcmp(s, "S") == 0)
	{
	    int fn = (HAL_GPIO_ReadPin(FC_NEG_GPIO_Port, FC_NEG_Pin) == FC_ACTIVO);
	    int fp = (HAL_GPIO_ReadPin(FC_POS_GPIO_Port, FC_POS_Pin) == FC_ACTIVO);
		long pos = (long)pan_pos * 3600 / PASOS_POR_VUELTA;
		long obj = (long)pan_obj * 3600 / PASOS_POR_VUELTA;
	    snprintf(resp, sizeof(resp), "ST POS%ld OBJ%ld FN%d FP%d BN%d BP%d EN%d\r\n",
	             pos, obj, fn, fp, bloqueo_neg, bloqueo_pos, HABILITADO());
	    HAL_UART_Transmit(&huart2, (uint8_t *)resp, strlen(resp), 100);
		return;
	}

	// Interruptor en OFF: no se mueve nada. Se responde OFF
	// para que la Raspberry sepa que la orden no se ejecutó.
	if (!HABILITADO())
	{
	snprintf(resp, sizeof(resp), "OFF\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)resp, strlen(resp), 100);
	return;
	}

	if(sscanf(s, "P%ld T%ld", &pan, &tilt) == 2)
	{
		if (tilt < TILT_MIN) tilt = TILT_MIN;
		if (tilt > TILT_MAX) tilt = TILT_MAX;
		servo_tilt(tilt);

	    if (pan < PAN_MIN) pan = PAN_MIN;
	    if (pan > PAN_MAX) pan = PAN_MAX;
	    pan_obj = (int32_t)pan * PASOS_POR_VUELTA / 3600;

		snprintf(resp, sizeof(resp), "OK P%ld T%ld\r\n", pan, tilt);
	}
	else
	{
		snprintf(resp, sizeof(resp), "ERR\r\n");
	}
	HAL_UART_Transmit(&huart2, (uint8_t *)resp, strlen(resp), 100);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  uint32_t t_led = HAL_GetTick();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (linea_lista)
	  {
		  procesar_linea(linea);
		  linea_lista = 0;
	  }
	// LD2: apagado = motores deshabilitados
	//      fijo     = algún fin de carrera activo
	//      parpadea = todo normal
	if (!HABILITADO())
	{
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	else if (HAL_GPIO_ReadPin(FC_NEG_GPIO_Port, FC_NEG_Pin) == FC_ACTIVO ||
			 HAL_GPIO_ReadPin(FC_POS_GPIO_Port, FC_POS_Pin) == FC_ACTIVO)
	{
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}
	else if (HAL_GetTick() - t_led >= 500)
	{
	  t_led = HAL_GetTick();
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|PUL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin PUL_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|PUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HABILITAR_Pin */
  GPIO_InitStruct.Pin = HABILITAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HABILITAR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FC_NEG_Pin */
  GPIO_InitStruct.Pin = FC_NEG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FC_NEG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FC_POS_Pin */
  GPIO_InitStruct.Pin = FC_POS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FC_POS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
