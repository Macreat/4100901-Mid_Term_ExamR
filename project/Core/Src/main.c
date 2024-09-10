/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t TL[] = "Turn LEFT\n\r";
uint8_t TR[] = "Turn RIGHT\n\r";

uint8_t TLL[] = "Turn LEFT indefinitely\n\r";
uint8_t TRR[] = "Turn RIGHT indefinitely\n\r";
uint8_t stac[] = "Stationary mode \n\r";


volatile uint8_t flag_left = 0;
volatile uint8_t flag_right = 0;
volatile uint8_t timmingb1= 0;
volatile uint8_t timmingb2= 0;
volatile uint8_t localtim = 0;
uint32_t last_button_press_left = 0;
uint32_t last_button_press_right = 0;
const uint32_t debounceDelay = 50; // 50ms debounce delay
const uint32_t doublePressTime = 500; // max time for double press


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	      // Logic to implement toggle led's
	  //  first on turn led
	         if (flag_left) {
	             flag_left = 0;
	             if (timmingb1 == 1) { // if b1 is pressed
	                 for (int i = 0; i < 3; i++) {
	                     HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
	                     HAL_Delay(125); // 4Hz = 125ms encendido
	                     HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
	                     HAL_Delay(125);
	                 }
	                 HAL_UART_Transmit(&huart2, TL, sizeof(TL) - 1, 100);
	             } else if (timmingb1 >= 2) { // Si se presiona más de 2 veces
	                 HAL_UART_Transmit(&huart2, TLL, sizeof(TLL) - 1, 100);
	                 while (1) { //
	                     HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
	                     HAL_Delay(125);
	                     HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
	                     HAL_Delay(125);
	                     if (flag_left || flag_right) { // Press any button to close loop
	                         break;
	                     }
	                 }
	             }
	         }

	         //  turn right
	         if (flag_right) {
	             flag_right = 0;
	             if (timmingb2 == 1) {
	                 for (int i = 0; i < 3; i++) {
	                     HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
	                     HAL_Delay(125); // 4Hz = 125ms
	                     HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
	                     HAL_Delay(125);
	                 }
	                 HAL_UART_Transmit(&huart2, TR, sizeof(TR) - 1, 100);
	             } else if (timmingb2 >= 2) {
	                 HAL_UART_Transmit(&huart2, TRR, sizeof(TRR) - 1, 100);
	                 while (1) {
	                     HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
	                     HAL_Delay(125);
	                     HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
	                     HAL_Delay(125);
	                     if (flag_left || flag_right) {
	                         break;
	                     }
	                 }
	             }
	         }

	         // stationary mode
	         if (timmingb1 >= 1 && timmingb2 >= 1 &&
	             (HAL_GetTick() - last_button_press_left < doublePressTime) &&
	             (HAL_GetTick() - last_button_press_right < doublePressTime)) {

	             HAL_UART_Transmit(&huart2, stac, sizeof(stac) - 1, 100);

	             while (1) {
	                 HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
	                 HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
	                 HAL_Delay(125);
	                 HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
	                 HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
	                 HAL_Delay(125);
	                 if (flag_left || flag_right) { // press any button to close loop

	                     break;
	                 }
	             }

	             // Reset to contrarrest debounce
	             timmingb1 = 0;
	             timmingb2 = 0;


	         }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Function used for implement the interruption from the system

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick(); // get actual time on ms

    if (GPIO_Pin == B1_Pin) { // Button 1 is pressed
        if (current_time - last_button_press_left < 500) {
            timmingb1++; // Increase counter button 1
        } else {
            timmingb1 = 1; // Reset counter if time is 500ms or plus
        }
        last_button_press_left = current_time;
        flag_left = 1;
    }

    if (GPIO_Pin == B2_Pin) { // button 2 is pressed
        if (current_time - last_button_press_right < 500) {
            timmingb2++;
        } else {
            timmingb2 = 1; // reset counter
        }
        last_button_press_right = current_time;
        flag_right = 1;
    }
}
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

#ifdef  USE_FULL_ASSERT
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
