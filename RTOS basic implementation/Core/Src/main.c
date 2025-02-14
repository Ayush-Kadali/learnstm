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
#include <stdio.h>

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
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bootTask */
osThreadId_t bootTaskHandle;
const osThreadAttr_t bootTask_attributes = {
  .name = "bootTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for idleTask */
osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
  .name = "idleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ascentTask */
osThreadId_t ascentTaskHandle;
const osThreadAttr_t ascentTask_attributes = {
  .name = "ascentTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for descentTask */
osThreadId_t descentTaskHandle;
const osThreadAttr_t descentTask_attributes = {
  .name = "descentTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for recoveryTask */
osThreadId_t recoveryTaskHandle;
const osThreadAttr_t recoveryTask_attributes = {
  .name = "recoveryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint32_t buttonPressTime = 0; // Tracks button press time
volatile uint8_t buttonPressed = 0;   // Tracks if button was pressed

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartUartTask(void *argument);
void StartBootTask(void *argument);
void StartIdleTask(void *argument);
void StartAscentTask(void *argument);
void StartDescentTask(void *argument);
void StartRecoveryTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
    BOOT,
    IDLE,
    TELEMETRY,
    ASCENT,
    DESCENT,
    RECOVERY
} SystemState_t;

volatile SystemState_t systemState = BOOT;
volatile uint32_t buttonPressStartTime = 0;
volatile uint8_t buttonPressCount = 0;


int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


uint8_t isButtonPressed()
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}

void handleButtonPress()
{
    if (isButtonPressed())
    {
        if (!buttonPressed) // If button was not already pressed
        {
            buttonPressTime = HAL_GetTick();
            buttonPressed = 1;
        }
    }
    else
    {
        if (buttonPressed && HAL_GetTick() - buttonPressTime >= 2000) // 2-second press
        {
            if (systemState == BOOT)
            {
                systemState = IDLE; // Transition from BOOT to IDLE
            }
        }
        else if (buttonPressed && HAL_GetTick() - buttonPressTime < 2000) // Single press
        {
            if (systemState == IDLE)
            {
                systemState = TELEMETRY;
            }
            else if (systemState == TELEMETRY)
            {
                systemState = ASCENT;
            }
            else if (systemState == ASCENT)
            {
                systemState = DESCENT;
            }
            else if (systemState == DESCENT)
            {
                systemState = RECOVERY;
            }
        }
        buttonPressed = 0; // Reset button state
    }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of bootTask */
  bootTaskHandle = osThreadNew(StartBootTask, NULL, &bootTask_attributes);

  /* creation of idleTask */
  idleTaskHandle = osThreadNew(StartIdleTask, NULL, &idleTask_attributes);

  /* creation of ascentTask */
  ascentTaskHandle = osThreadNew(StartAscentTask, NULL, &ascentTask_attributes);

  /* creation of descentTask */
  descentTaskHandle = osThreadNew(StartDescentTask, NULL, &descentTask_attributes);

  /* creation of recoveryTask */
  recoveryTaskHandle = osThreadNew(StartRecoveryTask, NULL, &recoveryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
	  const char *telemetryMessage = "Telemetry Data\r\n";

  /* Infinite loop */
  for(;;)
  {
	    handleButtonPress();

	    if (systemState == TELEMETRY) // Telemetry state
	    {
	      printf(telemetryMessage);
	      osDelay(1000); // Send telemetry every second
	    }
	    else
	    {
	      osDelay(100); // Check periodically
	    }

  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartBootTask */
/**
* @brief Function implementing the bootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBootTask */
void StartBootTask(void *argument)
{
  /* USER CODE BEGIN StartBootTask */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Example: Reset an LED
    printf("Boot State\r\n");

	HAL_Delay(100); // Simulate boot process
    printf("Initializing sensors and actuators \r\n");

    handleButtonPress();

    if (systemState == IDLE)
    {
      osThreadTerminate(NULL); // Terminate Boot Task after transition
    }

    osDelay(100);

  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartBootTask */
}

/* USER CODE BEGIN Header_StartIdleTask */
/**
* @brief Function implementing the idleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIdleTask */
void StartIdleTask(void *argument)
{
  /* USER CODE BEGIN StartIdleTask */
  /* Infinite loop */
  for(;;)
  {
	  handleButtonPress();

	      if (systemState == IDLE) // Idle state
	      {
	        printf("Idle State: All sensors and actuators initialized\r\n");
	        StartUartTask(NULL); // Start UART task to send telemetry
	      }

	      if (systemState != IDLE) // Exit Idle state
	      {
	        osThreadTerminate(NULL); // Terminate Idle Task
	      }

	      osDelay(100);


  }
  /* USER CODE END StartIdleTask */
}

/* USER CODE BEGIN Header_StartAscentTask */
/**
* @brief Function implementing the ascentTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAscentTask */
void StartAscentTask(void *argument)
{
  /* USER CODE BEGIN StartAscentTask */
  /* Infinite loop */
  for(;;)
  {
	    handleButtonPress();

	    if (systemState == ASCENT)
	    {
	      printf("Ascent Telemetry Data\r\n");
	      osDelay(1000);
	    }
	    else
	    {
	      osDelay(100);
	    }
  }
  /* USER CODE END StartAscentTask */
}

/* USER CODE BEGIN Header_StartDescentTask */
/**
* @brief Function implementing the descentTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDescentTask */
void StartDescentTask(void *argument)
{
  /* USER CODE BEGIN StartDescentTask */


  /* Infinite loop */
  for(;;)

  {
	    handleButtonPress();

	    if (systemState == DESCENT)
	    {
	      // Simulate actions during descent
	      HAL_UART_Transmit(&huart2, (uint8_t *)"Release drogue parachute\r\n", 26, HAL_MAX_DELAY);
	      osDelay(2000);

	      HAL_UART_Transmit(&huart2, (uint8_t *)"Payload ejection\r\n", 18, HAL_MAX_DELAY);
	      osDelay(2000);

	      HAL_UART_Transmit(&huart2, (uint8_t *)"Deploy main parachute\r\n", 24, HAL_MAX_DELAY);
	      osDelay(2000);

	      // Continue sending descent telemetry without waiting for actions
	      while (systemState == DESCENT)
	      {
	        printf("Descent Telemetry Data\r\n");
	        osDelay(1000); // Send telemetry every second
	      }
	    }
	    else
	    {
	      osDelay(100);
	    }
  }
  /* USER CODE END StartDescentTask */
}

/* USER CODE BEGIN Header_StartRecoveryTask */
/**
* @brief Function implementing the recoveryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecoveryTask */
void StartRecoveryTask(void *argument)
{
  /* USER CODE BEGIN StartRecoveryTask */
  /* Infinite loop */
  for(;;)
  {
	    handleButtonPress();

	    if (systemState == RECOVERY)
	    {
	      HAL_UART_Transmit(&huart2, (uint8_t *)"Shortened Telemetry\r\n", 22, HAL_MAX_DELAY);

	      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	      osDelay(1000);
	    }
	    else
	    {
	      osDelay(100);
	    }
  }
  /* USER CODE END StartRecoveryTask */
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
