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
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEAM_ID "ASI-050"
#define TELEMETRY_DELAY 1000
#define ERROR_CHECK_DELAY 100
#define MAX_TELEMETRY_STRING 256
#define EEPROM_STATE_ADDR 0x0800F000


typedef uint16_t ErrorFlags;
#define E_NONE      0x0000
#define E_BATT      0x0001
#define E_COMM_LOS  0x0002
#define E_COMM_DEL  0x0004
#define E_MEM       0x0008
#define E_SD        0x0010
#define E_FLASH     0x0020
#define E_OVER_MB   0x0040
#define E_OVER_AB   0x0080
#define E_TRAJECTORY 0x0100
#define E_PARA      0x0200
#define E_PAYLOAD   0x0400

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for stateTask */
osThreadId_t stateTaskHandle;
const osThreadAttr_t stateTask_attributes = {
  .name = "stateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for errorTask */
osThreadId_t errorTaskHandle;
const osThreadAttr_t errorTask_attributes = {
  .name = "errorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for storageTask */
osThreadId_t storageTaskHandle;
const osThreadAttr_t storageTask_attributes = {
  .name = "storageTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for stateMutex */
osMutexId_t stateMutexHandle;
const osMutexAttr_t stateMutex_attributes = {
  .name = "stateMutex"
};
/* Definitions for telemetryMutex */
osMutexId_t telemetryMutexHandle;
const osMutexAttr_t telemetryMutex_attributes = {
  .name = "telemetryMutex"
};
/* USER CODE BEGIN PV */
typedef enum {
    STATE_IDLE,
    STATE_ARMED,
    STATE_LAUNCH,
    STATE_ASCENT,
    STATE_DESCENT,
    STATE_RECOVERY
} FlightState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StateTask(void *argument);
void TelemetryTask(void *argument);
void ErrorTask(void *argument);
void StorageTask(void *argument);

/* USER CODE BEGIN PFP */
void SimulateStateActions(FlightState state);
void SaveToEEPROM(void);
void RestoreFromEEPROM(void);
void SimulateSDCardWrite(const char* data);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    __io_putchar(*ptr++);
  }
  return len;
}


static FlightState currentState = STATE_IDLE;
static uint32_t packetCount = 0;
static uint32_t missionTime = 0;
static ErrorFlags errorFlags = E_NONE;

void SimulateStateActions(FlightState state) {
    switch(state) {
        case STATE_IDLE:
            printf("IDLE: Initializing systems and calibrating sensors\r\n");
            break;
        case STATE_ARMED:
            printf("ARMED: Running pre-launch diagnostics\r\n");
            break;
        case STATE_LAUNCH:
            printf("LAUNCH: Detecting launch conditions\r\n");
            break;
        case STATE_ASCENT:
            printf("ASCENT: Monitoring flight data and motor burnout\r\n");
            break;
        case STATE_DESCENT:
            printf("DESCENT: Deploying parachutes\r\n");
            printf("Action: Main parachute deployment initiated\r\n");
            break;
        case STATE_RECOVERY:
            printf("RECOVERY: Activating recovery systems\r\n");
            printf("Action: GPS tracking activated\r\n");
            printf("Action: Audio beacon activated\r\n");
            break;
    }
}

void SaveToEEPROM(void) {
    printf("Saving state to EEPROM: State=%d, PacketCount=%lu, MissionTime=%lu\r\n",
    		currentState, packetCount, missionTime);
}

void SimulateSDCardWrite(const char* data) {
    printf("SD Card Write: %s", data);
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
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("System Starting...\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of stateMutex */
  stateMutexHandle = osMutexNew(&stateMutex_attributes);

  /* creation of telemetryMutex */
  telemetryMutexHandle = osMutexNew(&telemetryMutex_attributes);

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
  /* creation of stateTask */
  stateTaskHandle = osThreadNew(StateTask, NULL, &stateTask_attributes);

  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);

  /* creation of errorTask */
  errorTaskHandle = osThreadNew(ErrorTask, NULL, &errorTask_attributes);

  /* creation of storageTask */
  storageTaskHandle = osThreadNew(StorageTask, NULL, &storageTask_attributes);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StateTask */
/**
  * @brief  Function implementing the stateTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StateTask */
void StateTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	  uint32_t lastButtonPress = 0;
	  printf("State Task Started\r\n");

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	  printf("a");

      if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
          uint32_t currentTime = HAL_GetTick();
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
          printf("c");

          if(currentTime - lastButtonPress > 500) {
              osMutexAcquire(stateMutexHandle, osWaitForever);
              printf("d");

              currentState = (currentState + 1) % 6;

              printf("Button pressed - New State: %d\r\n", currentState);

              SimulateStateActions(currentState);

              SaveToEEPROM();

              osMutexRelease(stateMutexHandle);
              lastButtonPress = currentTime;
          }
      }
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin,SET);
	  printf("b");

      osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TelemetryTask */
void TelemetryTask(void *argument)
{
  /* USER CODE BEGIN TelemetryTask */
    char telemetryString[MAX_TELEMETRY_STRING];

  /* Infinite loop */
  for(;;)
  {
	  printf("e");
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      osMutexAcquire(telemetryMutexHandle, osWaitForever);

      float altitude = 100.0f;
      float pressure = 101.325f;
      float temp = 25.0f;
      float voltage = 11.8f;

      int len = snprintf(telemetryString, MAX_TELEMETRY_STRING,
              "#%s,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%lu,%.6f,%.6f,%.2f,%d,%.2f,%.2f,%d,%.2f,%04X$\r\n",
              TEAM_ID, missionTime, packetCount, altitude, pressure, temp, voltage,
              0UL, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f,
              currentState, temp, errorFlags);
      printf("tele");

      if (len > 0) {
          HAL_UART_Transmit(&huart2, (uint8_t*)telemetryString, len, HAL_MAX_DELAY);

          printf("Packet %lu sent\r\n", packetCount);

          SimulateSDCardWrite(telemetryString);

          packetCount++;
          missionTime++;
      }
      printf("f");

      osMutexRelease(telemetryMutexHandle);
      printf("g");


      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      printf("h");


      osDelay(TELEMETRY_DELAY);
      printf("i");

  }
  /* USER CODE END TelemetryTask */
}

/* USER CODE BEGIN Header_ErrorTask */
/**
* @brief Function implementing the errorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ErrorTask */
void ErrorTask(void *argument)
{
  /* USER CODE BEGIN ErrorTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  printf("h");

      float voltage = 11.8f;
      if(voltage < 11.0f) {
    	  printf("i");

          errorFlags |= E_BATT;
      } else {
          errorFlags &= ~E_BATT;
      }

      osDelay(ERROR_CHECK_DELAY);
  }
  /* USER CODE END ErrorTask */
}

/* USER CODE BEGIN Header_StorageTask */
/**
* @brief Function implementing the storageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StorageTask */
void StorageTask(void *argument)
{
  /* USER CODE BEGIN StorageTask */
  /* Infinite loop */
  for(;;)
  {
	printf("1");
    osDelay(10);
  }
  /* USER CODE END StorageTask */
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
