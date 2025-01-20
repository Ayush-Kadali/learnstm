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
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_TELEM_LENGTH 256
#define TEAM_ID "2024-ASI-050"
#define TELEM_RATE_MS 100  // 10Hz telemetry rate
#define ACK_TIMEOUT_MS 5000
#define FLASH_SAVE_INTERVAL_MS 5000
#define BUTTON_DEBOUNCE_MS 300
#define STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for stateManagerTas */
osThreadId_t stateManagerTaskHandle;
const osThreadAttr_t stateManagerTask_attributes = {
  .name = "stateManagerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for errorMonitorTas */
osThreadId_t errorMonitorTaskHandle;
const osThreadAttr_t errorMonitorTask_attributes = {
  .name = "errorMonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for flashManagerTas */
osThreadId_t flashManagerTaskHandle;
const osThreadAttr_t flashManagerTask_attributes = {
  .name = "flashManagerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for systemDataMutex */
osMutexId_t systemDataMutexHandle;
const osMutexAttr_t systemDataMutex_attributes = {
  .name = "systemDataMutex"
};
/* USER CODE BEGIN PV */
typedef enum {
    STATE_IDLE,
    STATE_ARMED,
    STATE_LAUNCH,
    STATE_ASCENT,
    STATE_DESCENT,
    STATE_RECOVERY
} SystemState;

// Error Flags
typedef uint16_t ErrorFlags;
#define E_NONE      	0x0000
#define E_BATT      	0x0001
#define E_COMM_LOS  	0x0002
#define E_COMM_DEL  	0x0004
#define E_MEM       	0x0008
#define E_SD        	0x0010
#define E_FLASH     	0x0020
#define E_OVER_MB   	0x0040
#define E_OVER_AB   	0x0080
#define E_TRAJECTORY 	0x0100
#define E_PARA      	0x0200
#define E_PAYLOAD   	0x0400


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void TelemetryTask(void *argument);
void SensorTask(void *argument);
void StateManagerTask(void *argument);
void ErrorMonitorTask(void *argument);
void FlashManagerTask(void *argument);

/* USER CODE BEGIN PFP */
void InitializeSystem(void);
void SaveToFlash(void);
void RestoreFromFlash(void);
void SimulateSDCardWrite(const char* data);
ErrorFlags CheckSystemErrors(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


void DebugPrint(const char* message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
}


void LogError(const char* message) {
    char errorMsg[100];
    snprintf(errorMsg, sizeof(errorMsg), "ERROR: %s [Time: %lu]\r\n",
             message, HAL_GetTick());
    HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), 100);
}

typedef struct {
    SystemState currentState;
    uint32_t packetCount;
    uint32_t missionTime;
    ErrorFlags errorFlags;
    float altitude;
    float pressure;
    float tempAB;
    float voltage;
    float gnssTime;
    float gnssLat;
    float gnssLong;
    float gnssAlt;
    uint8_t gnssSats;
    float accelData[3];
    float gyroData[3];
    float tempMB;
    uint8_t ackReceived;
    uint32_t lastAckTime;
    uint32_t lastButtonPress;  // Added for button debouncing

} SystemData;

osMutexId_t systemDataMutex;

SystemData systemData = {0};


void SimulateSDCardWrite(const char* data) {
    printf("SD Card Write: %s", data);
}

ErrorFlags CheckSystemErrors(void) {
    ErrorFlags flags = E_NONE;

    // Check various error conditions
    if(systemData.voltage < 3.3f) flags |= E_BATT;
    // Add other error checks

    return flags;
}

//void InitializeSystem(void) {
//    DebugPrint("Starting system initialization...\r\n");
//
//    // Initialize mutex
//    systemDataMutex = osMutexNew(NULL);
//    if (systemDataMutex == NULL) {
//        DebugPrint("Failed to create mutex!\r\n");
//        return;
//    }
//
//    // Initialize system data
//    memset(&systemData, 0, sizeof(SystemData));
//    systemData.currentState = STATE_IDLE;
//
//    // Create tasks with error checking
//    const osThreadAttr_t normalPriority = {
//        .priority = osPriorityNormal,
//        .stack_size = 512
//    };
//    const osThreadAttr_t highPriority = {
//        .priority = osPriorityHigh,
//        .stack_size = 512
//    };
//
//    sensorTaskHandle = osThreadNew(SensorTask, NULL, &highPriority);
//    if (sensorTaskHandle == NULL) {
//        DebugPrint("Failed to create sensor task!\r\n");
//        return;
//    }
//
//    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &normalPriority);
//    if (telemetryTaskHandle == NULL) {
//        DebugPrint("Failed to create telemetry task!\r\n");
//        return;
//    }
//
//    stateManagerTaskHandle = osThreadNew(StateManagerTask, NULL, &highPriority);
//    if (stateManagerTaskHandle == NULL) {
//        DebugPrint("Failed to create state manager task!\r\n");
//        return;
//    }
//
//    errorMonitorTaskHandle = osThreadNew(ErrorMonitorTask, NULL, &normalPriority);
//    if (errorMonitorTaskHandle == NULL) {
//        DebugPrint("Failed to create error monitor task!\r\n");
//        return;
//    }
//
//    flashManagerTaskHandle = osThreadNew(FlashManagerTask, NULL, &normalPriority);
//    if (flashManagerTaskHandle == NULL) {
//        DebugPrint("Failed to create flash manager task!\r\n");
//        return;
//    }
//
//    DebugPrint("System initialization complete!\r\n");
//}





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

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of systemDataMutex */
  systemDataMutexHandle = osMutexNew(&systemDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  if (systemDataMutexHandle == NULL) {
    Error_Handler();
  }
  memset(&systemData, 0, sizeof(SystemData));
  systemData.currentState = STATE_IDLE;
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

  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);

  /* creation of stateManagerTas */
  stateManagerTaskHandle = osThreadNew(StateManagerTask, NULL, &stateManagerTask_attributes);

  /* creation of errorMonitorTas */
  errorMonitorTaskHandle = osThreadNew(ErrorMonitorTask, NULL, &errorMonitorTask_attributes);

  /* creation of flashManagerTas */
  flashManagerTaskHandle = osThreadNew(FlashManagerTask, NULL, &flashManagerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (!defaultTaskHandle || !telemetryTaskHandle || !sensorTaskHandle ||
      !stateManagerTaskHandle || !errorMonitorTaskHandle || !flashManagerTaskHandle) {
    Error_Handler();
  }

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
	  char telemString[MAX_TELEM_LENGTH];
	  uint32_t last_wake_time = osKernelGetTickCount();

	  DebugPrint("Telemetry task started\r\n");

  /* Infinite loop */
  for(;;)
  {
	    osStatus_t status = osMutexAcquire(systemDataMutexHandle, 100);
	    if (status == osOK) {
	      // Format telemetry string
	      snprintf(telemString, MAX_TELEM_LENGTH,
	              "#%s,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.2f,%.2f,%d,%.2f,%u$\r\n",
	              TEAM_ID,
	              systemData.missionTime,
	              systemData.packetCount,
	              systemData.altitude,
	              systemData.pressure,
	              systemData.tempAB,
	              systemData.voltage,
	              systemData.gnssTime,
	              systemData.gnssLat,
	              systemData.gnssLong,
	              systemData.gnssAlt,
	              systemData.gnssSats,
	              systemData.accelData[0],
	              systemData.gyroData[0],
	              systemData.currentState,
	              systemData.tempMB,
	              systemData.errorFlags);

	      // Send via UART with error checking
	      if (HAL_UART_Transmit(&huart2, (uint8_t*)telemString, strlen(telemString), 100) != HAL_OK) {
	        systemData.errorFlags |= E_COMM_DEL;
	      }

	      systemData.packetCount++;
	      systemData.missionTime = HAL_GetTick();

	      osMutexRelease(systemDataMutexHandle);
	    }

	    // Use absolute timing for consistent period
	    osDelayUntil(last_wake_time + TELEM_RATE_MS);
	    last_wake_time = osKernelGetTickCount();
  }
  /* USER CODE END TelemetryTask */
}

/* USER CODE BEGIN Header_SensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTask */
void SensorTask(void *argument)
{
  /* USER CODE BEGIN SensorTask */
	  uint32_t last_wake_time = osKernelGetTickCount();

	  DebugPrint("Sensor task started\r\n");

  /* Infinite loop */
  for(;;)
  {
	    osStatus_t status = osMutexAcquire(systemDataMutexHandle, 50);
	    if (status == osOK) {
	      // Simulate sensor readings
	      systemData.altitude += 0.1f;
	      systemData.pressure = 101325.0f - (systemData.altitude * 12.0f);
	      systemData.tempAB = 25.0f;
	      systemData.voltage = 3.7f; // Simulate battery voltage

	      osMutexRelease(systemDataMutexHandle);
	    }

	    // Use absolute timing
	    osDelayUntil(last_wake_time + 10);  // 100Hz
	    last_wake_time = osKernelGetTickCount();

  }
  /* USER CODE END SensorTask */
}

/* USER CODE BEGIN Header_StateManagerTask */
/**
* @brief Function implementing the stateManagerTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateManagerTask */
void StateManagerTask(void *argument)
{
  /* USER CODE BEGIN StateManagerTask */
    DebugPrint("State manager task started\r\n");

    const char* stateNames[] = {
        "IDLE", "ARMED", "LAUNCH", "ASCENT", "DESCENT", "RECOVERY"
    };

  /* Infinite loop */
  for(;;)
  {
      if (osMutexAcquire(systemDataMutex, 100) == osOK) {
          // Check button press with debouncing
          if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
              uint32_t currentTime = HAL_GetTick();

              if (currentTime - systemData.lastButtonPress > BUTTON_DEBOUNCE_MS) {
                  SystemState nextState = systemData.currentState;

                  // State transition logic
                  switch(systemData.currentState) {
                      case STATE_IDLE:
                          nextState = STATE_ARMED;
                          break;
                      case STATE_ARMED:
                          nextState = STATE_LAUNCH;
                          break;
                      case STATE_LAUNCH:
                          nextState = STATE_ASCENT;
                          break;
                      case STATE_ASCENT:
                          nextState = STATE_DESCENT;
                          break;
                      case STATE_DESCENT:
                          nextState = STATE_RECOVERY;
                          break;
                      case STATE_RECOVERY:
                          nextState = STATE_IDLE;
                          break;
                  }

                  if (nextState != systemData.currentState) {
                      char stateMsg[50];
                      snprintf(stateMsg, sizeof(stateMsg),
                              "State changed to: %s\r\n",
                              stateNames[nextState]);
                      DebugPrint(stateMsg);
                      systemData.currentState = nextState;
                  }

                  systemData.lastButtonPress = currentTime;
              }
          }

          osMutexRelease(systemDataMutex);
      }
      osDelay(10);
  }
  /* USER CODE END StateManagerTask */
}

/* USER CODE BEGIN Header_ErrorMonitorTask */
/**
* @brief Function implementing the errorMonitorTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ErrorMonitorTask */
void ErrorMonitorTask(void *argument)
{
  /* USER CODE BEGIN ErrorMonitorTask */
  /* Infinite loop */
  for(;;)
  {
      osMutexAcquire(systemDataMutex, osWaitForever);
      systemData.errorFlags = CheckSystemErrors();
      osMutexRelease(systemDataMutex);

      osDelay(100);  // Check errors every 100ms
  }
  /* USER CODE END ErrorMonitorTask */
}

/* USER CODE BEGIN Header_FlashManagerTask */
/**
* @brief Function implementing the flashManagerTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FlashManagerTask */
void FlashManagerTask(void *argument)
{
  /* USER CODE BEGIN FlashManagerTask */
  /* Infinite loop */
  for(;;)
  {
      osMutexAcquire(systemDataMutex, osWaitForever);
//      SaveToFlash();  // Save critical data periodically
      osMutexRelease(systemDataMutex);

      osDelay(FLASH_SAVE_INTERVAL_MS);
  }
  /* USER CODE END FlashManagerTask */
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
