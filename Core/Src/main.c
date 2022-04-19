/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIGNAL_BUTTON_PRESS 1
#define SIGNAL_UART 1
#define SIGNAL_OBJECT_DETECT 1
#define SIGNAL_MOTOR 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId ImuTaskHandle;
osThreadId GpsTaskHandle;
osThreadId KFTaskHandle;
osThreadId RadarTaskHandle;
osThreadId UartTaskHandle;
osThreadId ExButtonIntTaskHandle;
osThreadId LcdTaskHandle;
osThreadId MotorTaskHandle;
osThreadId MainTaskHandle;
osThreadId ObjectDetectTasHandle;
osMessageQId UartQueueHandle;
osMessageQId ImuQueueHandle;
osMessageQId GpsQueueHandle;
osMutexId PrintMtxHandle;
/* USER CODE BEGIN PV */
uint8_t Rx_byte;
char Rx_indx, Transfer_cplt, Rx_Buffer[100];
uint8_t msg[30] = {'\0'};

typedef struct
{
	uint32_t timestamp;
	float lin_acc;
	float ang_vel;

} ImuData;


typedef struct
{
	uint32_t timestamp;
	float x;
	float y;
	float z;

} GpsData;

typedef struct
{
	uint32_t timestamp;
	float coef;
} UartData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartImuTask(void const * argument);
void StartGpsTask(void const * argument);
void StartKFTask(void const * argument);
void StartRadarTask(void const * argument);
void StartUartTask(void const * argument);
void StartExButtonIntTask(void const * argument);
void StartLcdTask(void const * argument);
void StartMotorTask(void const * argument);
void StartMainTask(void const * argument);
void StartObjectDetectTask(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
// UART callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of PrintMtx */
  osMutexDef(PrintMtx);
  PrintMtxHandle = osMutexCreate(osMutex(PrintMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of UartQueue */
  osMessageQDef(UartQueue, 1, UartData);
  UartQueueHandle = osMessageCreate(osMessageQ(UartQueue), NULL);

  /* definition and creation of ImuQueue */
  osMessageQDef(ImuQueue, 1, ImuData);
  ImuQueueHandle = osMessageCreate(osMessageQ(ImuQueue), NULL);

  /* definition and creation of GpsQueue */
  osMessageQDef(GpsQueue, 1, GpsData);
  GpsQueueHandle = osMessageCreate(osMessageQ(GpsQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, StartImuTask, osPriorityLow, 0, 128);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of GpsTask */
  osThreadDef(GpsTask, StartGpsTask, osPriorityLow, 0, 128);
  GpsTaskHandle = osThreadCreate(osThread(GpsTask), NULL);

  /* definition and creation of KFTask */
  osThreadDef(KFTask, StartKFTask, osPriorityLow, 0, 128);
  KFTaskHandle = osThreadCreate(osThread(KFTask), NULL);

  /* definition and creation of RadarTask */
  osThreadDef(RadarTask, StartRadarTask, osPriorityLow, 0, 128);
  RadarTaskHandle = osThreadCreate(osThread(RadarTask), NULL);

  /* definition and creation of UartTask */
  osThreadDef(UartTask, StartUartTask, osPriorityLow, 0, 128);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

  /* definition and creation of ExButtonIntTask */
  osThreadDef(ExButtonIntTask, StartExButtonIntTask, osPriorityHigh, 0, 128);
  ExButtonIntTaskHandle = osThreadCreate(osThread(ExButtonIntTask), NULL);

  /* definition and creation of LcdTask */
  osThreadDef(LcdTask, StartLcdTask, osPriorityLow, 0, 128);
  LcdTaskHandle = osThreadCreate(osThread(LcdTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityAboveNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityNormal, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of ObjectDetectTas */
  osThreadDef(ObjectDetectTas, StartObjectDetectTask, osPriorityHigh, 0, 128);
  ObjectDetectTasHandle = osThreadCreate(osThread(ObjectDetectTas), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YellowLed_Pin|RedLed_Pin|GreenLed_Pin|BlueLed_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : YellowLed_Pin RedLed_Pin GreenLed_Pin BlueLed_Pin */
  GPIO_InitStruct.Pin = YellowLed_Pin|RedLed_Pin|GreenLed_Pin|BlueLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ExButton_Pin */
  GPIO_InitStruct.Pin = ExButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ExButton_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// overwrite the HAL_GPIO_EXTI_Callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == ExButton_Pin) {
  	osSignalSet(ExButtonIntTaskHandle, SIGNAL_BUTTON_PRESS);
  }
  if(GPIO_Pin == B1_Pin) {
  	osSignalSet(ExButtonIntTaskHandle, SIGNAL_BUTTON_PRESS);
  }
}

// interrupt callback method - when the data reception is complete, this is called
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	 // current UART
	uint8_t i;

	if (huart->Instance == USART2) {
  	// Clear Rx_Buffer prior to use
  	if (Rx_indx == 0) {
    	// turn on the yellow led
    	HAL_GPIO_WritePin(YellowLed_GPIO_Port, YellowLed_Pin, GPIO_PIN_SET);
    	EmptyBuffer(Rx_Buffer);
  	}

  	// check for carriage return (ASCII: 13 == \r)
  	if (Rx_byte != 13) {
  		Rx_Buffer[Rx_indx++] = Rx_byte; // add data to Rx_Buffer
  	} else {
  		Rx_indx = 0;
  		Transfer_cplt = 1; // transfer complete, data is ready

  		// LED trigger phrase
  		if (strcmp(Rx_Buffer, "faster") == 0) {
  			sprintf(msg, "Accelerating!");
  		} else if (strcmp(Rx_Buffer, "slower")  == 0) {
  			sprintf(msg, "Decelerating!");
  		} else {
  			sprintf(msg, "Unknown command.");
  		}

  		// send to UART
  		HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  		HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  		HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
    	EmptyBuffer(msg);

    	// turn off the yellow led
    	HAL_GPIO_WritePin(YellowLed_GPIO_Port, YellowLed_Pin, GPIO_PIN_RESET);
  	}

  	// activate UART
  	HAL_UART_Receive_IT(&huart2, &Rx_byte, 1);
  	// send to UART
  	HAL_UART_Transmit(&huart2, &Rx_byte, 1, 100);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartImuTask */
/**
  * @brief  Function implementing the ImuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "IMU GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);

  /* Infinite loop */
  for(;;)
  {
  	// Get the RTOS kernel tick count
  	uint32_t  t =  osKernelSysTick();

  	// pseudo-measurements -  we assume that the the IMU gives perfect measurements that indicate that the vehicle moves in a circle with constant speeds
  	float imu_lin_acc = 0.1;
  	float imu_ang_vel = 0.2;
  	ImuData imu_readings = {t, imu_lin_acc, imu_ang_vel};

  	// send the data to the queue
  	osMessagePut(ImuQueueHandle, (uint32_t) &imu_readings, 100);

  	osDelay(10); // IMU signal every 0.01 sec
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the GpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void const * argument)
{
  /* USER CODE BEGIN StartGpsTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "GPS GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);

	/* Infinite loop */
  for(;;)
  {
  	// Get the RTOS kernel tick count
  	uint32_t  t =  osKernelSysTick();

  	// GPS pseudo-measurements
  	float gps_x = 1.1;
  	float gps_y = 2.2;
  	float gps_z = 3.3;
  	GpsData gps_readings = {t, gps_x, gps_y, gps_z};

  	// send the data to the queue
  	osMessagePut(GpsQueueHandle, (uint32_t) &gps_readings, 100);

  	osDelay(100); // GPS signal every 0.1 sec
  }
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartKFTask */
/**
* @brief Function implementing the KFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKFTask */
void StartKFTask(void const * argument)
{
  /* USER CODE BEGIN StartKFTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "KF GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);

	/* Infinite loop */
  for(;;)
  {
  	// Get the RTOS kernel tick count
  	uint32_t  t =  osKernelSysTick();

  	// receive GPS and IMU data from queues
  	osEvent retval_imu = osMessageGet(ImuQueueHandle, 0);
//  	((ImuData*)retval_imu.value.p)->timestamp;
//  	((ImuData*)retval_imu.value.p)->lin_acc;
//  	((ImuData*)retval_imu.value.p)->ang_vel;
  	osEvent retval_gps = osMessageGet(ImuQueueHandle, 0);
//  	((GpsData*)retval_imu.value.p)->timestamp;
//  	((GpsData*)retval_imu.value.p)->x;
//  	((GpsData*)retval_imu.value.p)->y;
//  	((GpsData*)retval_imu.value.p)->z;

  }
  /* USER CODE END StartKFTask */
}

/* USER CODE BEGIN Header_StartRadarTask */
/**
* @brief Function implementing the RadarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadarTask */
void StartRadarTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "Radar GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);

	// counter to periodically pseudo-detect an object
	uint8_t i = 0; // 8 bits means that after 256 it goes back to 0
  for(;;)
  {
  	// Get the RTOS kernel tick count
  	uint32_t  t =  osKernelSysTick();

  	// at iteration #200 and every 256 (2^8) iterations, "detect an object"
  	if (i == 200){
    	osMutexWait(PrintMtxHandle, osWaitForever);
    	sprintf(msg, "An object!\r\n");
    	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
    	EmptyBuffer(msg);
    	osMutexRelease(PrintMtxHandle);

    	// when detecting an object, send a signal to the object-detection-handle thread
			osSignalSet(ObjectDetectTasHandle, SIGNAL_OBJECT_DETECT);
  	}

  	// stop detecting the object 50 iterations after you detected it
  	if (i == 250){
    	osMutexWait(PrintMtxHandle, osWaitForever);
    	sprintf(msg, "No objects.\r\n");
    	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
    	EmptyBuffer(msg);
    	osMutexRelease(PrintMtxHandle);

    	// signal to the object-detection thread that we no longer detect an object
			osSignalSet(ObjectDetectTasHandle, SIGNAL_OBJECT_DETECT);
  	}

  	// Radar signal every 0.05 sec
    osDelay(50);

    // increment counter
    ++i;
  }
  /* USER CODE END StartRadarTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "UART GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
  /* Infinite loop */
  for(;;)
  {
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartExButtonIntTask */
/**
* @brief Function implementing the ExButtonIntTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExButtonIntTask */
void StartExButtonIntTask(void const * argument)
{
  /* USER CODE BEGIN StartExButtonIntTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "ExButton GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
  /* Infinite loop */
  for(;;)
  {
  	//wait for signal
  	osSignalWait(SIGNAL_BUTTON_PRESS, osWaitForever);

  	// print to uart
  	sprintf(msg, "Button pressed...\r\n");
  	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  	EmptyBuffer(msg);

  	// toggle led
  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  GPIO_PinState red_trig = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	  if (red_trig == GPIO_PIN_SET){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  } else {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  }
  }
  /* USER CODE END StartExButtonIntTask */
}

/* USER CODE BEGIN Header_StartLcdTask */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void const * argument)
{
  /* USER CODE BEGIN StartLcdTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "LCD GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLcdTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "Motor GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
	int lin_vel = 14; // 14 m/sec linear velocity
	int ang_vel = 0.2; // 0.2 rad/sec angular velocity
  for(;;)
  {
  	// block until resumed
  	osThreadSuspend(NULL);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* USER CODE BEGIN StartMainTask */
  /* Infinite loop */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "Main GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 200);
	EmptyBuffer(msg);
	sprintf(msg, "Enabling UART...\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
	// enable UART receive
	HAL_UART_Receive_IT(&huart2, &Rx_byte, 1);
  for(;;)
  {
  	sprintf(msg, "-Main\r\n");
  	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  	EmptyBuffer(msg);

  	// suspend main until it is resumed
  	osThreadSuspend(NULL);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartObjectDetectTask */
/**
* @brief Function implementing the ObjectDetectTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartObjectDetectTask */
void StartObjectDetectTask(void const * argument)
{
  /* USER CODE BEGIN StartObjectDetectTask */
	osMutexWait(PrintMtxHandle, osWaitForever);
	sprintf(msg, "ObjDetect GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
  /* Infinite loop */
  for(;;)
  {
  	//wait for signal
  	osSignalWait(SIGNAL_OBJECT_DETECT, osWaitForever);

  	// print to uart
  	sprintf(msg, "~~ Object (un)detected ~~\r\n");
  	HAL_UART_Transmit(&huart2, msg, strlen(msg), 100);
  	EmptyBuffer(msg);

  	// toggle led
  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
  }
  /* USER CODE END StartObjectDetectTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

// helper function to clear a string
void EmptyBuffer(char* buf){
	uint8_t i;
	int s = strlen(buf);

	for (i=0; i<s; i++) {
		buf[i] = 0;
	}
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

