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
#define SIGNAL_MOTOR 1
#define SIGNAL_OBJECT_DETECT 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId ImuTaskHandle;
osThreadId GpsTaskHandle;
osThreadId KFTaskHandle;
osThreadId RadarTaskHandle;
osThreadId ExButtonIntTaskHandle;
osThreadId MotorTaskHandle;
osThreadId MainTaskHandle;
osThreadId ObjectDetectTasHandle;
osThreadId UartTaskHandle;
osMessageQId UartQueueHandle;
osMessageQId ImuQueueHandle;
osMessageQId GpsQueueHandle;
osMessageQId ButtonQueueHandle;
osMutexId PrintMtxHandle;
osSemaphoreId VelSemaphoreHandle;
/* USER CODE BEGIN PV */
// UART stuff
uint8_t Rx_byte, Rx_indx, Transfer_cplt, Rx_Buffer[100], msg[40];
// velocities
int8_t v, w;
// speed of sound
const float speedOfSound = 0.0343/2;
float distance;
// timer - ultrasonic
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
uint8_t detect = 0;

// custom structures for IMU, gps data topics
typedef struct
{
	float ax;
	float ay;
	float az;
} LinAcc;

typedef struct
{
	float wx;
	float wy;
	float wz;
} AngVel;

typedef struct
{
	uint32_t timestamp;
	LinAcc lin_acc;
	AngVel ang_vel;

} ImuData;


typedef struct
{
	uint32_t timestamp;
	float x;
	float y;
	float z;

} GpsData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void StartImuTask(void const * argument);
void StartGpsTask(void const * argument);
void StartKFTask(void const * argument);
void StartRadarTask(void const * argument);
void StartExButtonIntTask(void const * argument);
void StartMotorTask(void const * argument);
void StartMainTask(void const * argument);
void StartObjectDetectTask(void const * argument);
void StartUartTask(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
// UART callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

// empty a string buffer
void EmptyBuffer(uint8_t* buf){
	uint8_t i;
	int s = strlen(buf);

	for (i=0; i<s; i++) {
		buf[i] = 0;
	}
}

// read stuff from the HCSR04 supersonic sensor
// Triggers the sensor to start the measurement.
// It will pull the TRIG Pin HIGH for 10 microseconds, and then pull it LOW.
// This will force the sensor to start the measurement, and the sensor will pull the ECHO Pin HIGH for the respective amount of time
// To measure this time, we will enable the Timer interrupt, so that we can capture this Rising and falling edges
void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

// delay function for the supersonic sensor
void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}
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
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // to get IC_CaptureCallback
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of PrintMtx */
  osMutexDef(PrintMtx);
  PrintMtxHandle = osMutexCreate(osMutex(PrintMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of VelSemaphore */
  osSemaphoreDef(VelSemaphore);
  VelSemaphoreHandle = osSemaphoreCreate(osSemaphore(VelSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of UartQueue */
  osMessageQDef(UartQueue, 1, int8_t);
  UartQueueHandle = osMessageCreate(osMessageQ(UartQueue), NULL);

  /* definition and creation of ImuQueue */
  osMessageQDef(ImuQueue, 1, ImuData);
  ImuQueueHandle = osMessageCreate(osMessageQ(ImuQueue), NULL);

  /* definition and creation of GpsQueue */
  osMessageQDef(GpsQueue, 1, GpsData);
  GpsQueueHandle = osMessageCreate(osMessageQ(GpsQueue), NULL);

  /* definition and creation of ButtonQueue */
  osMessageQDef(ButtonQueue, 1, uint8_t);
  ButtonQueueHandle = osMessageCreate(osMessageQ(ButtonQueue), NULL);

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

  /* definition and creation of ExButtonIntTask */
  osThreadDef(ExButtonIntTask, StartExButtonIntTask, osPriorityAboveNormal, 0, 128);
  ExButtonIntTaskHandle = osThreadCreate(osThread(ExButtonIntTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityHigh, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of ObjectDetectTas */
  osThreadDef(ObjectDetectTas, StartObjectDetectTask, osPriorityAboveNormal, 0, 128);
  ObjectDetectTasHandle = osThreadCreate(osThread(ObjectDetectTas), NULL);

  /* definition and creation of UartTask */
  osThreadDef(UartTask, StartUartTask, osPriorityNormal, 0, 128);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65536-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YellowLed_Pin|RedLed_Pin|GreenLed_Pin|BlueLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trig_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	uint8_t flag; // accelerate -> 1, decelerate -> 0, else -> don't do anything

	 // current UART
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
  			flag = 1;
  		} else if (strcmp(Rx_Buffer, "slower")  == 0) {
  			sprintf(msg, "Decelerating!");
  			flag = 0;
  		} else {
  			sprintf(msg, "Unknown command.");
  			flag = 2;
  		}

  		// send to UART
  		HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  		HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  		HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
    	EmptyBuffer(msg);

    	// send flag to the UartThread via a queue
	  	osMessagePut(UartQueueHandle, flag, 100);

    	// turn off the yellow led
    	HAL_GPIO_WritePin(YellowLed_GPIO_Port, YellowLed_Pin, GPIO_PIN_RESET);
  	}

  	// activate UART
  	HAL_UART_Receive_IT(&huart2, &Rx_byte, 1);
  	// send to UART
  	HAL_UART_Transmit(&huart2, &Rx_byte, 1, 100);
	}
}

// internal timer callback
// First Timestamp is captured, when the rising edge is detected. The polarity is now set for the falling edge
// Second Timestamp will be captured on the falling edge
// Difference between the Timestamps will be calculated. This Difference will be microseconds, as the timer is running at 1 MHz
// Based on the Difference value, the distance is calculated using the formula given in the datasheet
// Finally, the Interrupt will be disabled, so that we don’t capture any unwanted signals.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;

	  	// print object distance
//			sprintf(msg, "dist: %d\r\n", Distance);
//			HAL_UART_Transmit(&huart2, msg, strlen(msg), 1000);
//			EmptyBuffer(msg);

			// if an object is too close, STOP THE VEHICLE
	  	if (Distance < 20){
	    	// when detecting an object, send a signal to the object-detection-handle thread
				detect = 1;
	    	osSignalSet(ObjectDetectTasHandle, SIGNAL_OBJECT_DETECT);
	  	}

	  	// if there is no object, allow the vehicle to move again
	  	if (Distance >=20){
	    	// signal to the object-detection thread that we no longer detect an object
				detect = 0;
				osSignalSet(ObjectDetectTasHandle, SIGNAL_OBJECT_DETECT);
	  	}

			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
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
  	LinAcc imu_lin_acc = {0.1, 0.0, 0.0};
  	AngVel imu_ang_vel = {0.0, 0.0, 0.2};
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
  	osEvent retval_gps = osMessageGet(GpsQueueHandle, 0);
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
  	// read from the supersonic sensor
  	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  	delay(10);  // wait for 10 us
  	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

  	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
  	// delay for 0.05 sec -> pretty normal radar frequency
  	osDelay(50);
  }
  /* USER CODE END StartRadarTask */
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

  	// only if there are no detected objects the emergency button should do something
  	if (detect == 0) {
    	// read redled state
  	  GPIO_PinState red_trig = HAL_GPIO_ReadPin(RedLed_GPIO_Port, RedLed_Pin);
  	  if (red_trig == GPIO_PIN_RESET){
  	  	// stop the vehicle!!
  	  	osSemaphoreWait(VelSemaphoreHandle, osWaitForever);
  	  	v = 0; // update the velocity references
  	  	w = 0;
  	  	sprintf(msg, "Lin. vel: %d Ang vel: %d\r\n", v, w);
  	  	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  	  	EmptyBuffer(msg);
  	  	osSemaphoreRelease(VelSemaphoreHandle);

  	  	// turn on red light
  		  HAL_GPIO_WritePin(RedLed_GPIO_Port, RedLed_Pin, GPIO_PIN_SET);
  	  	// turn off green light
  		  HAL_GPIO_WritePin(GreenLed_GPIO_Port, GreenLed_Pin, GPIO_PIN_RESET);
  	  } else {
  	  	// start the vehicle
  	  	osSemaphoreWait(VelSemaphoreHandle, osWaitForever);
  	  	v = 10; // update the velocity references
  	  	w = 0;
  	  	sprintf(msg, "Lin. vel: %d Ang vel: %d\r\n", v, w);
  	  	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  	  	EmptyBuffer(msg);
  	  	osSemaphoreRelease(VelSemaphoreHandle);

  	  	// turn on the green light
  		  HAL_GPIO_WritePin(GreenLed_GPIO_Port, GreenLed_Pin, GPIO_PIN_SET);
  	  	// turn off the red light
  		  HAL_GPIO_WritePin(RedLed_GPIO_Port, RedLed_Pin, GPIO_PIN_RESET);
  	  }
  	}
  }
  /* USER CODE END StartExButtonIntTask */
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
	v = 0; // 14 m/sec linear velocity
	w = 0; // 0.2 rad/sec angular velocity
  for(;;)
  {
  	// block until resumed
  	osThreadSuspend(NULL);

  	// here, we can feed the incoming velocity references to the PID controller after another thread calls osThreadResume(MotorTaskHandle)
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
	// enable tim1 IT for radar object detection
  HAL_TIM_Base_Start_IT(&htim1);
//  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // to get IC_CaptureCallback

  /* Infinite loop */
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

	// previous detections flag
	uint8_t prev_detect = 0;
  /* Infinite loop */
  for(;;)
  {
  	//wait for signal
  	osSignalWait(SIGNAL_OBJECT_DETECT, osWaitForever);

  	// print to uart if something changed
  	if (detect != prev_detect) {
    	if (detect == 1) {
      	sprintf(msg, "~~ DANGER! Object! ~~\r\n");
      	HAL_UART_Transmit(&huart2, msg, strlen(msg), 100);
      	EmptyBuffer(msg);

      	// turn on blue light
      	HAL_GPIO_WritePin(BlueLed_GPIO_Port, BlueLed_Pin, GPIO_PIN_SET);

      	// stop the vehicle!!
      	osSemaphoreWait(VelSemaphoreHandle, osWaitForever);
      	v = 0; // update the velocity references
      	w = 0;
      	// turn on red light
      	HAL_GPIO_WritePin(RedLed_GPIO_Port, RedLed_Pin, GPIO_PIN_SET);
      	// turn off green light
      	HAL_GPIO_WritePin(GreenLed_GPIO_Port, GreenLed_Pin, GPIO_PIN_RESET);
      	sprintf(msg, "Lin. vel: %d Ang vel: %d\r\n", v, w);
      	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 200);
      	EmptyBuffer(msg);
      	osSemaphoreRelease(VelSemaphoreHandle);
    	} else {
      	sprintf(msg, "~~ Clear path. No objects. ~~\r\n");
      	HAL_UART_Transmit(&huart2, msg, strlen(msg), 100);
      	EmptyBuffer(msg);

      	// turn off blue light
      	HAL_GPIO_WritePin(BlueLed_GPIO_Port, BlueLed_Pin, GPIO_PIN_RESET);
    	}

    	// update previous detection
    	prev_detect = detect;
  	}
  }
  /* USER CODE END StartObjectDetectTask */
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
	sprintf(msg, "ObjDetect GO\r\n");
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	EmptyBuffer(msg);
	osMutexRelease(PrintMtxHandle);
  /* Infinite loop */
  for(;;)
  {
  	//wait for signal
  	osEvent retval_uart = osMessageGet(UartQueueHandle, osWaitForever);
  	uint8_t accel_flag = retval_uart.value.p;

  	// if the passed flag is 1, we accelerate by 10%, otherwise, we decelerate by 10%
  	osSemaphoreWait(VelSemaphoreHandle, osWaitForever);
  	if (accel_flag == 1) {
  		if ( v == 0 ) {
  			v = 10;
      	// turn off red light
      	HAL_GPIO_WritePin(RedLed_GPIO_Port, RedLed_Pin, GPIO_PIN_RESET);
      	// turn on green light
      	HAL_GPIO_WritePin(GreenLed_GPIO_Port, GreenLed_Pin, GPIO_PIN_SET);
  		} else {
    		v = v*1.1;
  		}
  	} else if (accel_flag == 0){
			if ( v != 0 ) {
				v = v*0.9;
			}
  	}
  	sprintf(msg, "Lin. vel: %d.%d Ang vel: %d.%d\r\n", v/10, v%10, w/10, w%10);
  	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  	EmptyBuffer(msg);
  	osSemaphoreRelease(VelSemaphoreHandle);
  }
  /* USER CODE END StartUartTask */
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

