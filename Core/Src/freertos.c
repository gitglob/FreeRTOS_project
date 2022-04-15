/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GpsTask */
osThreadId_t GpsTaskHandle;
const osThreadAttr_t GpsTask_attributes = {
  .name = "GpsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KFTask */
osThreadId_t KFTaskHandle;
const osThreadAttr_t KFTask_attributes = {
  .name = "KFTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RadarTask */
osThreadId_t RadarTaskHandle;
const osThreadAttr_t RadarTask_attributes = {
  .name = "RadarTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ExButtonIntTask */
osThreadId_t ExButtonIntTaskHandle;
const osThreadAttr_t ExButtonIntTask_attributes = {
  .name = "ExButtonIntTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LcdTask */
osThreadId_t LcdTaskHandle;
const osThreadAttr_t LcdTask_attributes = {
  .name = "LcdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartImuTask(void *argument);
void StartGpsTask(void *argument);
void StartKFTask(void *argument);
void StartRadarTask(void *argument);
void StartUartTask(void *argument);
void StartExButtonIntTask(void *argument);
void StartLcdTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of GpsTask */
  GpsTaskHandle = osThreadNew(StartGpsTask, NULL, &GpsTask_attributes);

  /* creation of KFTask */
  KFTaskHandle = osThreadNew(StartKFTask, NULL, &KFTask_attributes);

  /* creation of RadarTask */
  RadarTaskHandle = osThreadNew(StartRadarTask, NULL, &RadarTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of ExButtonIntTask */
  ExButtonIntTaskHandle = osThreadNew(StartExButtonIntTask, NULL, &ExButtonIntTask_attributes);

  /* creation of LcdTask */
  LcdTaskHandle = osThreadNew(StartLcdTask, NULL, &LcdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartImuTask */
/**
  * @brief  Function implementing the ImuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the GpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
void StartKFTask(void *argument)
{
  /* USER CODE BEGIN StartKFTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
void StartRadarTask(void *argument)
{
  /* USER CODE BEGIN StartRadarTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
void StartExButtonIntTask(void *argument)
{
  /* USER CODE BEGIN StartExButtonIntTask */
  /* Infinite loop */
  for(;;)
  {
  	vTaskSuspend(NULL); // suspend itself
  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
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
void StartLcdTask(void *argument)
{
  /* USER CODE BEGIN StartLcdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLcdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

