#ifndef HELPERS_H
#define HELPERS_H

// helper function to empty the contents of a buffer
void EmptyBuffer(uint8_t* buf){
	uint8_t i;
	int s = strlen(buf);

	for (i=0; i<s; i++) {
		buf[i] = 0;
	}
}

void PseudoStartRadarTask(void const * argument)
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

#endif /* HELPERS_H */
