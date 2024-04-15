/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "car_task.h"
#include "inv_mpu_user.h"
#include "esp32.h"
#include "connect.h"
#include "oled.h"
#include "oled_show.h"
#include "car_system.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId Task_200HZHandle;
osThreadId Task_100HZHandle;
osThreadId Task_5HZHandle;
osThreadId Task_InteractioHandle;

/* USER CODE BEGIN Variables */

#define   Message_Q_NUM      5
#define   Message_Q_Length   sizeof(tEsp32_RcvBuf)
xQueueHandle  Message_Queue;
tEsp32_RcvBuf Uart6_Rcv;


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartTask_200HZ(void const * argument);
void StartTask_100HZ(void const * argument);
void StartTask_5HZ(void const * argument);
void StartTask_Interaction(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

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

  /* Create the thread(s) */
  /* definition and creation of Task_200HZ */
  osThreadDef(Task_200HZ, StartTask_200HZ, osPriorityNormal, 0, 128);
  Task_200HZHandle = osThreadCreate(osThread(Task_200HZ), NULL);

  /* definition and creation of Task_100HZ */
  osThreadDef(Task_100HZ, StartTask_100HZ, osPriorityIdle, 0, 128);
  Task_100HZHandle = osThreadCreate(osThread(Task_100HZ), NULL);

  /* definition and creation of Task_5HZ */
  osThreadDef(Task_5HZ, StartTask_5HZ, osPriorityIdle, 0, 128);
  Task_5HZHandle = osThreadCreate(osThread(Task_5HZ), NULL);

  /* definition and creation of Task_Interactio */
  osThreadDef(Task_Interactio, StartTask_Interaction, osPriorityIdle, 0, 128);
  Task_InteractioHandle = osThreadCreate(osThread(Task_Interactio), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartTask_200HZ function */
void StartTask_200HZ(void const * argument)
{

  /* USER CODE BEGIN StartTask_200HZ */
	
	//printf("环境采集进程运行\n");
	
	
	MPU_Init();
	
	while(mpu_dmp_init());
	
  /* Infinite loop */
  for(;;)
  {
		
		Car_Task_200HZ();
		
    osDelay(5);
  }
  /* USER CODE END StartTask_200HZ */
}

/* StartTask_100HZ function */
void StartTask_100HZ(void const * argument)
{
  /* USER CODE BEGIN StartTask_100HZ */
	
	//printf("PID控制进程运行\n");
	
	
	
	
  /* Infinite loop */
  for(;;)
  {
		
		Car_Task_100HZ();
    osDelay(10);
  }
  /* USER CODE END StartTask_100HZ */
}

/* StartTask_5HZ function */
void StartTask_5HZ(void const * argument)
{
  /* USER CODE BEGIN StartTask_5HZ */
	
	printf("菜单显示进程运行\n");
	
	//0.96寸OLED显示屏初始化
	oled_Init();
	//显示顶层界面
	Interface();  
	
	
  /* Infinite loop */
  for(;;)
  {
		Car_Task_5HZ();
    osDelay(200);
  }
  /* USER CODE END StartTask_5HZ */
}

/* StartTask_Interaction function */
void StartTask_Interaction(void const * argument)
{
  /* USER CODE BEGIN StartTask_Interaction */
	
	uint8_t time = 0;
	
	 printf("交互进程运行\n");
	
	 Message_Queue =  xQueueCreate ( Message_Q_NUM, Message_Q_Length );
	
	 HAL_UART_Receive_DMA(&huart6, Uart6_Rcv.RcvBuf, 255);
	 __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE );
	
	ESP32_Init();
	
	
  /* Infinite loop */
  for(;;)
  {
		
		ESP32_Data_Rcv();
		
		time++;
		if(time >= 50)
		{
			time = 0;
			Connect_Send_data(READ_ALL_ARG);
		}
    osDelay(10);
  }
  /* USER CODE END StartTask_Interaction */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
