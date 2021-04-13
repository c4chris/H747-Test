/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STACK_SIZE               (1 * 1024)
/* fx_sd_thread priority */
#define DEFAULT_THREAD_PRIO              10

/* fx_sd_thread preemption priority */
#define DEFAULT_PREEMPTION_THRESHOLD      DEFAULT_THREAD_PRIO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Buffer for FileX FX_MEDIA sector cache. this should be 32-Bytes
aligned to avoid cache maintenance issues */
//ALIGN_32BYTES (uint32_t media_memory[512]);

/* Define ThreadX global data structures.  */
TX_THREAD       cm7_main_thread;
TX_THREAD       cm7_lcd_thread;
TX_EVENT_FLAGS_GROUP cm7_event_group;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm7_main_thread_entry(ULONG thread_input);
void tx_cm7_lcd_thread_entry(ULONG thread_input);
void Error_Handler(void);

/* USER CODE END PFP */
/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  CHAR *pointer;

  /*Allocate memory for fx_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check FILEX_DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm7_main_thread, "tx_cm7_main_thread", tx_cm7_main_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create an event flags group. */
  ret = tx_event_flags_create(&cm7_event_group, "cm7_event_group_name");

  /* If status equals TX_SUCCESS, my_event_group is ready for get and set services. */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }


  /*Allocate memory for fx_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check FILEX_DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm7_lcd_thread, "tx_cm7_lcd_thread", tx_cm7_lcd_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
void tx_cm7_main_thread_entry(ULONG thread_input)
{
  UINT status;
  CHAR read_buffer[32];
  CHAR data[] = "This is ThreadX working on STM32 CM7";

  /* Infinite Loop */
  for( ;; )
  {
  	BSP_LED_Toggle(LED_BLUE);
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
  }
}

void tx_cm7_lcd_thread_entry(ULONG thread_input)
{
  UINT status;
	ULONG actual_events;

  /* Infinite Loop */
  for( ;; )
  {
  	/* Request that event flags 0 is set. If it is set it should be cleared. If the event
  	flags are not set, this service suspends for a maximum of
  	20 timer-ticks. */
  	status = tx_event_flags_get(&cm7_event_group, 0x1, TX_AND_CLEAR, &actual_events, 200);

  	/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
  	if (status == TX_SUCCESS)
  	{
  		update_image();
  	}
  	else
  		BSP_LED_Toggle(LED_RED);
  }
}

//void BSP_SD_DetectCallback(uint32_t Instance, uint32_t Status)
//{
//  ULONG s_msg = CARD_STATUS_CHANGED;
//
//  if(Instance == SD_INSTANCE)
//  {
//    tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
//  }
//
//  UNUSED(Status);
//}
/* USER CODE END 1 */
