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

/* Define the GUIX resources. */

/* Define the root window pointer. */

GX_WINDOW_ROOT *root_window;

/* data comning from CM4 core */
__attribute__((section(".sram3.bridgeError"))) volatile unsigned int bridgeError[4];
__attribute__((section(".sram3.bridgeCount"))) volatile unsigned int bridgeCount[4];
__attribute__((section(".sram3.bridgeStale"))) volatile unsigned int bridgeStale[4];
__attribute__((section(".sram3.bridgeBadstatus"))) volatile unsigned int bridgeBadstatus[4];
__attribute__((section(".sram3.bridgeValue"))) volatile uint32_t bridgeValue[4];
__attribute__((section(".sram3.touchData"))) volatile uint16_t touchData[4], touchData2[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm7_main_thread_entry(ULONG thread_input);
void tx_cm7_lcd_thread_entry(ULONG thread_input);
void Error_Handler(void);
static void stm32h7_32argb_buffer_toggle(GX_CANVAS *canvas, GX_RECTANGLE *dirty_area);
UINT stm32h7_graphics_driver_setup_32argb(GX_DISPLAY *display);

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
#if 0
/* Table for touchscreen event information display on LCD : table indexed on enum @ref TS_TouchEventTypeDef information */
char * ts_event_string_tab[4] = { "None",
"Press down",
"Lift up",
"Contact"};

/* Table for touchscreen gesture Id information display on LCD : table indexed on enum @ref TS_GestureIdTypeDef information */
char * ts_gesture_id_string_tab[7] = { "No Gesture",
"Move Up",
"Move Right",
"Move Down",
"Move Left",
"Zoom In",
"Zoom Out"};

/*******************************************************************************
* Function Name  : ft6x06_gest_id
* Description    : Read Gesture ID
* Input          : Pointer to uint8_t
* Output         : Status of GEST_ID register
* Return         : Status [FT6X06_ERROR, FT6X06_OK]
*******************************************************************************/
int32_t  ft6x06_gest_id(ft6x06_ctx_t *ctx, uint8_t *value)
{
  return ft6x06_read_reg(ctx, FT6X06_GEST_ID_REG, (uint8_t *)value, 1);
}
#endif
/*
 * TODO -
 *
 * TS_State_t  TS_State = {0};
 * TS_MultiTouch_State_t  TS_MTState = {0};
 * uint32_t GestureId = GESTURE_ID_NO_GESTURE;
 * ts_status = BSP_TS_GetState(0, &TS_State);
 *   if(TS_State.TouchDetected)
 * ts_status = BSP_TS_Get_MultiTouchState(0, &TS_MTState);
 * ts_status = BSP_TS_GetGestureId(0, &GestureId);
 *
GX_EVENT_PEN_DOWN
- Description: This event is generated by touch screen and mouse input drivers to indicate user pen-down (or left mouse button click) event.
- Payload: gx_event_pointdata.gx_point_x = pen x position in pixels
           gx_event_pointdata.gx_point_y = pen y position in pixels
           gx_event_display_handle = handle of the target display

GX_EVENT_PEN_UP
- Description: This event is generated by touch screen and mouse input drivers to indicate user pen-up (or left mouse button released) event.
- Payload: gx_event_pointdata.gx_point_x = pen x position in pixels
           gx_event_pointdata.gx_point_y = pen y position in pixels
           gx_event_display_handle = handle of the target display

GX_EVENT_PEN_DRAG
- Description: This event is generated by mouse and touch input drivers to indicate the pen is being dragged across the screen, or the mouse is being moved while the left mouse button is pressed.
- Payload: gx_event_pointdata.gx_point_x = pen x position in pixels
           gx_event_pointdata.gx_point_y = pen y position in pixels
           gx_event_display_handle = handle of the target display

GX_EVENT_ZOOM_IN
- Description: This event is generated by multi-touch touch input drivers to indicate a zoom-in gesture has been input by the user.
- Payload: None

GX_EVENT_ZOOM_OUT
- Description: This event is generated by multi-touch touch input drivers to indicate a zoom-out gesture has been input by the user. ]
- Payload: None

 */

void tx_cm7_main_thread_entry(ULONG thread_input)
{
  //UINT status;
  //CHAR read_buffer[32];
  //CHAR data[] = "This is ThreadX working on STM32 CM7";
	uint16_t curTouch[4];
	uint16_t prevTouch[4] = {0};
	ULONG toggleTicks = tx_time_get();

  /* Infinite Loop */
  for( ;; )
  {
  	ULONG ticks = tx_time_get();
  	if (ticks >= toggleTicks)
  	{
    	BSP_LED_Toggle(LED_BLUE);
  		toggleTicks = ticks + TX_TIMER_TICKS_PER_SECOND;
  	}
  	touchData2[0] = touchData[0];
  	touchData2[1] = touchData[1];
  	touchData2[2] = touchData[2];
  	touchData2[3] = touchData[3];
  	if (touchData[3] != prevTouch[3])
  	{
  		/* Send a pen event for processing. */
  		GX_EVENT e = {0};
  		//e.gx_event_display_handle = LCD_FRAME_BUFFER;
    	curTouch[0] = touchData[0];
    	curTouch[1] = touchData[1];
    	curTouch[2] = touchData[2];
    	curTouch[3] = touchData[3];
  		e.gx_event_payload.gx_event_pointdata.gx_point_x = curTouch[2];       // Y value of touchscreen driver
  		e.gx_event_payload.gx_event_pointdata.gx_point_y = 480 - curTouch[1]; // X value of touchscreen driver, but reversed
  		if (curTouch[0] == 0)
  		{
    		e.gx_event_type = GX_EVENT_PEN_UP; // pen UP
  			//BSP_LED_Off(LED_RED);
  		}
  		else if (prevTouch[0] == 0)
  		{
    		e.gx_event_type = GX_EVENT_PEN_DOWN; // pen DOWN
  			//BSP_LED_On(LED_RED);
  		}
  		else
    		e.gx_event_type = GX_EVENT_PEN_DRAG; // pen DRAG
  		/* Push the event to event pool. */
  		if (e.gx_event_type == GX_EVENT_PEN_DRAG)
    		gx_system_event_fold(&e);
  		else
  			gx_system_event_send(&e);
  		memcpy(prevTouch, curTouch, sizeof(curTouch));
  	}
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 5);
  }
}

void tx_cm7_lcd_thread_entry(ULONG thread_input)
{
  extern GX_STUDIO_DISPLAY_INFO H747_Test_display_table[]; // meh...

  /* Initialize GUIX. */
  gx_system_initialize();

  /* Configure the main display. */
  H747_Test_display_table[MAIN_DISPLAY].canvas_memory = (GX_COLOR *) Buffers[0]; // I think...
  gx_studio_display_configure(MAIN_DISPLAY,                         /* Display to configure*/
															stm32h7_graphics_driver_setup_32argb, /* Driver to use */
                              LANGUAGE_ENGLISH,                     /* Language to install */
                              MAIN_DISPLAY_DEFAULT_THEME,           /* Theme to install */
                              &root_window);                        /* Root window pointer */

  /* Create the screen - attached to root window. */

  gx_studio_named_widget_create("main_window", (GX_WIDGET *) root_window, GX_NULL);

  /* Show the root window to make it visible. */
  gx_widget_show(root_window);

  /* Let GUIX run. */
  gx_system_start();
}

/*************************************************************************************/
VOID weight_update()
{
	uint32_t low[4] = { 950, 950, 950, 950 };
	uint32_t total = 0;
  /* Set a value to "my_numeric_pix_prompt". */
	for (unsigned int i = 0; i < 4; i++)
	{
		uint32_t weight = (bridgeValue[i] >> 16) & 0x3fff;
		if (weight < low[i])
			weight = 0;
		else
			weight -= low[i];
		weight *= 5000;
		weight /= 14000;
		total += weight;
	}
	total /= 10;
  gx_numeric_pixelmap_prompt_value_set(&main_window.main_window_weight_prompt, total);
}

/*************************************************************************************/
UINT main_screen_event_handler(GX_WINDOW *window, GX_EVENT *event_ptr)
{
	switch (event_ptr->gx_event_type)
	{
		case GX_EVENT_SHOW:
			/* Set current weight. */
			weight_update();

			/* Start a timer to update weight. */
			gx_system_timer_start(&main_window, CLOCK_TIMER, GX_TICKS_SECOND / 2, GX_TICKS_SECOND / 2);
			break;

		case GX_EVENT_TIMER:
			if (event_ptr->gx_event_payload.gx_event_timer_id == CLOCK_TIMER)
			{
				weight_update();
			}
			break;

		default:
			BSP_LED_Toggle(LED_RED);
			break;
	}
	return gx_window_event_process(window, event_ptr);
}

/* Define my numeric format function. */
VOID weight_format_func(GX_NUMERIC_PIXELMAP_PROMPT *prompt, INT value)
{
	/* If the value is "1234", the new format will be "123.4". */

	INT length;
	gx_utility_ltoa(value / 10,
									prompt->gx_numeric_pixelmap_prompt_buffer,
									GX_NUMERIC_PROMPT_BUFFER_SIZE);
	length = GX_STRLEN(prompt->gx_numeric_pixelmap_prompt_buffer);
	prompt->gx_numeric_pixelmap_prompt_buffer[length++] = '.';
	gx_utility_ltoa(value % 10,
									prompt->gx_numeric_pixelmap_prompt_buffer + length,
									GX_NUMERIC_PROMPT_BUFFER_SIZE - length);
}

UINT weight_prompt_event(GX_NUMERIC_PIXELMAP_PROMPT *widget, GX_EVENT *event_ptr)
{
	UINT status = GX_SUCCESS;

	switch(event_ptr->gx_event_type)
	{
		//case xyz:
			/* Insert custom event handling here */
			//break;

		default:
			/* Pass all other events to the default tree view event processing */
			status = gx_prompt_event_process((GX_PROMPT *) widget, event_ptr);
			break;
	}
	return status;
}

/*
 * VOID (*gx_display_driver_buffer_toggle)(struct GX_CANVAS_STRUCT *canvas, GX_RECTANGLE *dirty_area)
 * This is a pointer to a function to toggle between the working and visible frame buffers for
 * double-buffered memory systems. This function must first instruct the hardware to begin using
 * the new frame buffer, then copy the modified portion of the new visible buffer to the companion
 * buffer, to insure the two buffers stay in synch.
 */
static void stm32h7_32argb_buffer_toggle(GX_CANVAS *canvas, GX_RECTANGLE *dirty_area)
{
	ULONG offset;
	INT   copy_width;
	INT   copy_height;
	ULONG *get;
	ULONG *put;
	ULONG actual_events;

	/* FIXME - maybe make sure the event is cleared here ?  */

	/* swap the buffers */
	if (pend_buffer < 0)
	{
		/* Switch to other buffer */
		pend_buffer = 1 - front_buffer;

		/* Refresh the display */
		HAL_DSI_Refresh(&hlcd_dsi);
	}

	/* Request that event flags 0 is set. If it is set it should be cleared. If the event
	flags are not set, this service suspends for a maximum of 200 timer-ticks. */
	UINT status = tx_event_flags_get(&cm7_event_group, 0x1, TX_AND_CLEAR, &actual_events, 200);

	/* If status equals TX_SUCCESS, actual_events contains the actual events obtained. */
	if (status == TX_SUCCESS)
	{
		/* now refresh offline buffer and switch buffers in canvas  */

		copy_width = dirty_area->gx_rectangle_right - dirty_area->gx_rectangle_left + 1;
		copy_height = dirty_area->gx_rectangle_bottom - dirty_area->gx_rectangle_top + 1;

		/* Read the update area from the canvas */
		offset = dirty_area->gx_rectangle_top * canvas->gx_canvas_x_resolution;
		offset += dirty_area->gx_rectangle_left;
		get = canvas->gx_canvas_memory;
		get += offset;

		/* Read the area to be updated from the LCD video memory and copy the updated data from the canvas */
		put = (ULONG *) Buffers[1 - front_buffer];
		offset = (canvas->gx_canvas_display_offset_y + dirty_area->gx_rectangle_top) * MAIN_DISPLAY_X_RESOLUTION;
		offset += canvas->gx_canvas_display_offset_x + dirty_area->gx_rectangle_left;
		put += offset;

		// RM0388 - pp 780...  not sure about interrupt vs polling yet
    DMA2D->CR = 0x00000000UL; // | DMA2D_CR_TCIE;
    DMA2D->FGMAR = (uint32_t) get;
    DMA2D->OMAR = (uint32_t) put;
    DMA2D->FGOR = canvas->gx_canvas_x_resolution - copy_width;
    DMA2D->OOR = MAIN_DISPLAY_X_RESOLUTION - copy_width;
    DMA2D->FGPFCCR = LTDC_PIXEL_FORMAT_ARGB8888;
    DMA2D->OPFCCR = LTDC_PIXEL_FORMAT_ARGB8888;
    DMA2D->NLR = (uint32_t) (copy_width << 16) | (uint16_t) copy_height;
    DMA2D->CR |= DMA2D_CR_START;
    while (DMA2D->CR & DMA2D_CR_START) {}

		/* Assign canvas memory block. */
		status = gx_canvas_memory_define(canvas, (GX_COLOR *) Buffers[1 - front_buffer], (800*480*4));

		/* If status is GX_SUCCESS, the canvas memory pointer has been reassigned. */

	}
	else
		BSP_LED_Toggle(LED_RED);
}

UINT stm32h7_graphics_driver_setup_32argb(GX_DISPLAY *display)
{
	// Display hardware should have been setup already

	_gx_display_driver_32argb_setup(display, GX_NULL, stm32h7_32argb_buffer_toggle);

	return(GX_SUCCESS);
}
/* USER CODE END 1 */
