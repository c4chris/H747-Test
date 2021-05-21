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

/* Define the GUIX resources for this demo. */

/* GUIX display represents the physical display device */
GX_DISPLAY       demo_display;

/* GUIX canvas is the frame buffer on top of GUIX displayl. */
GX_CANVAS        default_canvas;

/* The root window is a special GUIX background window, right on top of the canvas. */
GX_WINDOW_ROOT   demo_root_window;

/* GUIX Prompt displays a string. */
GX_PROMPT        demo_prompt;

/* Memory for the frame buffer. */
//GX_COLOR default_canvas_memory[DEFAULT_CANVAS_PIXELS];
GX_COLOR *default_canvas_memory = (void *) (LCD_FRAME_BUFFER + 0x177000); // FIXME - maybe...

/* Define GUIX strings ID for the demo. */
enum demo_string_ids
{
    SID_HELLO_WORLD = 1,
    SID_MAX
};

/* Define GUIX string for the demo. */
CHAR *demo_strings[] = {
    NULL,
    "Hello World"
};

#if 0
/* User-defined color table. */
static GX_COLOR demo_color_table[] =
{
    /* In this demo, two color entries are added to the color table. */
    GX_COLOR_BLACK,
    GX_COLOR_WHITE
};
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm7_main_thread_entry(ULONG thread_input);
void tx_cm7_lcd_thread_entry(ULONG thread_input);
void Error_Handler(void);

/* USER CODE END PFP */
static void stm32h7_24xrgb_buffer_toggle(GX_CANVAS *canvas, GX_RECTANGLE *dirty)
{
	GX_RECTANGLE    Limit;
	GX_RECTANGLE    Copy;
	ULONG           offset;
	INT             copy_width;
	INT             copy_height;
	INT             row;
	INT             src_stride_ulongs;
	INT             dest_stride_ulongs;
	ULONG *get;
	ULONG *put;

	gx_utility_rectangle_define(&Limit, 0, 0,
															canvas->gx_canvas_x_resolution - 1,
															canvas->gx_canvas_y_resolution - 1);

	if (gx_utility_rectangle_overlap_detect(&Limit, &canvas->gx_canvas_dirty_area, &Copy))
	{
		copy_width = Copy.gx_rectangle_right - Copy.gx_rectangle_left + 1;
		copy_height = Copy.gx_rectangle_bottom - Copy.gx_rectangle_top + 1;

		/* Read the update area from the canvas */
		offset = Copy.gx_rectangle_top * canvas->gx_canvas_x_resolution;
		offset += Copy.gx_rectangle_left;
		get = canvas ->gx_canvas_memory;
		get += offset;

		/* Read the area to be updated from the LCD video memory and copy the updated data from the canvas */
		put = (ULONG *) LCD_FRAME_BUFFER;
		offset = (canvas->gx_canvas_display_offset_y + Copy.gx_rectangle_top)* DEMO_DISPLAY_WIDTH;
		offset += canvas->gx_canvas_display_offset_x + Copy.gx_rectangle_left;
		put += offset;

		src_stride_ulongs = canvas ->gx_canvas_x_resolution;
		dest_stride_ulongs = DEMO_DISPLAY_WIDTH;

		for(row = 0; row < copy_height; row++)
		{
			memcpy(put, get, copy_width * 4);
			put += dest_stride_ulongs;
			get += src_stride_ulongs;
		}
	}
}

UINT stm32h7_graphics_driver_setup_24xrgb(GX_DISPLAY *display)
{
	// Display hardware should have been setup already

	_gx_display_driver_32argb_setup(display, (void *)LCD_FRAME_BUFFER, stm32h7_24xrgb_buffer_toggle);

	return(GX_SUCCESS);
}

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
  //UINT status;
  //CHAR read_buffer[32];
  //CHAR data[] = "This is ThreadX working on STM32 CM7";

  /* Infinite Loop */
  for( ;; )
  {
  	BSP_LED_Toggle(LED_BLUE);
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
  }
}

void tx_cm7_lcd_thread_entry(ULONG thread_input)
{
#if 0
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
#endif
  GX_RECTANGLE    root_window_size;
  GX_RECTANGLE    prompt_position;

  /* Initialize GUIX. */
  gx_system_initialize();

	/* Install the demo string table. */
	//gx_system_string_table_set(demo_strings, SID_MAX);

	/* Install the demo color table. */
	//gx_system_color_table_set(demo_color_table, sizeof(demo_color_table) / sizeof(GX_COLOR));

	/* Create the demo display and associated driver. */
	gx_display_create(&demo_display, "demo display",
										stm32h7_graphics_driver_setup_24xrgb,
										DEMO_DISPLAY_WIDTH, DEMO_DISPLAY_HEIGHT);

	/* Create the default canvas. */
	gx_canvas_create(&default_canvas, "demo canvas",&demo_display,
							 GX_CANVAS_MANAGED | GX_CANVAS_VISIBLE,
							 DEMO_DISPLAY_WIDTH,DEMO_DISPLAY_HEIGHT,
							 default_canvas_memory, sizeof(default_canvas_memory));

	/*Define the size of the root window. */
	gx_utility_rectangle_define(&root_window_size, 0, 0,
													DEMO_DISPLAY_WIDTH - 1, DEMO_DISPLAY_HEIGHT - 1);

	/* Create a background root window and attach to the canvas. */
	gx_window_root_create(&demo_root_window, "demo root window", &default_canvas,
										GX_STYLE_BORDER_NONE, GX_ID_NONE, &root_window_size);

	/* Set the root window to be black. */
	//gx_widget_background_set(&demo_root_window, GX_COLOR_ID_BLACK, GX_COLOR_ID_BLACK);

	/* Create a text prompt on the root window. Set the text color to white, and the background to black. */

	/* Define the size and the position of the prompt. */
	gx_utility_rectangle_define(&prompt_position, 100, 90, 220, 130);

	/* Create the prompt on top of the root window using the string defined by string ID SID_HELLO_WORLD. */
	gx_prompt_create(&demo_prompt, NULL, &demo_root_window, SID_HELLO_WORLD,
							 GX_STYLE_NONE, GX_ID_NONE, &prompt_position);

	/* Set the text color to be white, and the background color to be black. */
	gx_prompt_text_color_set(&demo_prompt, GX_COLOR_ID_WHITE, GX_COLOR_ID_WHITE, GX_COLOR_ID_WHITE);
	//gx_widget_background_set(&demo_prompt, GX_COLOR_ID_BLACK,GX_COLOR_ID_BLACK);

	/* Show the root window. */
	gx_widget_show(&demo_root_window);

	/* let GUIX run! */
	gx_system_start();
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
