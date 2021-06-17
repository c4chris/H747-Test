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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _cell_t {
	I2C_HandleTypeDef *handle;
	volatile uint8_t address;
	GPIO_TypeDef *gpio;
	uint16_t pin;
} CellTypeDef;

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

/* Define ThreadX global data structures.  */
TX_THREAD       cm4_main_thread;
TX_THREAD       cm4_i2c1_thread;
TX_THREAD       cm4_i2c4_thread;
TX_THREAD       cm4_uart_thread;

/* ...  */
volatile unsigned int u2rc;
volatile unsigned int u2hrc;
volatile unsigned int u2tc;
volatile unsigned int u2htc;
volatile unsigned int u2ec;
volatile unsigned int u2ic;
__attribute__((section(".sram3.bridgeError"))) volatile unsigned int bridgeError[4];
__attribute__((section(".sram3.bridgeCount"))) volatile unsigned int bridgeCount[4];
__attribute__((section(".sram3.bridgeStale"))) volatile unsigned int bridgeStale[4];
__attribute__((section(".sram3.bridgeBadstatus"))) volatile unsigned int bridgeBadstatus[4];
__attribute__((section(".sram3.bridgeValue"))) volatile uint32_t bridgeValue[4];
volatile uint16_t touchData[4];
unsigned char dbgBuf[256];
unsigned char input[64];
unsigned char u2tx[256];
volatile uint8_t setZero[4];
volatile uint8_t setZero3;
volatile uint8_t setZero4;
volatile uint8_t availableCells;
CellTypeDef cell[4] = {
		{&hi2c4, 0x28 << 1, NE4_A_GPIO_Port, NE4_A_Pin},
		{&hi2c4, 0x36 << 1, NE4_B_GPIO_Port, NE4_B_Pin},
		{&hi2c1, 0x28 << 1, NE1_A_GPIO_Port, NE1_A_Pin},
		{&hi2c1, 0x36 << 1, NE1_B_GPIO_Port, NE1_B_Pin}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm4_main_thread_entry(ULONG thread_input);
void tx_cm4_i2c1_thread_entry(ULONG thread_input);
void tx_cm4_i2c4_thread_entry(ULONG thread_input);
void tx_cm4_uart_thread_entry(ULONG thread_input);
void Error_Handler(void);

HAL_StatusTypeDef read_cell(CellTypeDef *, uint8_t, uint8_t *, const uint32_t);
HAL_StatusTypeDef powerup_and_read(unsigned int, uint8_t *, const uint32_t);
HAL_StatusTypeDef write_word(CellTypeDef *, uint8_t *, uint8_t, uint8_t, uint8_t, const uint32_t);
HAL_StatusTypeDef exit_command_mode(CellTypeDef *, uint8_t *, const uint32_t);

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

  /*Allocate memory for main_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&cm4_main_thread, "tx_cm4_main_thread", tx_cm4_main_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /*Allocate memory for i2c1_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the i2c1 thread.  */
  ret = tx_thread_create(&cm4_i2c1_thread, "tx_cm4_i2c1_thread", tx_cm4_i2c1_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check i2c1 thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /*Allocate memory for i2c4_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the i2c4 thread.  */
  ret = tx_thread_create(&cm4_i2c4_thread, "tx_cm4_i2c4_thread", tx_cm4_i2c4_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check i2c4 thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /*Allocate memory for uart_thread_entry*/
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

  /* Check DEFAULT_STACK_SIZE allocation*/
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the uart thread.  */
  ret = tx_thread_create(&cm4_uart_thread, "tx_cm4_uart_thread", tx_cm4_uart_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
                         DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Check uart thread creation */
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/* USER CODE BEGIN 1 */

#if 0
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART2)
	{
		u2tc += 1;
		/* Notify the task that the transmission is complete. */
		// FIXME - should use a variable with the task currently waiting...
    vTaskNotifyGiveFromISR(DebugTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2htc += 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2rc += 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2hrc += 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2ec += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		setZero[0] = 1;
		setZero[1] = 1;
		setZero[2] = 1;
		setZero[3] = 1;
		//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hspi->Instance == SPI1)
	{
		/* Notify the task that the transmission is complete. */
		// FIXME - should use a variable with the task currently waiting...
    vTaskNotifyGiveFromISR(WriteLineTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}
#endif

#if 0
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c3)
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		memcpy(bridgeValue,dataBuf,2);
	}
}
#endif

int UART_Receive(unsigned char *dest, const unsigned char *rx, UART_HandleTypeDef *huart, unsigned int *uxcc, const unsigned int max)
{
	unsigned int cc = __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (*uxcc != cc)
	{
		HAL_UART_DMAPause(huart);
  	int len = 0;
		if (cc > *uxcc)
		{
			for (unsigned int i = max - *uxcc; i < max; i++)
				dest[len++] = rx[i];
			for (unsigned int i = 0; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		else
		{
			for (unsigned int i = max - *uxcc; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		HAL_UART_DMAResume(huart);
  	*uxcc = cc;
  	return len;
	}
	return 0;
}

HAL_StatusTypeDef read_cell(CellTypeDef *cell, uint8_t a, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = a;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_I2C_Master_Transmit(cell->handle, cell->address, dataBuf, 3, I2C_Timeout);
	memset(dataBuf, 0, 3);
	return HAL_I2C_Master_Receive(cell->handle, cell->address | 1, dataBuf, 3, I2C_Timeout);
}

HAL_StatusTypeDef powerup_and_read(unsigned int c, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0xA0;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	// we power up both sides, and then power off the side we do not want to talk to
	// this seems to do the trick - when powering up only one side we get no answer
	unsigned int o = c ^ 1;
	HAL_GPIO_WritePin(cell[c].gpio, cell[c].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(cell[o].gpio, cell[o].pin, GPIO_PIN_RESET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(cell[o].gpio, cell[o].pin, GPIO_PIN_SET);
	HAL_I2C_Master_Transmit(cell[c].handle, cell[c].address, dataBuf, 3, I2C_Timeout);
	return read_cell(cell + c, 2, dataBuf, I2C_Timeout);
}

HAL_StatusTypeDef write_word(CellTypeDef *cell, uint8_t *dataBuf, uint8_t a, uint8_t d1, uint8_t d2, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0x40 | a;
	dataBuf[1] = d1;
	dataBuf[2] = d2;
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(cell->handle, cell->address, dataBuf, 3, I2C_Timeout);
	HAL_Delay(15); // wait 15 ms according to DS
	return res;
}

HAL_StatusTypeDef exit_command_mode(CellTypeDef *cell, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0x80;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(cell->handle, cell->address, dataBuf, 3, I2C_Timeout);
	HAL_Delay(15); // wait another 15 ms to update EEPROM signature
	return res;
}

void tx_cm4_main_thread_entry(ULONG thread_input)
{
#if 0
	const int nbAddress = 5;
	const uint8_t address[5] = { 0x28, 0x36, 0x46, 0x48, 0x51 };
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
#endif
	uint8_t counted = 0;
	uint32_t low[4] = { 950, 950, 950, 950 };
	unsigned int c[4] = { 0, 0, 0, 0 };
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
#if 0
	/* debug I2C */
	GPIO_InitTypeDef  GPIO_InitStruct;
	HAL_I2C_MspDeInit(&hi2c1);
	HAL_I2C_MspDeInit(&hi2c4);
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitStruct.Pin = I2C1_SDA_Pin|I2C1_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = I2C4_SDA_Pin|I2C4_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C4_SDA_GPIO_Port, &GPIO_InitStruct);
	while (1)
	{
		HAL_GPIO_TogglePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin);
		HAL_GPIO_TogglePin(I2C4_SDA_GPIO_Port, I2C4_SDA_Pin);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND/50);
		HAL_GPIO_TogglePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin);
		HAL_GPIO_TogglePin(I2C4_SCL_GPIO_Port, I2C4_SCL_Pin);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND/50);
		HAL_GPIO_TogglePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin);
		HAL_GPIO_TogglePin(I2C4_SDA_GPIO_Port, I2C4_SDA_Pin);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND/50);
	}
	/* Power up load cells and detect them */
	while (counted != 4)
	{
		for (unsigned int i = 0; i < 4; i++)
		{
			printf("Detect cell %u", i + 1);
			for (unsigned int j = 0; j < nbAddress; j++)
			{
				cell[i].address = address[j] << 1;
				res = powerup_and_read(i, dataBuf, I2C_Timeout);
				HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_SET);
				HAL_Delay(10);
				if (res == HAL_OK && dataBuf[0] == 0x5A)
				{
					uint8_t a = (dataBuf[1] >> 2) & 3;
					uint8_t b = ((dataBuf[1] & 3) << 5) | (dataBuf[2] >> 3);
					printf(" %02X %u %02X", cell[i].address >> 1, a, b);
					break;
				} else {
					cell[i].address = 0;
					printf(" -------------");
				}
			}
			printf("\r\n");
		}
		printf("%02X %02X %02X %02X\r\n", cell[0].address >> 1, cell[1].address >> 1, cell[2].address >> 1, cell[3].address >> 1);
		// should now check whether we have all the cells ready
		for (unsigned int i = 0; i < 4; i += 2)
		{
			if (cell[i].address != 0 && cell[i].address == cell[i + 1].address)
			{
				// need to change one of the cell's address
				res = powerup_and_read(i, dataBuf, I2C_Timeout);
				if (res == HAL_OK && dataBuf[0] == 0x5A)
				{
					// confirmed, now change the address
					uint8_t new = address[0];
					if ((new << 1) == cell[i].address)
						new = address[1];
					write_word(cell + i, dataBuf,
										 2,
										 0x0C | (new >> 5),
										 0x06 | ((new << 3) & 0xff),
										 I2C_Timeout);
					exit_command_mode(cell + i, dataBuf, I2C_Timeout);
					cell[i].address = new << 1;
			  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
					HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_SET);
					printf("%u %02X => %02X\r\n", i, cell[i + 1].address >> 1, cell[i].address >> 1);
			  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
				}
				else
				{
					// should not happen...
					cell[i].address = 0;
					HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_SET);
					printf("%u BAD READ %d\r\n", i, res);
				}
			}
		}
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 5);
		counted = 0;
		for (unsigned int i = 0; i < 4; i++)
			if (cell[i].address != 0)
				counted += 1;
	}
#endif
	counted = 4;
	availableCells = counted;
  /* Infinite loop */
  for(;;)
  {
  	// should maybe setup a way to go back to probe and address adjust mode if there are errors
  	ULONG ticks = tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
		printf("WS %8lu",ticks);
		printf(" | %u %u %u %u",bridgeError[0],bridgeBadstatus[0],bridgeError[1],bridgeBadstatus[1]);
		printf(" %u %u %u %u",bridgeError[2],bridgeBadstatus[2],bridgeError[3],bridgeBadstatus[3]);
		for (unsigned int i = 0; i < 4; i++)
		{
			if (c[i] != bridgeCount[i])
			{
				uint32_t weight = (bridgeValue[i] >> 16) & 0x3fff;
				if (setZero[i])
				{
					low[i] = weight;
					setZero[i] = 0;
				}
				if (weight < low[i])
					weight = 0;
				else
					weight -= low[i];
				weight *= 5000;
				weight /= 14000;
				printf(" | %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
				uint32_t temp = (bridgeValue[i] >> 5) & 0x7ff;
				temp *= 2000;
				temp /= 2048; // just a guess at this point...
				temp -= 500;
				printf(" %2d.%01d C", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
				c[i] = bridgeCount[i];
			}
			else
				printf(" |                ");
		}
		printf(" | %u %4u %4u %6u\r\n", touchData[0], touchData[1], touchData[2], touchData[3]);
  	//BSP_LED_Toggle(LED_ORANGE);
  	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
  }
}

void tx_cm4_i2c1_thread_entry(ULONG thread_input)
{
	const unsigned int cellStart = 2;
	const unsigned int cellEnd = 4;
	/* Need to wait until I2C bus 1 peripherals are properly setup in WriteLine task */
	for(;;)
	{
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
		if (availableCells == 4)
			break;
	}
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	for (unsigned int i = cellStart; i < cellEnd; i++)
		HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	for (unsigned int i = cellStart; i < cellEnd; i++)
	{
		bridgeError[i] = 0;
		bridgeStale[i] = 0;
		bridgeCount[i] = 0;
		bridgeBadstatus[i] = 0;
	}
	/* Infinite loop */
	for(;;)
	{
		for (unsigned int i = cellStart; i < cellEnd; i++)
		{
			memset(dataBuf, 0, 4);
			res = HAL_I2C_Master_Receive(cell[i].handle, cell[i].address | 1, dataBuf, 4, I2C_Timeout);
			if (res != HAL_OK)
			{
				bridgeError[i] += 1;
				HAL_I2C_DeInit(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
				HAL_I2C_Init(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
			}
			else
			{
				uint8_t status = (dataBuf[0] >> 6) & 0x3;
				if (status == 0)
				{
					bridgeValue[i] = (dataBuf[0] << 24) | (dataBuf[1] << 16) | (dataBuf[2] << 8) | dataBuf[3];
					bridgeCount[i] += 1;
				} else if (status == 2)
					bridgeStale[i] += 1;
				else
					bridgeBadstatus[i] += 1;
			}
		}
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
	}
}

void tx_cm4_i2c4_thread_entry(ULONG thread_input)
{
	const unsigned int cellStart = 0;
	const unsigned int cellEnd = 2;
	/* Need to wait until I2C bus 3 peripherals are properly setup in WriteLine task */
	for(;;)
	{
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
		if (availableCells == 4)
			break;
	}
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	for (unsigned int i = cellStart; i < cellEnd; i++)
		HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	for (unsigned int i = cellStart; i < cellEnd; i++)
	{
		bridgeError[i] = 0;
		bridgeStale[i] = 0;
		bridgeCount[i] = 0;
		bridgeBadstatus[i] = 0;
	}
	memset((void *)touchData, 0, sizeof(touchData));
	/* Infinite loop */
	for(;;)
	{
		ULONG ticks_target = tx_time_get() + (TX_TIMER_TICKS_PER_SECOND / 20);
		for (unsigned int i = cellStart; i < cellEnd; i++)
		{
			memset(dataBuf, 0, 4);
			res = HAL_I2C_Master_Receive(cell[i].handle, cell[i].address | 1, dataBuf, 4, I2C_Timeout);
			if (res != HAL_OK)
			{
				bridgeError[i] += 1;
				HAL_I2C_DeInit(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
				HAL_I2C_Init(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
			}
			else
			{
				uint8_t status = (dataBuf[0] >> 6) & 0x3;
				if (status == 0)
				{
					bridgeValue[i] = (dataBuf[0] << 24) | (dataBuf[1] << 16) | (dataBuf[2] << 8) | dataBuf[3];
					bridgeCount[i] += 1;
				} else if (status == 2)
					bridgeStale[i] += 1;
				else
					bridgeBadstatus[i] += 1;
			}
		}
		/* this is active low */
		GPIO_PinState touch = HAL_GPIO_ReadPin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin);
		if (!touch || touchData[0] != 0)
		{
			BSP_LED_On(LED_GREEN);
			// read status register
			uint8_t nb_touch;
			uint8_t data[4];
			uint32_t touchX, touchY;
		  if (HAL_I2C_Mem_Read(&hi2c4, TS_I2C_ADDRESS, FT6X06_TD_STAT_REG, I2C_MEMADD_SIZE_8BIT, &nb_touch, 1, 1000) == HAL_OK)
		  {
		  	nb_touch &= FT6X06_TD_STATUS_BIT_MASK;
	      touchData[0] = nb_touch;
		    if(HAL_I2C_Mem_Read(&hi2c4, TS_I2C_ADDRESS, FT6X06_P1_XH_REG, I2C_MEMADD_SIZE_8BIT, data, 4, 1000) == HAL_OK)
		    {
		      touchX = (((uint32_t)data[0] & FT6X06_P1_XH_TP_BIT_MASK) << 8) | ((uint32_t)data[1] & FT6X06_P1_XL_TP_BIT_MASK);
		      touchY = (((uint32_t)data[2] & FT6X06_P1_YH_TP_BIT_MASK) << 8) | ((uint32_t)data[3] & FT6X06_P1_YL_TP_BIT_MASK);
		      touchData[1] = touchX;
		      touchData[2] = touchY;
		      touchData[3] += 1;
		    }
		  }
		  else
		  	touchData[0] = -1;
		}
		else
			BSP_LED_Off(LED_GREEN);
		ULONG ticks = tx_time_get();
		if (ticks < ticks_target)
			tx_thread_sleep(ticks_target - ticks);
	}
}

void tx_cm4_uart_thread_entry(ULONG thread_input)
{
	//UINT status;
	UCHAR read_buffer[32];
	CHAR data[] = "This is ThreadX working on STM32 CM4";

	printf("\r\n%s\r\nStarting Run on %s\r\n", data, _tx_version_id);
	/* Infinite Loop */
	for( ;; )
	{

		BSP_LED_Toggle(LED_ORANGE);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);

	}
	HAL_UART_Receive_DMA(&huart1, read_buffer, 32);
	unsigned int u2cc = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	//HAL_StatusTypeDef res;
	int inLen = 0;
	/* Infinite loop */
	for(;;)
	{
		int len = UART_Receive(input + inLen, read_buffer, &huart1, &u2cc, 32);
		if (len > 0)
		{
			printf("%.*s", len, input + inLen);
			inLen += len;
			if (input[inLen - 1] == '\r')
			{
				int cmdLen = inLen - 1;
				printf("\nReceived command '%.*s'\r\n# ", cmdLen, input);
				inLen = 0;
				if (strncmp((char *) input, "UART", cmdLen) == 0)
					printf("u2rc = %u u2hrc = %u u2tc = %u u2htc = %u u2ec = %u u2ic = %u u2cc = %u\r\n# ", u2rc, u2hrc, u2tc, u2htc, u2ec, u2ic, u2cc);
			}
		}
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
	}
}
/* USER CODE END 1 */
