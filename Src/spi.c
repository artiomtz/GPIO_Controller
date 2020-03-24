#include "main.h"
#include "string.h"

static void dataLatchDelay (int latchNum); // data latch delay

#define spiTimeOut 5
static GPIO_TypeDef *latchPort[] = {GPIOA, GPIOA, GPIOC}; // latch 1,2,3 ports
static uint16_t latchPin[] = {GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_4}; // latch 1,2,3 pins


// send data over spi
void spiTx (SPI_HandleTypeDef hspi1, uint8_t *dataSend, uint16_t numBytes) ////, uint8_t* sendData
{
	for (int i = 0 ; i < numBytes ; ++i)
	{
		HAL_SPI_Transmit(&hspi1, &dataSend[i], 1, spiTimeOut); // send 1 SPI byte (followed by ~1.5uSec delay)
		dataLatchDelay(i);
	}
}


// data latch delay
static void dataLatchDelay (int latchNum)
{
	HAL_GPIO_WritePin(latchPort[latchNum], latchPin[latchNum], 1); // latch data
	for (int i = 0 ; i < 10 ; ++i) {} // ~1uSec delay

	HAL_GPIO_WritePin(latchPort[latchNum], latchPin[latchNum], 0); // turn OFF latch data
	for (int i = 0 ; i < 10 ; ++i) {} // ~1uSec delay
}
