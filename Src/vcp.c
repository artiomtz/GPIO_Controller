#include "main.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdbool.h"
#include "usbd_def.h"
#include "flash.h"
#include <ctype.h>

static void copyErrorMsg (uint8_t *src1, int *len1, uint8_t const *src2, int len2); // buffer copy declaration
static uint32_t hexToBin (char *hex);			// phrase hex and update channel states buffer
static void cleanChannelDisplayBuffers (void);	// clean display channel configuration buffers
static void displayPortConfig (uint32_t portMode, char portLetter, int portNumber, char *displayPorts1); // add port configuration to display buffer
static void txUsbMessage (char *txUsbMsg);		// send a list to usb
static void txUsbMessage2 (uint8_t *txUsbMsg);	// send help list to usb
static void resetChannels (void);				// set default channel settings
static void linearApprox (float *slopeResult, float *axisPointResult, float x1, float y1, float x2, float y2); // linear approximation: find slope and axis point
static void charHexToInt (char *hexArray, uint8_t *uintArray); // convert char hex to int

// POWER /////////////////////////////////////////////////////////////////////////////////////////////////////////////
static float adc1Rin = 0;						// adc 1 top resistor of divider ------ set default value
static float adc1Rgnd = 1000;					// adc 1 bottom resistor of divider --- set default value
static float adc2Rin = 0;						// adc 2 top resistor of divider ------ set default value
static float adc2Rgnd = 1000;					// adc 2 bottom resistor of divider --- set default value
static float adcDropWarning = 13;				// high voltage warning --------------- set default value
static float adcDropError = 12;					// minimum allowed high voltage ------- set default value
static float adcScaleSlope[2] = {0, 0};			// ADC scale slopes (arbitrary init, 0 or negative is not good)
static float adcScaleAxis[2] = {0, 0};			// ADC scale axis points (arbitrary init, 0 or negative is not good)
static bool adc1ScaleDone = false;				// ADC1 scaling done
static bool adc2ScaleDone = false;				// ADC2 scaling done
static int sleepTime = 5*60; 					// default sleep time in seconds ------ set default value
static int sleepCount = 0;						// sleep timer
#define powerInPort GPIOA						/* power input port A*/
#define powerInPin GPIO_PIN_3					/* power input pin 3*/
#define powerOutPort GPIOE						/* power output port E*/
#define powerOutPin GPIO_PIN_14					/* power output pin 14*/
static bool powerInput = false;					// PA3 sync state - external interrupt EXT3
static bool powerOutput = false;				// PE14 pwr on enable
static bool powerMonitorEnable = true;			// adc-err: allows pwr-OFF PE14 at ADC error level
static bool displayWarning = false;				// warning display
static bool powerOnChange = false;				// check change in PA3 power-ON after interrupt
static bool powerEnableChange = false;			// check change in PE14 power enable

// ADC ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define vRefCalCount ((uint16_t*) ((uint32_t)0x1FFF7A2A)) /* internal 1.2v reference measurement (by us)*/
static uint32_t vRefCount = 0; 					// internal 1.2v reference measurement (by processor)
static const float adcResolution = 4095;		// 12 bit resolution = max is 2^12-1
static uint32_t adc1Measure = 5000;				// ADC1 measured value (0 to 4095)
static uint32_t adc2Measure = 5000;				// ADC2 measured value (0 to 4095)
static float processorVoltage = 0;				// calculated processor voltage
static float adc1Result = 0;					// ADC1 calculated value
static float adc2Result = 0;					// ADC1 calculated value
static _Bool adcResultDisplay = false;			// display ADC result every timer tick

// Channels //////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CHANNELS 32 /* number of channels */
static uint32_t Bport = 0;						// B port
static uint32_t Dport = 0;						// D port
static uint32_t Eport = 0;						// E port
GPIO_InitTypeDef gpioInit;						// configure GPIO
static int channels[32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}; // channels indices to physical ports
struct Port
{
	uint32_t 		*chPort; 					// port name
	GPIO_TypeDef 	*portRegs; 					// port register
	uint16_t 		chPin; 						// channel pin
	char			portLetter; 				// port letter
	int				portNumber; 				// port number
	uint32_t		portMode; 					// port input / output
};

static struct Port ports[CHANNELS] = { // array of ports
    {&Dport, GPIOD, GPIO_PIN_0 , 'D',  0, GPIO_MODE_OUTPUT_PP}, // PD0
    {&Dport, GPIOD, GPIO_PIN_1 , 'D',  1, GPIO_MODE_OUTPUT_PP}, // PD1
	{&Dport, GPIOD, GPIO_PIN_2 , 'D',  2, GPIO_MODE_OUTPUT_PP}, // PD2
	{&Dport, GPIOD, GPIO_PIN_3 , 'D',  3, GPIO_MODE_OUTPUT_PP}, // PD3
	{&Dport, GPIOD, GPIO_PIN_4 , 'D',  4, GPIO_MODE_OUTPUT_PP}, // PD4
	{&Dport, GPIOD, GPIO_PIN_5 , 'D',  5, GPIO_MODE_OUTPUT_PP}, // PD5
	{&Dport, GPIOD, GPIO_PIN_6 , 'D',  6, GPIO_MODE_OUTPUT_PP}, // PD6
    {&Dport, GPIOD, GPIO_PIN_7 , 'D',  7, GPIO_MODE_OUTPUT_PP}, // PD7
    {&Dport, GPIOD, GPIO_PIN_8 , 'D',  8, GPIO_MODE_OUTPUT_PP}, // PD8
	{&Dport, GPIOD, GPIO_PIN_9 , 'D',  9, GPIO_MODE_OUTPUT_PP}, // PD9
	{&Dport, GPIOD, GPIO_PIN_10, 'D', 10, GPIO_MODE_OUTPUT_PP}, // PD10
	{&Dport, GPIOD, GPIO_PIN_11, 'D', 11, GPIO_MODE_OUTPUT_PP}, // PD11
	{&Dport, GPIOD, GPIO_PIN_12, 'D', 12, GPIO_MODE_OUTPUT_PP}, // PD12
	{&Dport, GPIOD, GPIO_PIN_13, 'D', 13, GPIO_MODE_OUTPUT_PP}, // PD13
    {&Dport, GPIOD, GPIO_PIN_14, 'D', 14, GPIO_MODE_OUTPUT_PP}, // PD14

    {&Bport, GPIOB, GPIO_PIN_0 , 'B',  0, GPIO_MODE_OUTPUT_PP}, // PB0
	{&Bport, GPIOB, GPIO_PIN_1 , 'B',  1, GPIO_MODE_OUTPUT_PP}, // PB1
	{&Bport, GPIOB, GPIO_PIN_2 , 'B',  2, GPIO_MODE_OUTPUT_PP}, // PB2
	{&Bport, GPIOB, GPIO_PIN_3 , 'B',  3, GPIO_MODE_OUTPUT_PP}, // PB3
	{&Bport, GPIOB, GPIO_PIN_4 , 'B',  4, GPIO_MODE_OUTPUT_PP}, // PB4
	{&Bport, GPIOB, GPIO_PIN_5 , 'B',  5, GPIO_MODE_OUTPUT_PP}, // PB5
    {&Bport, GPIOB, GPIO_PIN_6 , 'B',  6, GPIO_MODE_OUTPUT_PP}, // PB6
    {&Bport, GPIOB, GPIO_PIN_7 , 'B',  7, GPIO_MODE_OUTPUT_PP}, // PB7
	{&Bport, GPIOB, GPIO_PIN_8 , 'B',  8, GPIO_MODE_OUTPUT_PP}, // PB8
	{&Bport, GPIOB, GPIO_PIN_9 , 'B',  9, GPIO_MODE_OUTPUT_PP}, // PB9
	{&Bport, GPIOB, GPIO_PIN_10, 'B', 10, GPIO_MODE_OUTPUT_PP}, // PB10
	{&Bport, GPIOB, GPIO_PIN_11, 'B', 11, GPIO_MODE_OUTPUT_PP}, // PB11
	{&Bport, GPIOB, GPIO_PIN_12, 'B', 12, GPIO_MODE_OUTPUT_PP}, // PB12
    {&Bport, GPIOB, GPIO_PIN_13, 'B', 13, GPIO_MODE_OUTPUT_PP}, // PD13
    {&Bport, GPIOB, GPIO_PIN_14, 'B', 14, GPIO_MODE_OUTPUT_PP}, // PD14
	{&Bport, GPIOB, GPIO_PIN_15, 'B', 15, GPIO_MODE_OUTPUT_PP}, // PD15

	{&Eport, GPIOE, GPIO_PIN_15, 'E', 15, GPIO_MODE_OUTPUT_PP}, // PE15
};
static const uint16_t gpioPins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

static uint32_t chStates = 0; 					// channel states - SAVED IN FLASH
static char displayChannels1[64]; 				// display channel configuration
static char displayChannels2[64]; 				// display channel configuration
static char displayChannels3[64]; 				// display channel configuration
static char displayChannels4[64]; 				// display channel configuration
static char displayChannels5[64]; 				// display channel configuration
static char displayChannels6[64]; 				// display channel configuration
static char displayChannels7[64]; 				// display channel configuration
static char displayPorts1[64]; 					// display port mode
static char displayPorts2[64]; 					// display port mode
static char displayPorts3[64]; 					// display port mode
static char displayPorts4[64]; 					// display port mode
static char displayPorts5[64]; 					// display port mode
static char displayPorts6[64]; 					// display port mode
static char displayPorts7[64]; 					// display port mode
static char displayPorts8[64]; 					// display port mode
static char displayPorts9[64]; 					// display port mode
static char displayPorts10[64];					// display port mode
static char displayPorts11[64]; 				// display port mode
static char portCnfg[10]; 						// print port mode

// Rx Messages ///////////////////////////////////////////////////////////////////////////////////////////////////////
static char const newLineRxMsg[] = "\r";
static char const help1RxMsg[] = "?\r";
static char const help2RxMsg[] = "help\r";
static char const help3RxMsg[] = "help?\r";
static char const idRxMsg[] = "id?\r";
static char const statusAllRxMsg[] = "status all\r";
static char const cnfgChAllMsg[] = "show-config ch all\r";
static char const cnfgPortAllMsg[] = "show-config port all\r";
static char const cnfgDefaultMsg[] = "config default\r";

// Tx Messages ///////////////////////////////////////////////////////////////////////////////////////////////////////
static char const startMsg[] = "Virtual Com Port is ready to receive commands.\r\n";
static char const NewLine[] = "\r\n";			// new line
static char const NewLineCli[] = "\r\n> ";		// new line cli
static char const NewLineCli2[] = "> ";			// new line cli
static char const idtxUsbMsg[] = "  GPIO Controller on the STM32F407VGT6 board\r\n> ";

// Help Menu /////////////////////////////////////////////////////////////////////////////////////////////////////////
struct Help
{
	uint8_t helpMessage[64];
};
#define numbHelpLines 23 /* number of help lines */
static struct Help helpList[numbHelpLines] = {
	{" ------------------------------------------------------------\r\n"},
	{" ID?                   Displays the board ID\r\n"},
	{" status all            Shows the status of all channels\r\n"},
	{" status <N>            Shows the status of a channel\r\n"},
	{" ch<N> <0/1>           Sets a state of a channel (ch13 1)\r\n"},
	{" ch 0x<N>              Sets states of all channels (ch 0x5aF)\r\n"},
	{" pwr <on/off>          Regulates supply via PE14\r\n"},
	{" sleep <N>             Set timer in minutes on PE14 (sleep 7)\r\n"},
	{" adc-err <on/off>      Allows pwr-OFF PE14 at ADC error level\r\n"},
	{" adc-lvl <0/1> <w> <e> ADC display, warning, error V levels\r\n"},
	{" adc<N> <Rin> <Rgnd>   Set ADC resistors in Ohms (adc1 24 68)\r\n"},
	{" adc<N>-scl x1y1x2y2   Set ADC scale slope high/low points\r\n"},
	{" show-config ch<N>     Shows the port name of a channel\r\n"},
	{" show-config ch all    Shows the port names of all channels\r\n"},
	{" echo <on/off>         Regulates echo\r\n"},
	{" show-config port <N>  Shows a port configuration\r\n"},
	{" show-config port all  Shows all ports configuration\r\n"},
	{" config port <N> <i/o> Configure a port as input or output\r\n"},
	{" config ch<N> P<x>     Set channel to port (config ch17 pd0)\r\n"},
	{" config default        Configure default start-up settings\r\n"},
	{" save                  Saves ADC and channel config in flash\r\n"},
	{" spi-send 0x<N> 0x<N>  Send 1-3 packages over SPI + latch \r\n"},
	{" --------------------------------------------------------\r\n\n> "},
};
static bool longMsgHelp = false; 				// long message flag
static bool longMsgCh = false;					// long message 2 flag
static bool longMsgPort = false; 				// long message 3 flag

// Flags /////////////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t txUsbState = USBD_BUSY; 			// usb state
static _Bool rxFlag = false; 					// data received
static _Bool procFlag = false; 					// processing data
static _Bool txFlag = false; 					// transmitting data
static _Bool echoEnable = true; 				// echo flag

// Buffers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RXBUFSIZE 64							/* Rx buffer size */
#define PROCBUFSIZE 64							/* Process buffer size */
#define MAXPACKAGESIZE 64						/* max bytes in tx/rx package */
#define TXBUFSIZE 64							/* Tx buffer size */
static uint8_t rxBuffer[RXBUFSIZE]; 			// Rx buffer
static int rxBufLen = 0; 						// Rx buffer lenght
static uint8_t procBuffer[PROCBUFSIZE]; 		// Rx buffer
static int procBufLen = 0; 						// Rx buffer lenght
static uint8_t txBuffer[TXBUFSIZE]; 			// Tx buffer
static int txBufLen = 0; 						// Tx buffer lenght

// Errors ////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define errorLedPort GPIOE						/* error led */
#define errorLedPin GPIO_PIN_13					/* error led */
typedef enum errorr
{
  NoError, 										// no errors
  ErrorUnknown, 								// unknown error
  ErrorSize, 									// buffer size error
  ErrorCommand, 								// invalid input
  ErrorChSyntax, 								// wrong channel command
  ErrorChHexSyntax, 							// wrong hex channel command
  ErrorStatusSyntax, 							// wrong status command
  ErrorPower, 									// wrong power command
  ErrorUSB, 									// usb error
  ErrorADC, 									// adc error
  ErrorAdcEnable, 								// adc monitoring enable
  ErrorResistors, 								// resistors error
  ErrorSleep, 									// sleep error
  ErrorShowChConfig, 							// show channel config error
  ErrorShowPortConfig, 							// show port config error
  ErrorPortConfig, 								// port config error
  ErrorChConfig, 								// ch config error
  ErrorADCconfig,								// adc config error
  ErrorSave, 									// flash save error
  ErrorADCcalibration,							// adc calibration error
  ErrorEcho, 									// echo command error
  ErrorSpi										// spi error
} Error;
Error ERRORTYPE = NoError;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ADC calibration
void adcCalibration(ADC_HandleTypeDef *hadc)
{
	if (HAL_ADC_PollForConversion(hadc, 5) == HAL_OK) // if successful adc conversion
		vRefCount = HAL_ADC_GetValue(hadc); // 1.2v internal reference measurement
	else // error
		ERRORTYPE = ErrorADC;

	processorVoltage = 3.3 * (((float)vRefCount / (float)(*vRefCalCount))); // calculate processor voltage
	vRefCount = 0; // clear
}



// check ADC measurements
void checkMeasurement(ADC_HandleTypeDef *hadc, int adcNum)
{
	if (HAL_ADC_PollForConversion(hadc, 5) == HAL_OK) // if successful ADC conversion
	{
		if (adcNum == 1) // ADC 1
			adc1Measure = HAL_ADC_GetValue(hadc); // ADC 1 measurement
		else // ADC 2
			adc2Measure = HAL_ADC_GetValue(hadc); // ADC 2 measurement
	}
	else // ADC measurement error
		ERRORTYPE = ErrorADC;

	if ((adc1Measure != 5000) && (adc2Measure != 5000) && (ERRORTYPE == NoError)) // both ADC done and measurement enabled
	{
		adc1Result = ((adc1Rin + adc1Rgnd) / adc1Rgnd) * (adc1Measure / adcResolution) * processorVoltage; // calculate value 1
		adc2Result = ((adc2Rin + adc2Rgnd) / adc2Rgnd) * (adc2Measure / adcResolution) * processorVoltage; // calculate value 2

		if (adc1ScaleDone) // if scaling done, recalculate
		{
			adc1Result = (((adc1Measure / adcResolution) * processorVoltage) - adcScaleAxis[0]) / adcScaleSlope[0];  // ADC1: x = (y-b)/m
			if (adc1Result < 0) // linear correction due to opAmp behavior
				adc1Result = 0;
		}
		if (adc2ScaleDone) // if scaling done, recalculate
		{
			adc2Result = (((adc2Measure / adcResolution) * processorVoltage) - adcScaleAxis[1]) / adcScaleSlope[1];  // ADC1: x = (y-b)/m
			if (adc2Result < 0) // linear correction due to opAmp behavior
				adc2Result = 0;
		}

		if (((adc1Result < adcDropError) || (adc2Result < adcDropError)) && (powerOutput) && (powerMonitorEnable)) // power drop
		{
			if (echoEnable)
			{
				sprintf(txBuffer, "\r\n  ADC1: %.2fv !\r\n  ADC2: %.2fv !\r\n  PE14: OFF\r\n> ",adc1Result, adc2Result);
				txBufLen = strlen(txBuffer);
				txFlag = true;
			}
			HAL_GPIO_WritePin(powerOutPort, powerOutPin, 0); // PE14 power OFF
			HAL_GPIO_WritePin(errorLedPort, errorLedPin, 1); // error led
			powerOutput = false;
			powerOutEnableSet(false); // power ON PE14 flag in main
			displayWarning = false;
		}

		else if (((adc1Result < adcDropWarning) || (adc2Result < adcDropWarning)) && (powerOutput) && (powerMonitorEnable) && (displayWarning)) // power drop
		{
			if (echoEnable)
			{
				sprintf(txBuffer, "\r\n  ADC1: %.2fv\r\n  ADC2: %.2fv\r\n  ADC ALERT LEVEL!\r\n> ",adc1Result, adc2Result);
				txBufLen = strlen(txBuffer);
				txFlag = true;
			}
			displayWarning = false;
		}

		else if (powerEnableChange) // power enabled / disabled
		{
			sprintf(txBuffer, "\r\n  ADC1: %.2fv\r\n  ADC2: %.2fv\r\n> ",adc1Result, adc2Result);
			txBufLen = strlen(txBuffer);
			txFlag = true;
		}
		powerEnableChange = false;
		adc1Measure = 5000; // clear ADC result
		adc2Measure = 5000; // clear ADC result
	}
}



// updates port states
static void updateChannels()
{
	uint32_t ch = chStates;
	Bport = 0;
	Dport = 0;
	Eport = 0;

	for (int i = 0; i < CHANNELS; ++i)
	{
		int portIdx = channels[i];
		struct Port const *portCfg = &ports[portIdx];
		if (ch & (1UL << i)) // set '1'
		{
			*(portCfg->chPort) |= portCfg->chPin;
	 		*(portCfg->chPort) &= ~(portCfg->chPin << 16);
		}
	    else // set '0'
	    {
	 		*(portCfg->chPort) |= (portCfg->chPin << 16);
	 		*(portCfg->chPort) &= ~(portCfg->chPin);
	    }
	}
	GPIOD->BSRR = Dport;
	GPIOB->BSRR = Bport;
	GPIOE->BSRR = Eport;
}



// power-up welcome message + initialize ports to 0
void powerUpInit(void)
{
	HAL_GPIO_WritePin(powerOutPort, powerOutPin, 0); // disable power output
	HAL_GPIO_WritePin(errorLedPort, errorLedPin, 0); // error led off
	chStates = 0; // initial channel state
	updateChannels(); // set initial channel state
	cleanChannelDisplayBuffers(); // clean display channel configuration buffers

	if ((_Bool)HAL_GPIO_ReadPin(powerInPort, powerInPin)) // check if power is ON PA3
	  powerInput = true;
	else // power is OFF
	  powerInput = false;

	HAL_Delay(5000); // startup-delay for testing. Debug mode
	txUsbState = USBD_BUSY;
	while (txUsbState == USBD_BUSY) // while usb is busy, wait
		txUsbState = CDC_Transmit_FS((uint8_t*)startMsg, sizeof(startMsg)-1); // display welcome message
}



// Rx callback from usb - received data
void usbPackageRx(uint8_t *buf, uint32_t len)
{
	if ((echoEnable) && (((rxBufLen > 0) && (buf[0] == '\177')) || ((buf[0] != '\177')))) // prevent deleting prompt
		CDC_Transmit_FS(buf, len); // echo

	if (rxBufLen <= MAXPACKAGESIZE) // Legal size
	{
		rxFlag = true; // start receiving
		if ((rxBufLen > 0) && (buf[0] == '\177')) // backspace
		{
			rxBuffer[rxBufLen-1] = '\0'; // clear
			rxBufLen--;
		}
		else if (buf[0] != '\177')// data / not a backspace
		{
			for (int i=0 ; i<len ; i++) // store received data
			{
				rxBuffer[rxBufLen] = buf[i]; // store received character / data
				rxBufLen++;
				if (rxBufLen > MAXPACKAGESIZE) // check size as storing
					ERRORTYPE = ErrorSize;
			}
		}
	}
	else // size error
		ERRORTYPE = ErrorSize;
	if (ERRORTYPE == ErrorSize) // if there was an error, clear rx buffer
	{
		for (int i=0 ; i<rxBufLen ; i++) // clear rx buffer
			rxBuffer[i] = '\0'; // clear
		rxBufLen = 0; // reset
	}
}



// move complete received data to process buffer
void receiveData(void)
{
	if ((rxFlag) && (rxBuffer[rxBufLen-1] == '\r')) // complete line received
	{
		sleepCount = 0; // reset sleep timer
		for (int i=0 ; i<rxBufLen ; i++) // copy to process buffer
		{
			procBuffer[procBufLen] = tolower(rxBuffer[i]); // copy + lower case
			rxBuffer[i] = '\0'; // clear Rx buffer
			procBufLen++;
		}
		rxBufLen = 0; // reset buffer length
		procFlag = true; // ready to process
		rxFlag = false; // done receiving
		txFlag = true; // send new line + prompt
		HAL_GPIO_WritePin(errorLedPort, errorLedPin, 0); // error led

		for (int i=0 ; i < sizeof(NewLine)-1 ; i++) // new line before printing
		{
			txBuffer[txBufLen] = NewLine[i];
			txBufLen++;
		}
	}
}



// process received data
void processData(void)
{
	if (powerOnChange) // power turned ON/oFF
	{
		powerOnChange = false;
		if (!(powerInput)) // power OFF
			sprintf(txBuffer, "PA3 power: OFF\r\n> ");
		else // power ON
			sprintf(txBuffer, "PA3 power: ON\r\n> ");
		txBufLen = strlen(txBuffer);
		txFlag = true; // ready to send
	}

	else if (procFlag) // complete data received
	{
		char Rx[procBufLen+1]; // Rx will have the command (Rx is in the size of the command - for comparison function)
		int i=0; // loop index
		for (i=0 ; i<procBufLen ; i++) // copy command part from buffer
		{
			Rx[i] = procBuffer[i];
			procBuffer[i] = '\0'; // clear buffer
		}
		procBufLen = 0; //reset length
		Rx[i] = '\0'; // for correct string comparison, last char is null


		if (!(strcmp(Rx, newLineRxMsg))) // new line
		{
			for (int i=0 ; i < strlen(NewLineCli2) ; i++) // add to tx buffer
			{
				txBuffer[txBufLen] = NewLineCli2[i];
				txBufLen++;
			}
		}


		else if ((!(strcmp(Rx, help1RxMsg))) || (!(strcmp(Rx, help2RxMsg))) || (!(strcmp(Rx, help3RxMsg)))) // "help" / "help?" / "?"
		{
			longMsgHelp = true; // help message is long
		}


		else if (!(strcmp(Rx, idRxMsg))) // "id"
		{
			for (int i=0 ; i < strlen(idtxUsbMsg) ; i++) // add to tx buffer
			{
				txBuffer[txBufLen] = idtxUsbMsg[i];
				txBufLen++;
			}
		}


		else if (!(strcmp(Rx, statusAllRxMsg))) // "status all"
			{
				sprintf(txBuffer,"\r\n  All channels status: ");
				uint32_t ch = chStates;
				for (int i = CHANNELS ; i ; --i)
				{
					if (ports[channels[i-1]].portMode == GPIO_MODE_INPUT) // input
					{
						if (HAL_GPIO_ReadPin(ports[channels[i-1]].portRegs, ports[channels[i-1]].chPin)) // input high
							strcat(txBuffer, "H");
						else // input low
							strcat(txBuffer, "L");
					}

					else // output
					{
						if (ch & 0x80000000UL)
							strcat(txBuffer, "1");
						else
							strcat(txBuffer, "0");
					}
					ch <<= 1;
				}
				strcat(txBuffer,NewLineCli); // add new line after
				txBufLen = strlen(txBuffer);
			}


		else if ((Rx[0] == 'c') && (Rx[1] == 'h') && (Rx[4] != 'x')) // channel command "ch X x\r" or "ch XX x\r"
		{
			uint32_t chNum = 0; // channel number
			int chState = 0; // channel state
			if ((2 != sscanf(Rx, "%*[^0123456789]%lu%*[^0123456789]%d",&chNum, &chState)) || (chNum < 0) || (chNum > CHANNELS-1) || (!((chState == 0) || (chState == 1)))) // error
				ERRORTYPE = ErrorChSyntax;
			else // no error
			{
				if (ports[channels[chNum]].portMode == GPIO_MODE_INPUT) // configured as input
					sprintf(txBuffer,"\r\n  Cannot set: Channel %lu is INPUT\r\n> ",chNum);
				else // configured as output
				{
					if (chState == 0)
						chStates &= ~(1 << chNum);
					else
						chStates |= (1 << chNum);
					updateChannels();
					sprintf(txBuffer,"\r\n  Channel %lu set to: %d\r\n> ",chNum, chState);
				}
				txBufLen = strlen(txBuffer);
			}
		}


		else if ((Rx[0] == 'c') && (Rx[1] == 'h')) // channel command "ch 0xNN....NN\r"
		{
			if ((Rx[2] != ' ') || (Rx[3] != '0') || (Rx[4] != 'x') || (sizeof(Rx) < 8) || (sizeof(Rx) > 15)) // error
					ERRORTYPE = ErrorChHexSyntax;
			else // no error
			{
				char chHex[8]; // store channel position
				int j = sizeof(chHex)-1; // chHex index

				for (int i = sizeof(Rx)-3 ; i > 4 ; i--) // parse channel number
				{
					if (!(((Rx[i] >= '0') && (Rx[i] <= '9')) || ((Rx[i] >= 'a') && (Rx[i] <= 'f')))) // error
						ERRORTYPE = ErrorChHexSyntax;
					else // no error
					{
						chHex[j] = Rx[i];
						j--;
					}
				}
				while(j >= 0) // fill in chHex with zeros
				{
					chHex[j] = 0;
					j--;
				}
				if (ERRORTYPE == NoError)
				{
					chStates = hexToBin(chHex); // phrase hex and update channel states buffer
					updateChannels();
					sprintf(txBuffer,"\r\n  Channels set: "); // send result to tx buffer
					uint32_t ch = chStates;
					for (int i = CHANNELS; i; --i)
					{
						if (ports[channels[i-1]].portMode == GPIO_MODE_INPUT) // input
						{
							if (HAL_GPIO_ReadPin(ports[channels[i-1]].portRegs, ports[channels[i-1]].chPin)) // input high
								strcat(txBuffer, "H");
							else // input low
								strcat(txBuffer, "L");
						}
						else // output
						{
							if (ch & 0x80000000UL) // '1'
								strcat(txBuffer, "1");
							else // '0'
								strcat(txBuffer, "0");
						}
						ch <<= 1;
					}
					strcat(txBuffer,NewLineCli); // add new line after
					txBufLen = strlen(txBuffer);
				}
			}
		}


		else if ((Rx[0] == 's') && (Rx[1] == 't') && (Rx[2] == 'a') && (Rx[3] == 't') && (Rx[4] == 'u') && (Rx[5] == 's')) // status command "status X\r" or "status XX\r"
		{
			int chNum = 0; // channel number
			if ((1 != sscanf(Rx, "%*[^0123456789]%d",&chNum)) || (chNum < 0) || (chNum > CHANNELS-1)) // error
				ERRORTYPE = ErrorStatusSyntax;
			else // no error
			{
				if (ports[channels[chNum]].portMode == GPIO_MODE_INPUT) // input
				{
					if (HAL_GPIO_ReadPin(ports[channels[chNum]].portRegs, ports[channels[chNum]].chPin))
						sprintf(txBuffer,"\r\n  Channel %d (input) status: HIGH\r\n> ", chNum);
					else
						sprintf(txBuffer,"\r\n  Channel %d (input) status: LOW\r\n> ", chNum);
				}
				else // output
					sprintf(txBuffer,"\r\n  Channel %d (output) status: %c\r\n> ", chNum, (chStates & (1 << chNum)) ? '1' : '0');
				txBufLen = strlen(txBuffer);
			}
		}


		else if ((Rx[0] == 's') && (Rx[1] == 'l') && (Rx[2] == 'e') && (Rx[3] == 'e') && (Rx[4] == 'p')) // sleep command "sleep x\r" or "sleep xx\r"
		{
			int timerNum = 0; // sleep time in minutes
			if ((1 != sscanf(Rx, "%*[^0123456789]%d",&timerNum)) || (timerNum <= 0) || (timerNum > 1000)) // error
				ERRORTYPE = ErrorSleep;
			else // no error
			{
				sleepTime = timerNum*60; // update sleep time in minutes
				sprintf(txBuffer,"\r\n  Sleep timer updated: %d minutes\r\n> ",timerNum);
				txBufLen = strlen(txBuffer);
				sleepCount = 0; // reset timer
			}
		}


		else if ((Rx[0] == 'p') && (Rx[1] == 'w') && (Rx[2] == 'r')) // power command "pwr on\r" or "pwr off\r"
		{
			if ((Rx[3] != ' ') || (!((Rx[6] == '\r') || (Rx[7] == '\r')))) // error
				ERRORTYPE = ErrorPower;
			else // no error
			{
				if ((Rx[4] == 'o') && (Rx[5] == 'n') && (Rx[6] == '\r')) // power on
				{
					HAL_GPIO_WritePin(powerOutPort, powerOutPin, 1); // power ON PE14 ---------------
					powerOutput = true; // power ON PE14 flag
					powerOutEnableSet(true); // power ON PE14 flag in main
					sprintf(txBuffer,"\r\n  Control via PE14: ON\r\n  ");
					txBufLen = strlen(txBuffer);
					displayWarning = true; // enable warning
					startADCs(true); // launch ADCs to measure
					powerEnableChange = true; // enable ADC measurement check
				}
				else if ((Rx[4] == 'o') && (Rx[5] == 'f') && (Rx[6] == 'f')) // power off ---------------
				{
					HAL_GPIO_WritePin(powerOutPort, powerOutPin, 0); // power OFF PE14
					powerOutput = false; // power OFF PE14 flag
					powerOutEnableSet(false); // power ON PE14 flag in main
					sprintf(txBuffer,"\r\n  Control via PE14: OFF\r\n  ");
					txBufLen = strlen(txBuffer);
					startADCs(true); // launch ADCs to measure
					powerEnableChange = true; // enable ADC measurement check
				}
				else // error
					ERRORTYPE = ErrorPower;
			}
		}


		else if ((Rx[0] == 'a') && (Rx[1] == 'd') && (Rx[2] == 'c') && (Rx[3] == '-') && (Rx[4] == 'e') && (Rx[5] == 'r') && (Rx[6] == 'r')) // power command "adc-err on\r" or "adc-err off\r"
		{
			if (!((Rx[7] == ' ') || (Rx[10] == '\r') || (Rx[11] == '\r'))) // error
				ERRORTYPE = ErrorAdcEnable;
			else // no error
			{
				displayWarning = true; // enable warning
				if ((Rx[8] == 'o') && (Rx[9] == 'n') && (Rx[10] == '\r')) // PE14 monitoring on
				{
					powerMonitorEnable = true; // adc error not allowed
					sprintf(txBuffer,"\r\n  ADC error monitoring: Enabled\r\n> ");
					txBufLen = strlen(txBuffer);
				}
				else if ((Rx[8] == 'o') && (Rx[9] == 'f') && (Rx[10] == 'f')) // PE14 monitoring off
				{
					powerMonitorEnable = false; // power off
					sprintf(txBuffer,"\r\n  ADC error monitoring: Disabled\r\n> ");
					txBufLen = strlen(txBuffer);
				}
				else // error
					ERRORTYPE = ErrorAdcEnable;
			}
		}


		else if ((Rx[0] == 'a') && (Rx[1] == 'd') && (Rx[2] == 'c') && (Rx[3] == '-') && (Rx[4] == 'l') && (Rx[5] == 'v') && (Rx[6] == 'l')) // power command "adc-lvl <0/1> <w> <e>\r"
		{
			int adcDisplay = 0, argErr = -1; // error if more than 3 arguments;
			float adcWarning = 0.0, adcError = 0.0;

			int argsReturn = sscanf (Rx, "%*[^0123456789]%d%*[^0123456789]%f%*[^0123456789]%f%*[^0123456789]%d",&adcDisplay, &adcWarning, &adcError, &argErr);
			if ((argsReturn <  1) || (argErr != -1)) // count arguments
				ERRORTYPE = ErrorADCconfig;
			else if ((adcDisplay < 0) || (adcDisplay > 1) || (adcWarning < 0) || (adcWarning > 1000) || (adcError < 0) || (adcError > 1000))
				ERRORTYPE = ErrorADCconfig;
			else // no error
			{
				if (adcDisplay) // adc display argument
					adcResultDisplay = true;
				else
					adcResultDisplay = false;

				if (argsReturn > 1) // warning range updated
					adcDropWarning = adcWarning;
				if (argsReturn > 2) // error range updated
					adcDropError = adcError;

				displayWarning = true; // enable warning
				sprintf(txBuffer,"\r\n  ADC config:  Display = %c  Warning = %.1f  Error = %.1f\r\n> ", (adcDisplay) ? 'Y':'N', adcDropWarning, adcDropError);
				txBufLen = strlen(txBuffer);
			}
		}


		else if ((Rx[0] == 'a') && (Rx[1] == 'd') && (Rx[2] == 'c') && (Rx[4] == '-') && (Rx[5] == 's') && (Rx[6] == 'c') && (Rx[7] == 'l')) // adc scale factor command "adcN-scl X1 Y1 X2 Y2\r"
		{
			int adcNum = 0, er = -1; // error if more than 5 numbers
			float adcX1 = 0, adcY1 = 0, adcX2 = 0, adcY2 = 0;

			if (5 != sscanf(Rx, "%*[^0123456789]%d%*[^0123456789]%f%*[^0123456789]%f%*[^0123456789]%f%*[^0123456789]%f%*[^0123456789]%d",&adcNum, &adcX1, &adcY1, &adcX2, &adcY2, &er)) // get 5 numbers
				ERRORTYPE = ErrorADCcalibration;
			else if ((adcNum < 1) || (adcNum > 2) || (adcX1 < 0) || (adcX1 > 1000) || (adcY1 < 0) || (adcY1 > 1000) || (adcX2 < 0) || (adcX2 > 1000) || (adcY2 < 0) || (adcY2 > 1000) || (er != -1))
				ERRORTYPE = ErrorADCcalibration;
			else // no error
			{
				if (adcNum == 1) // ADC1
				{
					linearApprox(&adcScaleSlope[0], &adcScaleAxis[0], adcX1, adcY1, adcX2, adcY2); // find slope and axis point
					adc1ScaleDone = true;
					sprintf(txBuffer,"\r\n  ADC%d  scaled slope: %.2f  axis point: %.2f\r\n> ",adcNum, adcScaleSlope[0], adcScaleAxis[0]);
				}
				else // ADC2
				{
					linearApprox(&adcScaleSlope[1], &adcScaleAxis[1], adcX1, adcY1, adcX2, adcY2); // find slope and axis point
					sprintf(txBuffer,"\r\n  ADC%d  scaled slope: %.2f  axis point: %.2f\r\n> ",adcNum, adcScaleSlope[1], adcScaleAxis[1]);
					adc2ScaleDone = true;
				}
				txBufLen = strlen(txBuffer);
			}
		}


		else if ((Rx[0] == 'a') && (Rx[1] == 'd') && (Rx[2] == 'c')) // adc resistors command "adcN X X\r"
		{
			int adcNum = 0, Rin = 0, Rgnd = 0 , er = -1; // error if more than 3 numbers
			if (3 != sscanf(Rx, "%*[^0123456789]%d%*[^0123456789]%d%*[^0123456789]%d%*[^0123456789]%d",&adcNum, &Rin, &Rgnd, &er)) // get 3 numbers
				ERRORTYPE = ErrorResistors;
			else if ((adcNum < 1) || (adcNum > 2) || (Rin < 0) || (Rin > 100000000) || (Rgnd < 1) || (Rgnd > 100000000) || (er != -1))
				ERRORTYPE = ErrorResistors;
			else // no error
			{
				if (adcNum == 1) // ADC1
				{
					adc1Rin = (float)Rin; // update value
					adc1Rgnd = (float)Rgnd; // update value
					adc1ScaleDone = false;
				}
				else // ADC2
				{
					adc2Rin = (float)Rin; // update value
					adc2Rgnd = (float)Rgnd; // update value
					adc2ScaleDone = false;
				}
				sprintf(txBuffer,"\r\n  ADC%d resistors updated:  Rin=%d  Rgnd=%d\r\n> ",adcNum, Rin, Rgnd);
				txBufLen = strlen(txBuffer);
			}
		}


		else if (!(strcmp(Rx, cnfgChAllMsg))) // "show-config channel all" command
		{
			char addString[12];
			for (int i = 0 ; i < 5 ; i++) // add to buffer
			{
				sprintf(addString," Ch%d: %c%d\r\n",i, ports[channels[i]].portLetter, ports[channels[i]].portNumber);
				strcat(displayChannels1,addString);
				sprintf(addString," Ch%d: %c%d\r\n",i+5, ports[channels[i+5]].portLetter, ports[channels[i+5]].portNumber);
				strcat(displayChannels2,addString);
				sprintf(addString," Ch%d: %c%d\r\n",i+10, ports[channels[i+10]].portLetter, ports[channels[i+10]].portNumber);
				strcat(displayChannels3,addString);
				sprintf(addString," Ch%d: %c%d\r\n",i+15, ports[channels[i+15]].portLetter, ports[channels[i+15]].portNumber);
				strcat(displayChannels4,addString);
				sprintf(addString," Ch%d: %c%d\r\n",i+20, ports[channels[i+20]].portLetter, ports[channels[i+20]].portNumber);
				strcat(displayChannels5,addString);
				sprintf(addString," Ch%d: %c%d\r\n",i+25, ports[channels[i+25]].portLetter, ports[channels[i+25]].portNumber);
				strcat(displayChannels6,addString);
				if (i+30 < CHANNELS)
				{
					sprintf(addString," Ch%d: %c%d\r\n",i+30, ports[channels[i+30]].portLetter, ports[channels[i+30]].portNumber);
					strcat(displayChannels7,addString);
				}
			}
			strcat(displayChannels7,NewLineCli2); // new line after list
			longMsgCh = true;
		}


		else if ((Rx[0] == 's') && (Rx[1] == 'h') && (Rx[2] == 'o') && (Rx[3] == 'w') && (Rx[4] == '-') && (Rx[5] == 'c') && (Rx[6] == 'o') && (Rx[7] == 'n') && (Rx[8] == 'f') && (Rx[9] == 'i') && (Rx[10] == 'g') && (Rx[11] == ' ') && (Rx[12] == 'c') && (Rx[13] == 'h')) // "show config chX\r" / "show config chXX\r"
		{
			int chNum = 0;
			if ((1 != sscanf(Rx, "%*[^0123456789]%d",&chNum))
				|| (!((Rx[15] == '\r') || (Rx[16] == '\r') || (Rx[17] == '\r'))) || (chNum < 0 ) || (chNum > CHANNELS-1)) // error
				ERRORTYPE = ErrorShowChConfig;
			else // no error
			{
				sprintf(txBuffer,"\r\n  Channel %d is port: P%c%d\r\n> ",chNum, ports[channels[chNum]].portLetter, ports[channels[chNum]].portNumber);
				txBufLen = strlen(txBuffer);
			}
		}


		else if ((Rx[0] == 'c') && (Rx[1] == 'o') && (Rx[2] == 'n') && (Rx[3] == 'f') && (Rx[4] == 'i') && (Rx[5] == 'g') && (Rx[6] == ' ') && (Rx[7] == 'c') && (Rx[8] == 'h')) // "config chX PXN\r" / "config chXX PXNN\r"
		{
			int chNum = 0; // channel number
			int portNum = 0; // port number
			char portLtr; // port letter

			if ((2 != sscanf(Rx, "%*[^0123456789]%d%*[^0123456789]%d",&chNum, &portNum))
					|| (chNum < 0 ) || (chNum > CHANNELS-1) || (portNum < 0 ) || (portNum > 99) || (!((((Rx[11] == 'p') && (isalpha(Rx[12])))) || ((Rx[12] == 'p') && (isalpha(Rx[13])))))) // error
				ERRORTYPE = ErrorChConfig;
			else // no error
			{
				if (Rx[11] == 'p')
					portLtr = toupper(Rx[12]); // port letter
				else
					portLtr = toupper(Rx[13]); // port letter

				for (int i = 0 ; i < CHANNELS ; i++) // find desired port
				{
					if ((ports[i].portLetter == portLtr) && (ports[i].portNumber == portNum)) // find desired port
					{
						channels[chNum] = i; // index to new port - ch2 pd5 - config ch2 pd1 - chNum portLtr portNum
						updateChannels();
						sprintf(txBuffer,"\r\n  Channel %d updated to port: P%c%d\r\n> ",chNum, portLtr, portNum);
					}
				}
				txBufLen = strlen(txBuffer);
				if (txBufLen < 5) // didn't find port
				{
					sprintf(txBuffer,"\r\n  The port P%c%d is not a channel\r\n> ",portLtr, portNum);
					txBufLen = strlen(txBuffer);
				}
			}
		}


		else if (!(strcmp(Rx, cnfgPortAllMsg))) // "show-config port all" command
		{
			for (int i = 0 ; i < 3 ; ++i) // add to buffer
			{
				displayPortConfig(ports[i].portMode, ports[i].portLetter, ports[i].portNumber, displayPorts1);
				displayPortConfig(ports[i+3].portMode, ports[i+3].portLetter, ports[i+3].portNumber, displayPorts2);
				displayPortConfig(ports[i+6].portMode, ports[i+6].portLetter, ports[i+6].portNumber, displayPorts3);
				displayPortConfig(ports[i+9].portMode, ports[i+9].portLetter, ports[i+9].portNumber, displayPorts4);
				displayPortConfig(ports[i+12].portMode, ports[i+12].portLetter, ports[i+12].portNumber, displayPorts5);
				displayPortConfig(ports[i+15].portMode, ports[i+15].portLetter, ports[i+15].portNumber, displayPorts6);
				displayPortConfig(ports[i+18].portMode, ports[i+18].portLetter, ports[i+18].portNumber, displayPorts7);
				displayPortConfig(ports[i+21].portMode, ports[i+21].portLetter, ports[i+21].portNumber, displayPorts8);
				displayPortConfig(ports[i+24].portMode, ports[i+24].portLetter, ports[i+24].portNumber, displayPorts9);
				displayPortConfig(ports[i+27].portMode, ports[i+27].portLetter, ports[i+27].portNumber, displayPorts10);

				if (i+30 < CHANNELS)
					displayPortConfig(ports[i+30].portMode, ports[i+30].portLetter, ports[i+30].portNumber, displayPorts11);
			}
			strcat(displayPorts11, NewLineCli2); // new line after list
			longMsgPort = true;
		}


		else if ((Rx[0] == 's') && (Rx[1] == 'h') && (Rx[2] == 'o') && (Rx[3] == 'w') && (Rx[4] == '-') && (Rx[5] == 'c') && (Rx[6] == 'o') && (Rx[7] == 'n') && (Rx[8] == 'f') && (Rx[9] == 'i') && (Rx[10] == 'g') && (Rx[11] == ' ') && (Rx[12] == 'p') && (Rx[13] == 'o') && (Rx[14] == 'r') && (Rx[15] == 't')) // "show config port PXNN\r"
		{
			int portNum = 0; // port number
			char portLtr = toupper(Rx[18]); // port letter

			if ((1 != sscanf(Rx, "%*[^0123456789]%d",&portNum)) || (!(isalpha(portLtr))) || (!((Rx[20] == '\r') || (Rx[21] == '\r'))) || (portNum < 0 ) || (portNum > 99) || (Rx[17] != 'p')) // error
				ERRORTYPE = ErrorShowPortConfig;
			else // no error
			{
				for (int i = 0 ; i < CHANNELS ; i++) // find desired port
				{
					if ((ports[i].portLetter == portLtr) && (ports[i].portNumber == portNum)) // find desired port
					{
						if (ports[i].portMode == GPIO_MODE_OUTPUT_PP) // output mode
							sprintf(txBuffer,"\r\n  Port P%c%d is in mode: OUTPUT\r\n> ",portLtr, portNum);
						else if (ports[i].portMode == GPIO_MODE_INPUT) // input mode
							sprintf(txBuffer,"\r\n  Port P%c%d is in mode: INPUT\r\n> ",portLtr, portNum);
						else // unknown mode
							sprintf(txBuffer,"\r\n  Port P%c%d is in mode: UNKNOWN\r\n> ",portLtr, portNum);
					}
				}
				txBufLen = strlen(txBuffer);
				if (txBufLen < 5) // didn't find port
				{
					sprintf(txBuffer,"\r\n  The port P%c%d is not a channel\r\n> ",portLtr, portNum);
					txBufLen = strlen(txBuffer);
				}
			}
		}


		else if ((Rx[0] == 'c') && (Rx[1] == 'o') && (Rx[2] == 'n') && (Rx[3] == 'f') && (Rx[4] == 'i') && (Rx[5] == 'g') && (Rx[6] == ' ') && (Rx[7] == 'p') && (Rx[8] == 'o') && (Rx[9] == 'r') && (Rx[10] == 't') && (Rx[11] == ' ')) // "config port PXNN i\r"
		{
			int portNum = 0; // port number
			char portLtr = toupper(Rx[13]); // port letter

			if ((1 != sscanf(Rx, "%*[^0123456789]%d",&portNum)) || (!(isalpha(portLtr))) || (!((Rx[17] == '\r') || (Rx[18] == '\r'))) || (portNum < 0 ) || (portNum > 99) || (Rx[12] != 'p')) // error
				ERRORTYPE = ErrorPortConfig;
			else // no error
			{
				for (int i = 0 ; i < CHANNELS ; i++) // find desired port
				{
					if ((ports[i].portLetter == portLtr) && (ports[i].portNumber == portNum)) // find desired port
					{
						if ((Rx[16] == 'i') || (Rx[17] == 'i')) // set as input
						{
							ports[i].portMode = GPIO_MODE_INPUT;
							gpioInit.Pin = ports[i].chPin;
							gpioInit.Mode = GPIO_MODE_INPUT;
							gpioInit.Pull = GPIO_PULLDOWN;
							gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
							HAL_GPIO_Init(ports[i].portRegs, &gpioInit);
							sprintf(txBuffer,"\r\n  Port P%c%d set to: INPUT\r\n> ",portLtr, portNum);
						}
						else if ((Rx[16] == 'o') || (Rx[17] == 'o')) // set as output
						{
							ports[i].portMode = GPIO_MODE_OUTPUT_PP;
							gpioInit.Pin = ports[i].chPin;
							gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
							gpioInit.Pull = GPIO_PULLDOWN;
							gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
							HAL_GPIO_Init(ports[i].portRegs, &gpioInit);
							sprintf(txBuffer,"\r\n  Port P%c%d set to: OUTPUT\r\n> ",portLtr, portNum);
						}
						else
							ERRORTYPE = ErrorPortConfig; // error
					}
				}
				txBufLen = strlen(txBuffer);
				if ((txBufLen < 5) && (ERRORTYPE == NoError))// didn't find port
				{
					sprintf(txBuffer,"\r\n  The port P%c%d is not a channel\r\n> ",portLtr, portNum);
					txBufLen = strlen(txBuffer);
				}
			}
		}


		else if (!(strcmp(Rx, cnfgDefaultMsg))) // "config default" command
		{
			resetChannels();
			sprintf(txBuffer,"\r\n  All default settings were set\r\n> ");
			txBufLen = strlen(txBuffer);
		}


		else if ((Rx[0] == 'e') && (Rx[1] == 'c') && (Rx[2] == 'h') && (Rx[3] == 'o')) // command "echo on\r" / "echo off\r"
		{
			if ((Rx[4] != ' ') || (!((Rx[7] == '\r') || (Rx[8] == '\r')))) // error
				ERRORTYPE = ErrorEcho;
			else // no error
			{
				if ((Rx[5] == 'o') && (Rx[6] == 'n') && (Rx[7] == '\r')) // echo on
				{
					echoEnable = true;
					sprintf(txBuffer,"\r\n  Echo set: ON\r\n> ");
					txBufLen = strlen(txBuffer);
				}
				else if ((Rx[5] == 'o') && (Rx[6] == 'f') && (Rx[7] == 'f')) // echo off
				{
					echoEnable = false;
					sprintf(txBuffer,"\r\n  Echo set: OFF\r\n> ");
					txBufLen = strlen(txBuffer);
				}
				else // error
					ERRORTYPE = ErrorEcho;
			}
		}


		else if ((Rx[0] == 's') && (Rx[1] == 'a') && (Rx[2] == 'v') && (Rx[3] == 'e')) // command "save\r"
		{
			if (Rx[4] != '\r') // error
				ERRORTYPE = ErrorSave;
			else // no error
			{
				static uint32_t txFlash[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array to store in flash
				txFlash[0] = (uint32_t) adc1Rin;
				txFlash[1] = (uint32_t) adc1Rgnd;
				txFlash[2] = (uint32_t) adc2Rin;
				txFlash[3] = (uint32_t) adc2Rgnd;
				txFlash[4] = (uint32_t) (adcDropWarning * 10); // *10 to store numbers after point
				txFlash[5] = (uint32_t) (adcDropError * 10); // *10 to store numbers after point
				txFlash[6] = chStates;

				if (adc1ScaleDone) // save adc1 scaling
				{
					txFlash[7] = (uint32_t) (adcScaleSlope[0] * 1000 + 1000); // to store negative + store after point
					txFlash[8] = (uint32_t) (adcScaleAxis[0] * 1000 + 1000); // to store negative + store after point
				}
				else // dont save
				{
					txFlash[7] = 0xFFFFFFFF; // default flash value
					txFlash[8] = 0xFFFFFFFF; // default flash value
				}
				if (adc2ScaleDone) // save adc2 scaling
				{
					txFlash[9] = (uint32_t) (adcScaleSlope[1] * 1000 + 1000); // to store negative + store after point
					txFlash[10] = (uint32_t) (adcScaleAxis[1] * 1000 + 1000); // to store negative + store after point
				}
				else // dont save
				{
					txFlash[9] = 0xFFFFFFFF; // default flash value
					txFlash[10] = 0xFFFFFFFF; // default flash value
				}

				MY_FLASH_WriteN(0, txFlash, 11, DATA_TYPE_32); // save to flash
				sprintf(txBuffer,"\r\n  ADC and channel configuration were saved in flash.\r\n> ");
				txBufLen = strlen(txBuffer);
			}
		}


		else if ((Rx[0] == 's') && (Rx[1] == 'p') && (Rx[2] == 'i') && (Rx[3] == '-') && (Rx[4] == 's') && (Rx[5] == 'e') && (Rx[6] == 'n') && (Rx[7] == 'd')  && (Rx[8] == ' ')) // command "spi-send 0xN 0xN 0xN\r"
		{
			char spiParseData[3][2] = {{0, 0}, {0, 0}, {0, 0}};
			uint8_t spiData[3] = {0, 0, 0};
			int argNum = 0; // counting arguments
			bool storeData = false; // store parsed argument
			int j = 0; // stored arguments index

			for (int i = 9 ; ((i < sizeof(Rx) - 2) && (argNum < 3) && (ERRORTYPE == NoError)) ; ++i) // parse spi data
			{
				if (!(((Rx[i] >= '0') && (Rx[i] <= '9')) || ((Rx[i] >= 'a') && (Rx[i] <= 'f')) || (Rx[i] == 'x') || (Rx[i] == ' '))) // error
					ERRORTYPE = ErrorSpi;
				else // no error
				{
					if (Rx[i] == ' ') // next argument
					{
						storeData = false;
						argNum++;
						j = 0;
					}
					if (storeData) // storing spi data
					{
						if (j > 1) // error, argument too long
							ERRORTYPE = ErrorSpi;
						else
						{
							spiParseData[argNum][j] = Rx[i];
							j++;
						}
					}
					if (Rx[i] == 'x') // store argument
						storeData = true;
				}
			}
			argNum++; // add last argument

			if (argNum == 1)
			{
				charHexToInt(spiParseData[0], &spiData[0]); // convert to uint8_t
				sendSPI(spiData, 1); // spi send
			}
			else if (argNum == 2)
			{
				charHexToInt(spiParseData[0], &spiData[0]); // convert to uint8_t
				charHexToInt(spiParseData[1], &spiData[1]); // convert to uint8_t
				sendSPI(spiData, 2); // spi send
			}
			else if (argNum == 3)
			{
				charHexToInt(spiParseData[0], &spiData[0]); // convert to uint8_t
				charHexToInt(spiParseData[1], &spiData[1]); // convert to uint8_t
				charHexToInt(spiParseData[2], &spiData[2]); // convert to uint8_t
				sendSPI(spiData, 3); // spi send
			}
			else // error
				ERRORTYPE = ErrorSpi;

			if (ERRORTYPE == NoError) // no error
			{
				sprintf(txBuffer,"\r\n  %d SPI packages sent.\r\n> ",argNum);
				txBufLen = strlen(txBuffer);
			}
		}


		else // all other input
		{
			ERRORTYPE = ErrorCommand;
		}
		procFlag = false; // processing done
		txFlag = true; // ready to send
	}
}



// send data
void transmitData(void)
{
	if (txFlag) // ready to send
	{
		txUsbState = CDC_Transmit_FS((uint8_t*)txBuffer, txBufLen); // send
		txFlag = false; // transmit done

		if (txUsbState == USBD_BUSY) // busy
			txFlag = true; // still busy - try again next loop

		else if (txUsbState == USBD_FAIL) // sent failed
			ERRORTYPE = ErrorUSB;

		else // successfully sent
		{
			for (int i=0 ; i < txBufLen ; i++) // clear tx buffer
				txBuffer[i] = '\0';
			txBufLen = 0;
		}

		if (longMsgHelp) // still sending
		{
			for (int i = 0 ; i < numbHelpLines ; ++i)
				txUsbMessage2(helpList[i].helpMessage);

			for (int i=0 ; i < txBufLen ; i++) // clear tx buffer
				txBuffer[i] = '\0';
			txBufLen = 0;
			longMsgHelp = false;
			txFlag = false;
		}

		if (longMsgCh) // still sending
		{
			txUsbMessage(displayChannels1); // channel configuration 1
			txUsbMessage(displayChannels2); // channel configuration 2
			txUsbMessage(displayChannels3); // channel configuration 3
			txUsbMessage(displayChannels4); // channel configuration 4
			txUsbMessage(displayChannels5); // channel configuration 5
			txUsbMessage(displayChannels6); // channel configuration 6
			txUsbMessage(displayChannels7); // channel configuration 7

			for (int i=0 ; i < txBufLen ; i++) // clear tx buffer
				txBuffer[i] = '\0';
			txBufLen = 0;
			longMsgCh = false;
			cleanChannelDisplayBuffers();
			txFlag = false;
		}

		if (longMsgPort) // still sending
		{
			txUsbMessage(displayPorts1); // port configuration 1
			txUsbMessage(displayPorts2); // port configuration 2
			txUsbMessage(displayPorts3); // port configuration 3
			txUsbMessage(displayPorts4); // port configuration 4
			txUsbMessage(displayPorts5); // port configuration 5
			txUsbMessage(displayPorts6); // port configuration 6
			txUsbMessage(displayPorts7); // port configuration 7
			txUsbMessage(displayPorts8); // port configuration 8
			txUsbMessage(displayPorts9); // port configuration 9
			txUsbMessage(displayPorts10); // port configuration 10
			txUsbMessage(displayPorts11); // port configuration 11

			for (int i=0 ; i < txBufLen ; i++) // clear tx buffer
				txBuffer[i] = '\0';
			txBufLen = 0;
			longMsgPort = false;
			cleanChannelDisplayBuffers();
			txFlag = false;
		}
	}
}



// analyze errors
void checkErrors(void) // display errors
{
	if (ERRORTYPE) // error happened
	{
		HAL_GPIO_WritePin(errorLedPort, errorLedPin, 1); // error led
		static uint8_t const Error0cli[] = "  ERROR! an unknown error has occurred.\r\n> ";
		static uint8_t const Error1cli[] = "\r\n  ERROR! size of receive package is larger than 64 bytes.\r\n> ";
		static uint8_t const Error2cli[] = "  ERROR! Invalid command.\r\n> ";
		static uint8_t const Error3cli[] = "  ERROR! USB error has occurred.\r\n> ";
		static uint8_t const Error4cli[] = "  ERROR! Set channel: \"ch<N> <0/1>\" (ch13 1).\r\n> ";
		static uint8_t const Error5cli[] = "  ERROR! Status commands: \"status <N>\" / status all.\r\n> ";
		static uint8_t const Error6cli[] = "  ERROR! Set all 32 channels: \"ch 0x<N>\" (ch 0x5aF).\r\n> ";
		static uint8_t const Error7cli[] = "  ERROR! Regulate power via PE14: \"pwr on\" / \"pwr off\".\r\n> ";
		static uint8_t const Error8cli[] = "  ERROR! ADC conversion error.\r\n> ";
		static uint8_t const Error9cli[] = "  ERROR! Sleep command: \"sleep\" <N> (sleep 7).\r\n> ";
		static uint8_t const Error10cli[] = "  ERROR! ADC resistors: adc<N> <Rin> <Rgnd> (adc2 45 170).\r\n> ";
		static uint8_t const Error11cli[] = "  ERROR! Show channel configuration: \"show config ch<N>\".\r\n> ";
		static uint8_t const Error12cli[] = "  ERROR! Show port configuration: \"show config port P<N>\".\r\n> ";
		static uint8_t const Error13cli[] = "  ERROR! Echo command: \"echo on\" / \"echo off\".\r\n> ";
		static uint8_t const Error14cli[] = "  ERROR! \"config port P<N> <i/o>\" (config port PD15 i).\r\n> ";
		static uint8_t const Error15cli[] = "  ERROR! \"config ch<N> P<N>\" (config ch17 pd0).\r\n> ";
		static uint8_t const Error16cli[] = "  ERROR! Allow pwr-OFF: \"adc-err on\" or \"adc-err off\".\r\n> ";
		static uint8_t const Error17cli[] = "  ERROR! Config \"adc-lvl <0/1> <w> <e>\" (adc-lvl 1 13 12).\r\n> ";
		static uint8_t const Error18cli[] = "  ERROR! Flash save command is \"save\".\r\n> ";
		static uint8_t const Error19cli[] = "  ERROR! ADC scale: \"adc#-scl <x1> <y1> <x2> <y2>\".\r\n> ";
		static uint8_t const Error20cli[] = "  ERROR! SPI: \"spi-send 0x<N>\" (spi-send 0xa 0x7A 0x7).\r\n> ";

		switch (ERRORTYPE)
		{
		case ErrorSize:
			copyErrorMsg(txBuffer, &txBufLen, Error1cli, (int)sizeof(Error1cli)-1); // add error to tx buffer
			break;

		case ErrorCommand:
			copyErrorMsg(txBuffer, &txBufLen, Error2cli, (int)sizeof(Error2cli)-1); // add error to tx buffer
			break;

		case ErrorUSB:
			copyErrorMsg(txBuffer, &txBufLen, Error3cli, (int)sizeof(Error3cli)-1); // add error to tx buffer
			break;

		case ErrorChSyntax:
			copyErrorMsg(txBuffer, &txBufLen, Error4cli, (int)sizeof(Error4cli)-1); // add error to tx buffer
			break;

		case ErrorStatusSyntax:
			copyErrorMsg(txBuffer, &txBufLen, Error5cli, (int)sizeof(Error5cli)-1); // add error to tx buffer
			break;

		case ErrorChHexSyntax:
			copyErrorMsg(txBuffer, &txBufLen, Error6cli, (int)sizeof(Error6cli)-1); // add error to tx buffer
			break;

		case ErrorPower:
			copyErrorMsg(txBuffer, &txBufLen, Error7cli, (int)sizeof(Error7cli)-1); // add error to tx buffer
			break;

		case ErrorADC:
			copyErrorMsg(txBuffer, &txBufLen, Error8cli, (int)sizeof(Error8cli)-1); // add error to tx buffer
			break;

		case ErrorSleep:
			copyErrorMsg(txBuffer, &txBufLen, Error9cli, (int)sizeof(Error9cli)-1); // add error to tx buffer
			break;

		case ErrorResistors:
			copyErrorMsg(txBuffer, &txBufLen, Error10cli, (int)sizeof(Error10cli)-1); // add error to tx buffer
			break;

		case ErrorShowChConfig:
			copyErrorMsg(txBuffer, &txBufLen, Error11cli, (int)sizeof(Error11cli)-1); // add error to tx buffer
			break;

		case ErrorShowPortConfig:
			copyErrorMsg(txBuffer, &txBufLen, Error12cli, (int)sizeof(Error12cli)-1); // add error to tx buffer
			break;

		case ErrorEcho:
			copyErrorMsg(txBuffer, &txBufLen, Error13cli, (int)sizeof(Error13cli)-1); // add error to tx buffer
			break;

		case ErrorPortConfig:
			copyErrorMsg(txBuffer, &txBufLen, Error14cli, (int)sizeof(Error14cli)-1); // add error to tx buffer
			break;

		case ErrorChConfig:
			copyErrorMsg(txBuffer, &txBufLen, Error15cli, (int)sizeof(Error15cli)-1); // add error to tx buffer
			break;

		case ErrorAdcEnable:
			copyErrorMsg(txBuffer, &txBufLen, Error16cli, (int)sizeof(Error16cli)-1); // add error to tx buffer
			break;

		case ErrorADCconfig:
			copyErrorMsg(txBuffer, &txBufLen, Error17cli, (int)sizeof(Error17cli)-1); // add error to tx buffer
			break;

		case ErrorSave:
			copyErrorMsg(txBuffer, &txBufLen, Error18cli, (int)sizeof(Error18cli)-1); // add error to tx buffer
			break;

		case ErrorADCcalibration:
			copyErrorMsg(txBuffer, &txBufLen, Error19cli, (int)sizeof(Error19cli)-1); // add error to tx buffer
			break;

		case ErrorSpi:
			copyErrorMsg(txBuffer, &txBufLen, Error20cli, (int)sizeof(Error20cli)-1); // add error to tx buffer
			break;

		case ErrorUnknown:
			copyErrorMsg(txBuffer, &txBufLen, Error0cli, (int)sizeof(Error0cli)-1); // add error to tx buffer
			break;

		default:
			copyErrorMsg(txBuffer, &txBufLen, Error0cli, (int)sizeof(Error0cli-1)); // add error to tx buffer
			break;
		}
		ERRORTYPE = NoError; // handled error
		txFlag = true; // send the error
	}
}



// copy error message to tx buffer
static void copyErrorMsg(uint8_t *src1, int *len1, uint8_t const *src2, int len2)
{
	memcpy(src1 + *len1, src2, len2);
	(*len1) += len2;
}



// phrase hex and update channel states buffer
static uint32_t hexToBin(char *hex)
{
	uint32_t result = 0;
	for (int i = 8 ; i ; ++hex, --i)
	{
		char d = *hex;
		result <<= 4;

		if ((d >= '0') && (d <= '9'))
		    result |= d - '0';
		else if ((d >= 'a') && (d <= 'f'))
			result |= d - 'a' + 10;
	}
	return result;
}



// check sleep timer
void sleepCheck(void)
{
	if (powerOutput) // if power output on
		sleepCount++;
	else // power off
		sleepCount = 0; // reset timer

	if (sleepCount == sleepTime) // sleep
	{
		HAL_GPIO_WritePin(powerOutPort, powerOutPin, 0); // power off PE14 ---------------
		powerOutput = false; // power OFF PE14
		powerOutEnableSet(false); // power ON PE14 flag in main

		if (echoEnable)
		{
			sprintf(txBuffer, "%.0f minutes elapsed, Going to sleep: PE14 OFF\r\n> ", sleepCount/60.0);
			txBufLen = strlen(txBuffer);
			txFlag = true;
		}
		sleepCount = 0; // reset timer
	}

	if ((powerOutput) && (adcResultDisplay) && (sleepCount % 5 == 0)) // ADC results display every 5 seconds
	{
		sprintf(txBuffer, "\r\n  ADC1: %.2fv\r\n  ADC2: %.2fv\r\n> ",adc1Result, adc2Result);
		txBufLen = strlen(txBuffer);
		txFlag = true;
	}
}



// clean display channel configuration buffers
static void cleanChannelDisplayBuffers(void)
{
	for (int i=0 ; i < 64 ; ++i) // initialize channel configuration display
	{
		displayChannels1[i] = '\0';
		displayChannels2[i] = '\0';
		displayChannels3[i] = '\0';
		displayChannels4[i] = '\0';
		displayChannels5[i] = '\0';
		displayChannels6[i] = '\0';
		displayChannels7[i] = '\0';
		displayPorts1[i] = '\0';
		displayPorts2[i] = '\0';
		displayPorts3[i] = '\0';
		displayPorts4[i] = '\0';
		displayPorts5[i] = '\0';
		displayPorts6[i] = '\0';
		displayPorts7[i] = '\0';
		displayPorts8[i] = '\0';
		displayPorts9[i] = '\0';
		displayPorts10[i] = '\0';
		displayPorts11[i] = '\0';
	}
}



// add port configuration to display buffer
static void displayPortConfig(uint32_t portMode, char portLetter, int portNumber, char *displayPorts)
{
	char addString[30];

	if (portMode == GPIO_MODE_OUTPUT_PP) // output
		strcpy(portCnfg, "OUTPUT\r\n");
	else if (portMode == GPIO_MODE_INPUT) // input
		strcpy(portCnfg, "INPUT\r\n");
	else
		strcpy(portCnfg, "UNKNOWN\r\n");

	sprintf(addString," Port P%c%d: ",portLetter, portNumber);
	strcat(addString, portCnfg);
	strcat(displayPorts, addString);
}



// send a list to usb
static void txUsbMessage(char *txUsbMsg)
{
	txUsbState = USBD_BUSY;
	while (txUsbState == USBD_BUSY) // while usb is busy
		txUsbState = CDC_Transmit_FS((uint8_t*)txUsbMsg, strlen(txUsbMsg)); // send
}



// send help list to usb
static void txUsbMessage2(uint8_t *txUsbMsg)
{
	txUsbState = USBD_BUSY;
	while (txUsbState == USBD_BUSY) // while usb is busy
		txUsbState = CDC_Transmit_FS((uint8_t*)txUsbMsg, strlen(txUsbMsg)); // send
}



// set default channel settings
static void resetChannels(void)
{
	for (int i = 0 ; i < CHANNELS ; ++i)
	{
		if ((i >= 0) && (i <= 14)) // port D   0, &Dport, GPIOD, GPIO_PIN_0 , 'D',  0, GPIO_MODE_OUTPUT_PP
		{
			ports[i].chPort = &Dport; // update the selected channel
			ports[i].portRegs = GPIOD; // update the selected channel
			ports[i].chPin = gpioPins[i]; // update the selected channel
			ports[i].portLetter = 'D'; // update the selected channel
			ports[i].portNumber = i; // update the selected channel
		}
		else if ((i >= 15) && (i <= 30)) // port B
		{
			ports[i].chPort = &Bport; // update the selected channel
			ports[i].portRegs = GPIOB; // update the selected channel
			ports[i].chPin = gpioPins[i-15]; // update the selected channel
			ports[i].portLetter = 'B'; // update the selected channel
			ports[i].portNumber = i-15; // update the selected channel
		}
		else // port E
		{
			ports[i].chPort = &Eport; // update the selected channel
			ports[i].portRegs = GPIOE; // update the selected channel
			ports[i].chPin = gpioPins[15]; // update the selected channel
			ports[i].portLetter = 'E'; // update the selected channel
			ports[i].portNumber = 15; // update the selected channel
		}
		channels[i] = i; // reset channel-port mapping
		ports[i].portMode = GPIO_MODE_OUTPUT_PP; // update the selected channel
		gpioInit.Pin = ports[i].chPin;
		gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
		gpioInit.Pull = GPIO_PULLDOWN;
		gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(ports[i].portRegs, &gpioInit);
	}

	HAL_GPIO_WritePin(powerOutPort, powerOutPin, 0); // initialize power output
	chStates = 0; // initial channel state
	updateChannels(); // set initial channel state
	cleanChannelDisplayBuffers(); // clean display channel configuration buffers
}



// returns the state of PE14 - pwr on
_Bool powerOutputEnableCheck(void)
{
	return powerOutput; // enable monitoring when power is enabled
}



// sets the state of PE14 - pwr on
void powerIsOnSet(_Bool set)
{
	powerInput = set; // set PE14
	powerOnChange = true;
}



// set data from flash at power up
void loadFromFlash (void)
{
	static uint32_t rxFlash[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	MY_FLASH_ReadN(0, rxFlash, 11, DATA_TYPE_32); // get flash data

	if (rxFlash[0] != 0xFFFFFFFF) // not first new processor power up
	{
		adc1Rin = rxFlash[0];
		adc1Rgnd = rxFlash[1];
		adc2Rin = rxFlash[2];
		adc2Rgnd = rxFlash[3];
		adcDropWarning = rxFlash[4];
		adcDropError = rxFlash[5];
		chStates = rxFlash[6];

		adcDropWarning /= 10; // /10 to restore numbers after point
		adcDropError /= 10; // /10 to restore numbers after point

		if (rxFlash[7] != 0xFFFFFFFF) // load ADC1 scaling
		{
			adcScaleSlope[0] = rxFlash[7];
			adcScaleSlope[0] = (adcScaleSlope[0] - 1000) / 1000; // to restore negative + store after point
			adcScaleAxis[0] = rxFlash[8];
			adcScaleAxis[0] = (adcScaleAxis[0] - 1000) / 1000; // to restore negative + store after point
			adc1ScaleDone = true;
		}

		if (rxFlash[9] != 0xFFFFFFFF) // load ADC2 scaling
		{
			adcScaleSlope[1] = rxFlash[9];
			adcScaleSlope[1] = (adcScaleSlope[1] - 1000) / 1000; // to restore negative + store after point
			adcScaleAxis[1] = rxFlash[10];
			adcScaleAxis[1] = (adcScaleAxis[1] - 1000) / 1000; // to restore negative + store after point
			adc2ScaleDone = true;
		}

		sprintf(txBuffer, "Configuration was loaded from flash.\r\n> ");
		txBufLen = strlen(txBuffer);
		txFlag = true;
	}
}



// linear approximation: find slope and axis point
static void linearApprox (float *slopeResult, float *axisPointResult, float x1, float y1, float x2, float y2)
{
	*slopeResult = (y1 - y2) / (x1 - x2);
	*axisPointResult = y1 - (*slopeResult) * x1;
}



// convert char hex to int
static void charHexToInt (char *hexArray, uint8_t *uintArray)
{
	for (int i = 0 ; i < 2 ; ++i)
	{
		if (*hexArray == 0) // 1 digit argument
		{
			if (i == 1)
				*uintArray >>= 4;
		}

		else if ((*hexArray >= '0') && (*hexArray <= '9')) // number
			if (i == 0) // upper digit
				*uintArray += ((*hexArray - '0') << 4);
			else // lower digit
				*uintArray += (*hexArray - '0');

		else // letter
			if (i == 0) // upper digit
				*uintArray += ((*hexArray - 'a' + 10) << 4);
			else // lower digit
				*uintArray += (*hexArray - 'a' + 10);

		hexArray++;
	}
}
