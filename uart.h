// uart.h
// Description: Module .h file for UART initializing, and transmitting and receiving
// Author: Tristan Huen
// Date Oct. 29, 2021

#pragma once

#include <stdint.h>
#include "tm4c123gh6pm.h"

#define NO_DATA -1 //If there is no data to read

//uartInit
//Description: Initializes UART0 to transmit at 14400 baud N81
//Arguments: none
//Return value: none
void uartInit(void);

//uartTX
//Description: Transmits data to FIFO register
//Arguments: data - Data to be transmitted
//Return value: none
void uartTX(uint8_t data);

//uartRX
//Description: Gets received byte from FIFO otherwise NO_DATA is returned
//Arguments: none
//Return value: Data from FIFO or NO_DATA = -1
int16_t uartRX();

