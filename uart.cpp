// uart.c
// Description: Module .c file for UART initializing, and transmitting and receiving
// Author: Tristan Huen
// Date Oct. 29, 2021

#include <uart.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"

//Set baud rate to 14400 baud with 16MHz system clock
//#define IBRD_VAL 69
//#define FBRD_VAL 28

//Set baud rate to 115200 baud with 16MHz system clock
#define IBRD_VAL 8
#define FBRD_VAL 44


//uartInit
//Description: Initializes UART0 to transmit at 14400 baud N81
//Arguments: none
//Return value: none
void uartInit(void) {
    //Setup UART0 to transmit at 14400 baud N81
    UART0_CTL_R &= ~UART_CTL_UARTEN; //Disable UART
    UART0_IBRD_R = IBRD_VAL;
    UART0_FBRD_R = FBRD_VAL;
    UART0_LCRH_R = UART_LCRH_WLEN_8; //8-bit word
    UART0_CTL_R |= UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE; //Enable UART

}

//uartTX
//Description: Transmits data to FIFO register
//Arguments: data - Data to be transmitted
//Return value: none
void uartTX(uint8_t data) {
    while (UART0_FR_R & UART_FR_TXFF) {} //Wait for space in transmit FIFO
    UART0_DR_R = data; //Put byte to transmit in FIFO

}

//uartRX
//Description: Gets received byte from FIFO otherwise NO_DATA is returned
//Arguments: none
//Return value: Data from FIFO or NO_DATA = -1
int16_t uartRX() {
    int16_t data;
    /*
    if(!(UART0_FR_R & UART_FR_RXFE)) { //If receive byte is sent
        data = UART0_DR_R; //Get received byte from FIFO
        return data;
    }

    else {
        return NO_DATA; //Return -1 if no receive byte sent
    }
    */

    //Need to convert this to an interrupt
    while(UART0_FR_R & UART_FR_RXFE){}
    data = UART0_DR_R;

    return data;
}


