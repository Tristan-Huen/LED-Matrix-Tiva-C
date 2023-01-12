#include "tm4c123gh6pm.h"
#include "uart.h"
#include <utility>
#include <cstdint>
#include <string>
#include <array>
#include <stdio.h>

/* TO DO
 -Consider converting uart.cpp to a class.
 -Make uart be interrupt based.
 -Do basic led matrix setup (RGB and CMY for now).
 -Implement some rgb function stuff.

 */

//Use UART0 RXD and TXD connected to Port A pins 0 and 1 respectively
#define U0RX BIT0
#define U0TX BIT1

#define COLOR_ARRAY_SIZE 7

std::array<uint16_t, 7> color_vals;
int counter = 0;

void reverse(char str[], int length);
char* itoa(int num, char* str, int base);
void printstring(char* str);

int main() {

    char num_str[5];
    int16_t first_bit;
    int16_t second_bit;
    int16_t third_bit;
    int16_t fourth_bit;
    uint16_t num;

    //Activate clock for GPIO Port A, B, C, D, and F.
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;

    //Activate clock for UART Module 0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;

    //Enable RXD/TXD pins for UART0 on Port A pins 0 and 1
    GPIO_PORTA_AFSEL_R |= U0RX | U0TX;
    GPIO_PORTA_DEN_R |= U0RX | U0TX;
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~(GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M)) |
    GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX; // UART mode

    uartInit();

    while(1){

        //Expect 4 chars to be sent in. NOTE: All these statements are blocking.
        first_bit = uartRX();
        second_bit = uartRX();
        third_bit = uartRX();
        fourth_bit = uartRX();

        //If array data comes in.
        if(counter == COLOR_ARRAY_SIZE) {
            counter = 0;
        }

        std::string str = std::string() + (char)first_bit + (char)second_bit + (char)third_bit + (char)fourth_bit;
        //Convert string to int using hex format
        num = (uint16_t) strtoul(str.c_str(), NULL, 16);

        color_vals[counter] = num;
        counter++;

        //Transmit received num back to serial port.
        //Might not be needed except for debugging purposes
        itoa((int) num, num_str, 10);
        printstring(num_str);

        /*
        if(first_byte != NO_DATA) {
            int ret = stroul
            itoa((int) first_byte, str, 10);
            printstring(str);
            printstring(" ");
        }
        */
    }

	return 0;


}
//https://www.geeksforgeeks.org/implement-itoa/

/* A utility function to reverse a string  */
void reverse(char str[], int length)
{
    int start = 0;
    int end = length -1;
    while (start < end)
    {
        std::swap(*(str+start), *(str+end));
        start++;
        end--;
    }
}

// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitly, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled only with
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);

    return str;
}

/////////////////////////////////////////////////////////////////////////////////////
// printstring - Function to output a string to the terminal using uartTX()
// Arguments: string - Pointer to string
// Return Value: none
////////////////////////////////////////////////////////////////////////////////////
void printstring(char* str) {

    //Transmit each char in string until '\0' is encountered
    while (*str) {
           uartTX(*(str++));
    }

}
