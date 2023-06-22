#include "tm4c123gh6pm.h"
#include <utility>
#include <cstdint>
#include <string>
#include <array>
#include <stdio.h>
#include <iostream>

//SPECIAL NOTE: PB6 is connected to PD0 by a 0 ohm resistor (R9): Should be removed later.
#define R1 BIT2         //PF2
#define G1 BIT5         //PB5
#define B1 BIT4         //PB4
#define R2 BIT1         //PE1
#define G2 BIT2         //PE2
#define B2 BIT3         //PE3
#define LATCH BIT3      //PF3
#define CLK BIT1        //PF1
#define OE BIT4         //PF4
#define A BIT0          //PD0
#define B BIT1          //PD1
#define C BIT2          //PD2
#define D BIT3          //PD3
#define E BIT6          //PD6
#define TIME_LOAD unsigned(156250 * 256)
#define BITS_PER_COLOR 8
#define COLOR_ARRAY_SIZE 4096
#define U0RX BIT0
#define U0TX BIT1
#define IBRD_VAL 43
#define FBRD_VAL 26
#define _1SEC unsigned(80e6)

unsigned extract_bits(unsigned val, unsigned num_bits, unsigned start_pos);
void rgb_24(unsigned rgb, unsigned &r, unsigned &g, unsigned &b);

std::array<unsigned, 4096> colors = {0};

int main() {

    //Configure the system clock to operate at 80MHz
    SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;  // Enable advanced RCC2 mode
    SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;  // Bypass PLL while configuring

    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~SYSCTL_RCC_XTAL_M) | SYSCTL_RCC_XTAL_16MHZ;  // Set crystal frequency (16 MHz) in RCC register

    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~SYSCTL_RCC2_OSCSRC2_M) | SYSCTL_RCC2_OSCSRC2_MO;
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;  // Power up PLL

    SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // Set system divider to divide by 400
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) | (4 << 22);

    while (!(SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS)) {}  // Wait until PLL is locked
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;  // Enable PLL

    //Activate clock for GPIO Port A, B, C, D, E, and F.
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC  | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    // activate clock for Timer 0 Module
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;

    //Activate clock for UART Module 0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;

    //Enable RXD/TXD pins for UART0 on Port A pins 0 and 1
    GPIO_PORTA_AFSEL_R |= U0RX | U0TX;
    GPIO_PORTA_DEN_R |= U0RX | U0TX;
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~(GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M)) | GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX; // UART mode

    GPIO_PORTF_DIR_R |= R1 | LATCH | CLK | OE;
    GPIO_PORTF_DEN_R |= R1 | LATCH | CLK | OE;

    GPIO_PORTD_DIR_R |= A | B | C | D | E;
    GPIO_PORTD_DEN_R |= A | B | C | D | E;

    GPIO_PORTB_DIR_R |= G1 | B1;
    GPIO_PORTB_DEN_R |= G1 | B1;

    GPIO_PORTE_DIR_R |= R2 | G2 | B2;
    GPIO_PORTE_DEN_R |= R2 | G2 | B2;

    GPIO_PORTF_DATA_R &= ~R1;
    GPIO_PORTD_DATA_R &= ~(A | B | C | D | E);
    GPIO_PORTB_DATA_R &= ~(G1 | B1);
    GPIO_PORTE_DATA_R &= ~(R2 | G2 | B2);

    //Configure timer0 A for periodic mode, 32-bit timer
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // ensure time is disabled
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER; // set to 32-bit mode
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // set to one-shot mode, count up
    TIMER0_TAILR_R = _1SEC / TIME_LOAD; // set the interval
    TIMER0_ICR_R = TIMER_ICR_TATOCINT; // clear the time out interrupt flag

    //Configure UART0 for 8 bits at 115200 baud rate, and 8 byte FIFO interrupt
    UART0_CTL_R &= ~UART_CTL_UARTEN;  //Disable UART
    UART0_IBRD_R = IBRD_VAL;
    UART0_FBRD_R = FBRD_VAL;
    UART0_LCRH_R = UART_LCRH_WLEN_8; //8-bit word
    UART0_LCRH_R |= UART_LCRH_FEN;   //Enable FIFO
    UART0_IFLS_R |= UART_IFLS_RX4_8; //UART_IFLS_RX2_8; //Set trigger level to 8 bytes
    UART0_CTL_R |= UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE; //Enable UART

    UART0_IM_R |= UART_IM_RXIM;
    UART0_ICR_R = UART_ICR_RXIC;

    //UART0 is vector number 21 and interrupt number 5 (for EN register).
    NVIC_EN0_R |= BIT5;

    unsigned r1,g1,b1,r2,g2,b2;

    while(1) {

        for (unsigned k = 0; k < BITS_PER_COLOR; k++) {

            //Change timer load value to 2 times previous value
            unsigned shifted = 1<< k;
            TIMER0_TAILR_R = _1SEC / TIME_LOAD * shifted ;

            for (unsigned i = 0; i < 32; i++) {

                for (unsigned j = i * 64; j < 64 * (1+i); j++) {
                    GPIO_PORTF_DATA_R &= ~CLK;

                    GPIO_PORTF_DATA_R &= ~R1;
                    GPIO_PORTB_DATA_R &= ~(G1 | B1);
                    GPIO_PORTE_DATA_R &= ~(R2 | G2 | B2);

                    //Extract the RGB values in each color for the top and bottom of the matrix
                    rgb_24(colors[j],r1,g1,b1);
                    rgb_24(colors[j+2048],r2,g2,b2);

                    //Check if the bits in the RGB values based on the current BCM delay are non-zero.

                    if(r1 & shifted) {
                        GPIO_PORTF_DATA_R |= R1;
                    }

                    if (g1 & shifted) {
                        GPIO_PORTB_DATA_R |= G1;
                    }

                    if (b1 & shifted) {
                        GPIO_PORTB_DATA_R |= B1;
                    }

                    if(r2 & shifted) {
                        GPIO_PORTE_DATA_R |= R2;
                    }

                    if (g2 & shifted) {
                        GPIO_PORTE_DATA_R |= G2;
                    }

                    if (b2 & shifted) {
                        GPIO_PORTE_DATA_R |= B2;
                    }

                    GPIO_PORTF_DATA_R |= CLK;

                }

                GPIO_PORTF_DATA_R |= OE;

                //A,B,C,D are in PD0-PD3 but E is PD6 so the below must be used for correct operation.
                //Also note that moving these statements can lead to ghosting.
                GPIO_PORTD_DATA_R &= ~(A | B | C | D | E);
                GPIO_PORTD_DATA_R |= (i & 15) | ((i & BIT4) << 2);

                GPIO_PORTF_DATA_R |= LATCH;

                GPIO_PORTF_DATA_R &= ~LATCH;

                GPIO_PORTF_DATA_R &= ~OE;


           }


           TIMER0_CTL_R |= TIMER_CTL_TAEN; // start timer
           while ((TIMER0_RIS_R & TIMER_RIS_TATORIS) == 0) {} //Delay for given BCM time
           TIMER0_ICR_R = TIMER_ICR_TATOCINT;
           TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer

        }

    }
}

/////////////////////////////////////////////////////////
// uart0AISR - UART0 Interrupt Service Routine
//
// Triggers when 8 bytes are in FIFO and converts them to
// a color value.
/////////////////////////////////////////////////////////
extern "C" void uart0ISR(void) { //Since we are using C++ `extern "C"` must be here.

    UART0_ICR_R = UART_ICR_RXIC; //Clear interrupt flag
    static unsigned array_count = 0;
    char bits[] = {' ',' ',' ',' ',' ',' ',' ',' ','\0'};
    char data;

    //Load bytes from FIFO into char array
    for (unsigned i = 0; i < BITS_PER_COLOR; i++) {
        data = UART0_DR_R;
        bits[i] = data;
    }

    if(array_count >= COLOR_ARRAY_SIZE) {
        array_count = 0;
    }


    //Convert char array to hex number.
    colors[array_count] = (unsigned) strtoul(bits,NULL,16);
    array_count++;

    //Transmit a character back to indicate data was sent.
    //Also helps to delay the serial writes from going to fast for the microcontroller.
    while (UART0_FR_R & UART_FR_TXFF) {}
    UART0_DR_R = '1';

}

////////////////////////////////////////////////////////////////////////
// extract_bits
// Extracts a group of bits from an unsigned integer.
//
// param val The value to extract bits from.
// param num_bits The number of bits to extract.
// param start_pos The start position from which bits will be extracted.
//
// return The desired extracted bits.
////////////////////////////////////////////////////////////////////////
inline unsigned extract_bits(unsigned val, unsigned num_bits, unsigned start_pos) {
    unsigned  mask = ((1 << num_bits) - 1) << start_pos;
    return (val & mask) >> start_pos;
}


///////////////////////////////////////////////////////////////////////////////////
// rgb_24
// Takes a 24-bit color value and extracts the red, green, and blue bits from it.
//
// param rgb The 24-bit color value.
// param r The red bits.
// param g The green bits.
// param b The blue bits.
//
// return This function does not return and will modify by the last 3 variables by
//        reference.
//////////////////////////////////////////////////////////////////////////////////
inline void rgb_24(unsigned rgb, unsigned &r, unsigned &g, unsigned &b) {
    r = extract_bits(rgb, 8, 16);
    g = extract_bits(rgb, 8, 8);
    b = extract_bits(rgb, 8, 0);
}
