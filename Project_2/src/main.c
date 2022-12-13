/***********************************************************************
 * 
 * Use Analog-to-digital conversion to read push buttons on LCD keypad
 * shield and display it on LCD screen.
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2018 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/
#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif

#define JOYSTICK_SW PD2 // switch

#define SERVO_X PB2     // servo 1
#define SERVO_Y PB3     // servo 2


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <util/delay.h>     // Functions for busy-wait delay loops
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions

volatile uint16_t stepX;
volatile uint16_t stepY;
volatile uint8_t channel = 0;



/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and start ADC conversion every 100 ms.
 *           When AD conversion ends, send converted value to LCD screen.
 * Returns:  none
 **********************************************************************/


int main(void)
{
    // Initialize display
    
    lcd_init(LCD_DISP_ON);
    lcd_gotoxy(5, 0); lcd_puts("x:");
    lcd_gotoxy(5, 1); lcd_puts("y:");
    
    // Set pins for switch and servo 1,2 
    
    GPIO_mode_input_pullup(&DDRD, JOYSTICK_SW);

    GPIO_mode_output(&DDRB, SERVO_X);
    GPIO_mode_output(&DDRB, SERVO_Y);
    


    
    
    

    // Configure 16-bit Timer/Counter0 to start ADC conversion
    // Set prescaler to 16 ms and enable overflow interrupt
    TIM0_overflow_16ms();
    TIM0_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
    }

    // Will never reach this
    return 0;
}


/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter0 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 16 ms.
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
   
  


}

