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
#define ENCODER_SW PD3
#define JOYSTICK_SW PD2

#define ENCODER_A PB4
#define ENCODER_B PB5

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions



#define PD3 3          // In Arduino world, PB5 is called "13"
#define PD2 2
#define PB4 12
#define PB5 13

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
    
    GPIO_mode_input_pullup(&DDRD, JOYSTICK_SW);
    GPIO_mode_input_pullup(&DDRD, ENCODER_SW);

    GPIO_mode_input_nopull(&DDRB, ENCODER_A);
    GPIO_mode_input_nopull(&DDRB, ENCODER_B);
    


    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);

    // Select input channel ADC0 (voltage divider pin)
   
    // Enable ADC module
    ADCSRA |= (1 << ADEN);
    // Enable conversion complete interrupt
    ADCSRA |= (1 << ADIE);
    // Set clock prescaler to 128
    ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2));

    

   // Select input channel ADC1 (voltage divider pin)
    //ADMUX |= ((1 << MUX0) | (0 << MUX1) | (0 << MUX2) | (0 << MUX3));
    

    // Configure 16-bit Timer/Counter1 to start ADC conversion
    // Set prescaler to 33 ms and enable overflow interrupt
    TIM1_overflow_33ms();
    TIM1_overflow_interrupt_enable();

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
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 100 ms.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
   // ADC
   static uint8_t channel = 0;
   if (channel == 0)
   {
    ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
    channel = 1;
   }
   else
   {
    ADMUX &= ~((1 << MUX1) | (1 << MUX2) | (1 << MUX3));
    ADMUX |= (1 << MUX0);
    channel = 0;
   }
    
   // Start ADC conversion
    ADCSRA |= (1 << ADSC);



    // Switche

    static uint8_t encodeButtVal = 0;
    encodeButtVal = GPIO_read(&PIND, ENCODER_SW);

    static uint8_t buttonVal = 0;
    buttonVal = GPIO_read(&PIND, JOYSTICK_SW);

    if (buttonVal == 0)
    {
       lcd_gotoxy(5, 0); 
       lcd_puts("z:");
    }
    else
    {
      lcd_gotoxy(5, 0); 
       lcd_puts("x:");
    }
    
    if (encodeButtVal == 0)
    {
       lcd_gotoxy(5, 1); 
       lcd_puts("z:");
    }
    else
    {
      lcd_gotoxy(5, 1); 
       lcd_puts("y:");
    }
   

    
    

}

/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{
   static uint8_t channel = 0;
    uint16_t value;
   
    char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    
    //value = ADC;
    // Convert "value" to "string" and display it

    if (channel == 0)
    {
    value = ADC;

    itoa(value, string, 10);
    lcd_gotoxy(8,0);
    lcd_puts(string);

    channel = 1;
    }
    
    else if (channel == 1)
    
    {
    value = ADC;

    itoa(value, string, 10);
    lcd_gotoxy(8,1);
    lcd_puts(string);

    channel = 0;
    }

  
    
}