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


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions


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
    lcd_gotoxy(1, 0); lcd_puts("value:");
    lcd_gotoxy(3, 1); lcd_puts("key:");
    lcd_gotoxy(8, 0); lcd_puts("a");  // Put ADC value in decimal
    lcd_gotoxy(13,0); lcd_puts("b");  // Put ADC value in hexadecimal
    lcd_gotoxy(8, 1); lcd_puts("c");  // Put button name here

    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);

    // Select input channel ADC0 (voltage divider pin)
    ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
    // Enable ADC module
    ADCSRA |= (1 << ADEN);
    // Enable conversion complete interrupt
    ADCSRA |= (1 << ADIE);
    // Set clock prescaler to 128
    ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2));

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
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
}

/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{
    uint16_t value;
    char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    value = ADC;
    // Convert "value" to "string" and display it
    itoa(value, string, 10);
    if(value > 999)
    {
      lcd_gotoxy(8,0);
      lcd_puts(string);
    }else{
      if(value>99)
      {
        lcd_gotoxy(9,0);
        lcd_puts(string);
        lcd_gotoxy(8,0);
        lcd_putc('0');
      }else{
        if(value>9)
        {
          lcd_gotoxy(10,0);
          lcd_puts(string);
          lcd_gotoxy(8,0);
          lcd_putc('0');
          lcd_gotoxy(9,0);
          lcd_putc('0');
        }else{
          lcd_gotoxy(11,0);
          lcd_puts(string);
          lcd_gotoxy(8,0);
          lcd_putc('0');
          lcd_gotoxy(9,0);
          lcd_putc('0');
          lcd_gotoxy(10,0);
          lcd_putc('0');
        }
      }
    }
    if(value > 255)
    {
      itoa(value, string, 16);
      lcd_gotoxy(13,0);
      lcd_puts(string);
    }else{
      if(value > 15)
      {
        itoa(value, string, 16);
        lcd_gotoxy(14,0);
        lcd_puts(string);
        lcd_gotoxy(13,0);
        lcd_putc('0');
      }else{
        itoa(value, string, 16);
        lcd_gotoxy(15,0);
        lcd_puts(string);
        lcd_gotoxy(13,0);
        lcd_putc('0');
        lcd_gotoxy(14,0);
        lcd_putc('0');
      }
    }

    if(value < 25)
    {
      lcd_gotoxy(8,1);
      lcd_puts("right ");
    }else{
      if(value < 125)
      {
        lcd_gotoxy(8,1);
        lcd_puts("up    ");
      }else{
        if(value < 280)
        {
          lcd_gotoxy(8,1);
          lcd_puts("down  ");
        }else{
          if(value < 450)
          {
            lcd_gotoxy(8,1);
            lcd_puts("left  ");
          }else{
            if(value < 670)
            {
              lcd_gotoxy(8,1);
              lcd_puts("select");
            }else{
              lcd_gotoxy(8,1);
              lcd_puts("none  ");
            }
          }
        }
      }
    }
    
}