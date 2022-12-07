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
#define JOYSTICK_SW PD2

#define SERVO_X PB2
#define SERVO_Y PB3

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions

volatile int stepX;
volatile int stepY;

#define PB2 10        // In Arduino world, PB5 is called "13"
#define PB3 11 

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

    GPIO_mode_output(&DDRB, SERVO_X);
    GPIO_mode_output(&DDRB, SERVO_Y);
    


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
    TIM0_overflow_16us();
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
 * Purpose:  Use single conversion mode and start conversion every 100 ms.
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
   static uint16_t no_of_overflows = 0;
   
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

    static uint8_t buttonVal = 0;
    buttonVal = GPIO_read(&PIND, JOYSTICK_SW);

    no_of_overflows ++;

  if (stepX == 0)
  {
    GPIO_write_high(&PORTB, SERVO_X);
    if (no_of_overflows > 62)
    {
    
      GPIO_write_low(&PORTB, SERVO_X);
      if (no_of_overflows > 1250)
      {
        no_of_overflows = 0;
  
      }
   }
  }
  else if (stepX == 1 || buttonVal == 1)
  {
     GPIO_write_high(&PORTB, SERVO_X);
     if (no_of_overflows > 93)
     {
    
     GPIO_write_low(&PORTB, SERVO_X);
      if (no_of_overflows > 1250)
      {
       no_of_overflows = 0;

      }
  }
  }
else if (stepX == 2)
  {
     GPIO_write_high(&PORTB, SERVO_X);
     if (no_of_overflows > 125)
     {
     GPIO_write_low(&PORTB, SERVO_X);
      if (no_of_overflows > 1250)
      {
       no_of_overflows = 0;

      }
  }
  }



if (stepY == 0)
  {
    GPIO_write_high(&PORTB, SERVO_Y);
    if (no_of_overflows > 62)
    {
    
      GPIO_write_low(&PORTB, SERVO_Y);
      if (no_of_overflows > 1250)
      {
        no_of_overflows = 0;
  
      }
   }
  }
  else if (stepY == 1)
  {
     GPIO_write_high(&PORTB, SERVO_Y);
     if (no_of_overflows > 93)
     {
    
     GPIO_write_low(&PORTB, SERVO_Y);
      if (no_of_overflows > 1250)
      {
       no_of_overflows = 0;

      }
  }
  }
else if (stepY == 2)
  {
     GPIO_write_high(&PORTB, SERVO_Y);
     if (no_of_overflows > 125)
     {
    
     GPIO_write_low(&PORTB, SERVO_Y);
      if (no_of_overflows > 1250)
      {
       no_of_overflows = 0;

      }
  }
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
    uint8_t out1;
    uint8_t out2;
   
    char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    
    //value = ADC;
    // Convert "value" to "string" and display it

    if (channel == 0)
    {
    value = ADC;

  
    
    if (value < 400)
    {
      stepX = 0;
    }
    else if (value < 650)
    {
      stepX = 1;
    }
    else 
    {
      stepX = 2;
    }
    
    itoa(value, string, 10);
    lcd_gotoxy(8,0);
    lcd_puts(string);

    channel = 1;
    }
    
    else if (channel == 1)
    
    {
    value = ADC;

    


    if (value < 400)
    {
      stepY = 0;
    }
    else if (value < 650)
    {
      stepY = 1;
    }
    else 
    {
      stepY = 2;
    }

    itoa(value, string, 10);
    lcd_gotoxy(8,1);
    lcd_puts(string);

    channel = 0;
    }

    
}