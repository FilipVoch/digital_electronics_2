/***********************************************************************
 * 
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2022 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/

// Definitions of outputs and edge servo positions
#define SERVO_X PB1 
#define SERVO_Y PB2
#define POSITION_2 10
#define POSITION_1 1250

#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <stdlib.h>         // C library. Needed for number conversions

// Definitions of momentary servo position variables
volatile uint16_t servo_x = POSITION_1;
volatile uint16_t servo_y = POSITION_2;

int main(void)
{
    // Configure 8-bit Timer/Counter0 to control servo motors
    // Set prescaler to 16 ms and enable overflow interrupt
    TIM0_overflow_16ms();
    TIM0_overflow_interrupt_enable();

    // Configuration of the servo outputs
    GPIO_mode_output(&DDRB, SERVO_X);        // Servo 1
    GPIO_mode_output(&DDRB, SERVO_Y);        // Servo 2

    // Mode of operation for Timer1 control registers A and B: PWM, phase 
    // correct, TOP source set to ICR1, update of OCR1A/B set to TOP
    TCCR1A |= (1 << WGM11);                  
    TCCR1B |= (1 << WGM13);

    // Setting of the compare registers A and B 
    TCCR1A |= (1 << COM0A1) | (1 << COM0B1); 

    // TOP value setting
    ICR1 = 2500;
     
    // Setting of the Timer1 output compare registers A and B to the values of  
    // servo momentary position variables
    OCR1A = servo_x;
    OCR1B = servo_y;
    
    // Setting of Timer1, register B prescaler to clk/64
    TCCR1B |= (1 << CS11) | (1 << CS10); 

    // Setting pin change interrupt control register bit PCIE0 to 1 
    PCICR |= (1<<PCIE0); 
    // Setting pin change mask register bit PCINT0 to 1
    PCMSK0 |= (1<<PCINT0);                 
  

    // Enables interrupts by setting the global interrupt mask
    sei();
    
    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines, ISRs */
    }

    // Will never reach this
    return 0;
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer0 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 16 ms.
 **********************************************************************/

ISR(TIMER0_OVF_vect)
{
    // Definitions of servo step variables
    static uint8_t servo_x_step = 0;        
    static uint8_t servo_y_step = 0;
    
    // Definition of increment variable used to set servo step value
    static uint8_t increment = 1;   

    // Movement of servo x
    if (servo_x == POSITION_1)      
      {
        servo_x_step = 1;           
      }
    else if (servo_x == POSITION_2) 
      {
        servo_x_step = 0;          
      }
       
    if (servo_x_step == 0)
      {
        servo_x += increment;              
      }
        
    if (servo_x_step == 1)
      {
        servo_x -= increment;               
      }
    OCR1A = servo_x;                
       
    // Movement of servo y  
    if (servo_y == POSITION_1)      
      {
        servo_y_step = 1;              
      }
    else if (servo_y == POSITION_2) 
      {
        servo_y_step = 0;               
      }
       
    if (servo_y_step == 0)
      {
        servo_y += increment + 1;               
      }

    if (servo_y_step == 1)
      {
        servo_y -= increment + 1;                
      }
    OCR1B = servo_y;                  
}
