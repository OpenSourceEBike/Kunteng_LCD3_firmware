/*
 * LCD3 firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main (void);

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_cruise_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

int main (void)
{
  //set clock at the max 16MHz
  CLK_HSIPrescalerConfig (CLK_PRESCALER_HSIDIV1);

  gpio_init ();

  GPIO_Init(GPIOA,
            GPIO_PIN_2,
            GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_WriteHigh(GPIOA, GPIO_PIN_2);

  while (1)
  {
    if (!GPIO_ReadInputPin(LCD_BUTTON_DOWN__PORT, LCD_BUTTON_DOWN__PIN))
    {
      GPIO_WriteReverse(LCD_BACKLIGHT__PORT, LCD_BACKLIGHT__PIN);

      while (!GPIO_ReadInputPin(LCD_BUTTON_DOWN__PORT, LCD_BUTTON_DOWN__PIN)) ;
    }
  }

  return 0;
}

