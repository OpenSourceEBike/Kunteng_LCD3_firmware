/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "timers.h"
#include "adc.h"
#include "lcd.h"
#include "uart.h"

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

// UART2 Receive interrupt
//void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// This is the interrupt that happesn when UART2 receives data.
//void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
//{
//  if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
//  {
////    ui8_byte_received = UART2_ReceiveData8 ();
//  }
//}

int main (void)
{
  uint16_t ui16_temp;
  uint8_t ui8_button_up_state;
  uint8_t ui8_button_down_state;
  uint8_t ui8_back_light_duty_cyle = 0;

  //set clock at the max 16MHz
  CLK_HSIPrescalerConfig (CLK_PRESCALER_HSIDIV1);

  gpio_init ();

  GPIO_Init(GPIOC,
            GPIO_PIN_4,
      GPIO_MODE_OUT_OD_HIZ_FAST); // enable PWM pin for LCD backlight control

  timer1_init ();
  timer3_init ();
  uart2_init ();
//  adc_init ();

  GPIO_WriteHigh(GPIOB, GPIO_PIN_4); // enable VDD to HT1622 ??
  GPIO_WriteHigh(GPIOE, GPIO_PIN_3);

  GPIO_WriteHigh(LCD3_ENABLE_BACKLIGHT_POWER__PORT, LCD3_ENABLE_BACKLIGHT_POWER__PIN);
  GPIO_WriteLow(LCD3_ENABLE_BACKLIGHT__PORT, LCD3_ENABLE_BACKLIGHT__PIN);

  lcd_init ();
  lcd_clear_frame_buffer ();
  lcd_send_frame_buffer();

  while (1)
  {
    ui16_temp = GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN);
    if (ui16_temp == 0)
    {
        lcd_enable_w_symbol(1);
      lcd_send_frame_buffer();

      while (!GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN)) ;
    }

    ui16_temp = GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN);
    if (ui16_temp == 0)
    {
        lcd_enable_w_symbol(0);
      lcd_send_frame_buffer();

      while (!GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN)) ;
    }

    ui16_temp = GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN);
    if (ui16_temp == 0)
    {
      ui8_back_light_duty_cyle += 3;
      if (ui8_back_light_duty_cyle >= 12) { ui8_back_light_duty_cyle = 0; }
      TIM1_SetCompare4(ui8_back_light_duty_cyle);

      lcd_clear_frame_buffer();
      lcd_send_frame_buffer();

      while (!GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN)) ;
    }
  }

  return 0;
}

