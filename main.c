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
#include "eeprom.h"
#include "button.h"
#include "ht162.h"

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

uint8_t ui8_received_package_flag = 0;
uint8_t ui8_rx_buffer[9];
uint8_t ui8_rx_counter = 0;
uint8_t ui8_tx_buffer[7];
uint8_t ui8_tx_counter = 0;
uint8_t ui8_i;
uint8_t ui8_checksum;
uint8_t ui8_byte_received;
uint8_t ui8_state_machine = 0;

uint8_t ui8_battery_current_filtered = 0;
uint16_t ui16_battery_current_accumulated = 0;

struct_configuration_variables configuration_variables;

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes
// UART2 Receive interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

// main -- start of firmware and main loop
int main (void);

volatile static uint8_t ui8_log;

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
  eeprom_init ();

  ht1622_init ();
  lcd_init ();


  lcd_enable_odometer_point_symbol (1);
  lcd_print (0, ODOMETER_FIELD);
  lcd_print (configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD);
  lcd_send_frame_buffer ();

  while  (1) ;

//  ui8_lcd_frame_buffer[23] |= 1;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 2;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 4;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 8;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 16;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 32;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 64;
//  lcd_send_frame_buffer ();
//
//  ui8_lcd_frame_buffer[23] |= 128;
//  lcd_send_frame_buffer ();

  TIM1_SetCompare4 (10);

  enableInterrupts ();

  while (1)
  {


//      lcd_print (ui16_adc_read_battery_voltage_10b (), ODOMETER_FIELD);
//      lcd_send_frame_buffer ();

//
//
    // see if we have a received package from UART, to be processed
    if (ui8_received_package_flag)
    {
      // validation of the package data
      // last byte is the checksum
      ui8_checksum = 0;
      for (ui8_i = 0; ui8_i <= 7; ui8_i++)
      {
        ui8_checksum += ui8_rx_buffer[ui8_i];
      }
      ui8_checksum = ui8_checksum % 256;

      // see if checksum is ok
      // NOTE: seems the firmware always send checksum = 0
//      if (ui8_checksum == ui8_rx_buffer [8])
      if (1)
      {
        // low pass filter the battery to smooth the signal
        ui16_battery_current_accumulated -= ui16_battery_current_accumulated >> 2;
        ui16_battery_current_accumulated += ((uint16_t) ui8_rx_buffer[3]);
        ui8_battery_current_filtered = ui16_battery_current_accumulated >> 2;

        //ui8_rx_buffer[2] == 8 if torque sensor
        //ui8_rx_buffer[2] == 4 if motor running


        // 64 no brake
        // 80 brake
        if (ui8_rx_buffer[3] == 80)
          lcd_enable_brake_symbol (1);
        else
          lcd_enable_brake_symbol (0);

        lcd_print (0, ODOMETER_FIELD);
        lcd_send_frame_buffer ();


//        lcd_print (ui8_battery_current_filtered << 1, ODOMETER_FIELD);
      }

      // send the packet from LCD to motor controller
//      59 40 00 1C 00 1B D0
//      1. fixed/startbyte = 59?
//      2. byte contains flags for the selected step,light and 6kmh. 40=01000000=step1. bits from left to right as i know: unknown,step1,6kmh-active,off,step4,step3,step2,headlight
//      3. not sure
//      4. wheel size, 1C hex = 28inch
//      5. not sure
//      6. max speed, 1B hex = 27(kmh?)
//      7. 1byte checksum

      ui8_tx_buffer[0] = 0x59;

      ui8_tx_buffer[1] = 0x10;
      if (configuration_variables.ui8_assist_level == 1) ui8_tx_buffer[1] = 0x40;
      if (configuration_variables.ui8_assist_level == 2) ui8_tx_buffer[1] = 0x02;
      if (configuration_variables.ui8_assist_level == 3) ui8_tx_buffer[1] = 0x04;
      if (configuration_variables.ui8_assist_level == 4) ui8_tx_buffer[1] = 0x08;

      ui8_tx_buffer[2] = 0x00;
      ui8_tx_buffer[3] = 0x1C;
      ui8_tx_buffer[4] = 0x00;
      ui8_tx_buffer[5] = 0x1B;

      ui8_checksum = 0;
      for (ui8_i = 0; ui8_i <= 5; ui8_i++)
      {
        ui8_checksum += ui8_tx_buffer[ui8_i];
      }
      ui8_checksum = ui8_checksum % 256;
      ui8_tx_buffer[6] = ui8_checksum;

      // send the full package to UART
      for (ui8_i = 0; ui8_i <= 6; ui8_i++)
      {
        putchar (ui8_tx_buffer[ui8_i]);
      }


      // signal that we processed the full package
      ui8_received_package_flag = 0;

      // enable UART2 receive interrupt as we are now ready to receive a new package
      UART2->CR2 |= (1 << 5);
    }

    if (get_button_up_state () && get_button_down_state ())
    {
      while (get_button_up_state () || get_button_down_state ()) ;

      lcd_enable_brake_symbol (1);
      lcd_send_frame_buffer ();
    }

    if (get_button_up_state ())
    {
      if (configuration_variables.ui8_assist_level < 4)
        configuration_variables.ui8_assist_level++;

      lcd_enable_brake_symbol (0);

      while (get_button_up_state () && !get_button_down_state ()) ;

      lcd_print (configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD);
    }

    if (get_button_down_state ())
    {
      if (configuration_variables.ui8_assist_level > 0)
        configuration_variables.ui8_assist_level--;

      lcd_enable_brake_symbol (0);

      while (get_button_down_state () && !get_button_up_state ()) ;

      lcd_print (configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD);
    }

    if (get_button_onnoff_state ())
    {
      lcd_enable_w_symbol (1);

      while (get_button_onnoff_state ()) ;
    }

    lcd_send_frame_buffer ();


    // now write values to EEPROM, but only if one of them changed
    eeprom_write_if_values_changed ();




//    // because of continue; at the end of each if code block that will stop the while (1) loop there,
//    // the first if block code will have the higher priority over any others
//    ui16_TIM2_counter = TIM2_GetCounter ();
//    if ((ui16_TIM2_counter - ui16_ebike_app_controller_counter) > 100) // every 100ms
//    {
//      ui16_ebike_app_controller_counter = ui16_TIM2_counter;
//      // ebike_app_controller() takes about 13ms (measured at 2018.03)
//      ebike_app_controller ();
//      continue;
//    }

#ifdef DEBUG_UART
    ui16_TIM2_counter = TIM2_GetCounter ();
    if ((ui16_TIM2_counter - ui16_debug_uart_counter) > 20) // every 20ms
    {
      ui16_debug_uart_counter = ui16_TIM2_counter;

      // sugestion: no more than 6 variables printed (takes about 3ms to printf 6 variables)
      printf ("%d,%d,%d,%d,%d\n",
        ui8_duty_cycle_target,
        ui8_duty_cycle,
        ui16_motor_get_motor_speed_erps(),
        UI8_ADC_BATTERY_CURRENT,
        ui8_angle_correction);
      continue;
    }
#endif
  }

  return 0;
}

// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    ui8_byte_received = UART2_ReceiveData8 ();

    switch (ui8_state_machine)
    {
      case 0:
      if (ui8_byte_received == 67) // see if we get start package byte
      {
        ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
        ui8_rx_counter++;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
      ui8_rx_counter++;

      // see if is the last byte of the package
      if (ui8_rx_counter > 9)
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
        ui8_received_package_flag = 1; // signal that we have a full package to be processed
        UART2->CR2 &= ~(1 << 5); // disable UART2 receive interrupt
      }
      break;

      default:
      break;
    }
  }
}

struct_configuration_variables* get_configuration_variables (void)
{
  return &configuration_variables;
}
