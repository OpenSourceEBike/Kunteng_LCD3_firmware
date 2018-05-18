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
// UART2 Receive interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

void communications_controller (void);

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

uint8_t ui8_received_package_flag = 0;
uint8_t ui8_rx_buffer[9];
uint8_t ui8_rx_counter = 0;
uint8_t ui8_i;
uint8_t ui8_crc;
uint8_t ui8_byte_received;
uint8_t ui8_state_machine = 0;

int main (void)
{
  uint16_t ui16_temp;
  uint8_t ui8_button_up_state;
  uint8_t ui8_button_down_state;
  uint8_t ui8_back_light_duty_cyle = 0;

  uint32_t ui32_number;

  static uint8_t ui8_1;
  static uint8_t ui8_2;
  static uint8_t ui8_3;
  static uint8_t ui8_4;
  static uint8_t ui8_5;
  static uint8_t ui8_6;
  static uint8_t ui8_7;
  static uint8_t ui8_8;
  static uint8_t ui8_9;
  static uint8_t ui8_10;

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

  lcd_init ();
  lcd_clear_frame_buffer ();
  lcd_send_frame_buffer();

  lcd_enable_odometer_point_symbol (1);
  lcd_print (ui32_number, ODOMETER);

  TIM1_SetCompare4 (10);

  enableInterrupts ();

  while (1)
  {
    communications_controller ();





//    ui16_temp = GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN);
//    if (ui16_temp == 0)
//    {
//      ui32_number += 10;
//      lcd_print (ui32_number, ODOMETER);
//
//      while (!GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN)) ;
//    }
//
//    ui16_temp = GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN);
//    if (ui16_temp == 0)
//    {
//      ui32_number += 1;
//      lcd_print (ui32_number, ODOMETER);
//
//      while (!GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN)) ;
//    }
//
//    ui16_temp = GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN);
//    if (ui16_temp == 0)
//    {
//      ui8_back_light_duty_cyle += 3;
//      if (ui8_back_light_duty_cyle >= 12) { ui8_back_light_duty_cyle = 0; }
//      TIM1_SetCompare4(ui8_back_light_duty_cyle);
//
//      lcd_clear_frame_buffer();
//      lcd_send_frame_buffer();
//
//      while (!GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN)) ;
//    }
  }

  return 0;
}

void communications_controller (void)
{
//  uint8_t ui8_moving_indication;
//  uint8_t ui8_battery_current;
//
//  /********************************************************************************************/
//  // Prepare and send packate to LCD
//  //
//
//  // calc wheel period in ms
//  if (f_wheel_speed < 1) { ui16_wheel_period_ms = 36000.0 * f_wheel_perimeter; } // this is needed to get LCD showing 0 km/h
//  else { ui16_wheel_period_ms = (3600.0 * f_wheel_perimeter) / f_wheel_speed; }
//
//  // calc battery pack state of charge (SOC)
//  ui16_battery_volts = ((uint16_t) ebike_app_get_ADC_battery_voltage_filtered ()) * ((uint16_t) ADC_BATTERY_VOLTAGE_K);
//  if (ui16_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_80)) { ui8_battery_soc = 16; } // 4 bars | full
//  else if (ui16_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_60)) { ui8_battery_soc = 12; } // 3 bars
//  else if (ui16_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_40)) { ui8_battery_soc = 8; } // 2 bars
//  else if (ui16_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_20)) { ui8_battery_soc = 4; } // 1 bar
//  else if (ui16_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_10)) { ui8_battery_soc = 3; } // empty
//  else { ui8_battery_soc = 1; } // flashing
//
//  // prepare error
//  ui16_error = ebike_app_get_error (); // get the error value
//  // if battery under voltage, signal instead on LCD battery symbol
//  if (ui16_error == EBIKE_APP_ERROR_91_BATTERY_UNDER_VOLTAGE)
//  {
//    ui8_battery_soc = 1; // empty flashing
//    ui16_error = 0;
//  }
//
//  // prepare moving indication info
//  ui8_moving_indication = 0;
//  if (motor_controller_state_is_set (MOTOR_CONTROLLER_STATE_BRAKE) ||
//      motor_controller_state_is_set (MOTOR_CONTROLLER_STATE_BRAKE_LIKE_COAST_BRAKES)) { ui8_moving_indication = (1 << 5); }
//  if (ebike_app_cruise_control_is_set ()) { ui8_moving_indication |= (1 << 3); }
//  if (ebike_app_throttle_is_released ()) { ui8_moving_indication |= (1 << 1); }
//  if (pas_is_set ()) { ui8_moving_indication |= (1 << 4); }

//  // if battery over voltage, signal instead on LCD with battery symbol flashing and brake signal
//  if (motor_controller_state_is_set(MOTOR_CONTROLLER_STATE_OVER_VOLTAGE))
//  {
//    ui8_battery_soc = 2; // border flashing
//    ui16_error = 0;
//    ui8_moving_indication = (1 << 5);
//  }
//
//  // preparing the package
//  // B0: start package (?)
//  ui8_tx_buffer [0] = 65;
//  // B1: battery level
//  ui8_tx_buffer [1] = ui8_battery_soc;
//  // B2: 24V controller
//  ui8_tx_buffer [2] = (uint8_t) COMMUNICATIONS_BATTERY_VOLTAGE;
//  // B3: speed, wheel rotation period, ms; period(ms)=B3*256+B4;
//  ui8_tx_buffer [3] = (ui16_wheel_period_ms >> 8) & 0xff;
//  ui8_tx_buffer [4] = ui16_wheel_period_ms & 0xff;
//  // B5: error info display
//  ui8_tx_buffer [5] = ui16_error;
//  // B6: CRC: xor B1,B2,B3,B4,B5,B7,B8,B9,B10,B11
//  // 0 value so no effect on xor operation for now
//  ui8_tx_buffer [6] = 0;
//  // B7: moving mode indication, bit
//  // throttle: 2
//  ui8_tx_buffer [7] = ui8_moving_indication;
//  // B8: 4x controller current
//  // each unit of B8 = 0.5A
//  ui8_battery_current = ebike_app_get_battery_current_filtered ();
//  ui8_tx_buffer [8] = (uint8_t) ((float) ui8_battery_current * (float) LCD_BATTERY_CURRENT_FACTOR);
//  // B9: motor temperature
//  ui8_tx_buffer [9] = 0;
//  // B10 and B11: 0
//  ui8_tx_buffer [10] = 0;
//  ui8_tx_buffer [11] = 0;
//
//  // calculate CRC xor
//  ui8_crc = 0;
//  for (ui8_i = 1; ui8_i <= 11; ui8_i++)
//  {
//    ui8_crc ^= ui8_tx_buffer[ui8_i];
//  }
//  ui8_tx_buffer [6] = ui8_crc;
//
//  // send the package over UART
//  for (ui8_i = 0; ui8_i <= 11; ui8_i++)
//  {
//    putchar (ui8_tx_buffer [ui8_i]);
//  }

  /********************************************************************************************/
  // Process received package from the LCD
  //

  // see if we have a received package to be processed
  if (ui8_received_package_flag)
  {
    // validation of the package data
    ui8_crc = 0;
    for (ui8_i = 0; ui8_i <= 8; ui8_i++)
    {
      if (ui8_i == 7) continue; // last byte is checksum
      ui8_crc += ui8_rx_buffer[ui8_i];
    }
    ui8_crc = ui8_crc % 256;

    // see if CRC is ok
//    if (ui8_crc == ui8_rx_buffer [8])
    if (1)
    {
//      lcd_configuration_variables.ui8_assist_level = ui8_rx_buffer [3] & 7;
//      lcd_configuration_variables.ui8_motor_characteristic = ui8_rx_buffer [5];
//      lcd_configuration_variables.ui8_wheel_size = ((ui8_rx_buffer [6] & 192) >> 6) | ((ui8_rx_buffer [4] & 7) << 2);
//      lcd_configuration_variables.ui8_max_speed = 10 + ((ui8_rx_buffer [4] & 248) >> 3) | (ui8_rx_buffer [6] & 32);
//      lcd_configuration_variables.ui8_power_assist_control_mode = ui8_rx_buffer [6] & 8;
//      lcd_configuration_variables.ui8_controller_max_current = (ui8_rx_buffer [9] & 15);
//
//      // now write values to EEPROM, but only if one of them changed
//      eeprom_write_if_values_changed ();

//        lcd_print ((ui8_rx_buffer[4] - 88) * 10, ODOMETER);
        lcd_print (ui8_rx_buffer[3] * 10, ODOMETER);
    }

    UART2->CR2 |= (1 << 5); // enable UART2 receive interrupt as we are now ready to receive a new package
  }

//  // do here some tasks that must be done even if we don't receive a package from the LCD
//// NOTE: we are now setup controller max current on another place
//  set_motor_controller_max_current (lcd_configuration_variables.ui8_controller_max_current);
//  ui8_wheel_speed_max = lcd_configuration_variables.ui8_max_speed;
}

// This is the interrupt that happesn when UART2 receives data. We need it to be the fastest possible and so
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
        ui8_rx_buffer[ui8_rx_counter++] = ui8_byte_received;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      ui8_rx_buffer[ui8_rx_counter++] = ui8_byte_received;

      // see if is the last byte of the package
      if (ui8_rx_counter > 8)
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
