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
#include "stm8s_uart2.h"
#include "main.h"
#include "lcd.h"

volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[21];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[7];
volatile uint8_t ui8_tx_counter = 0;
volatile uint8_t ui8_i;
volatile uint8_t ui8_checksum;
volatile uint8_t ui8_checksum_1st_package;
volatile uint8_t ui8_checksum_2nd_package;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;

void uart2_init (void)
{
  UART2_DeInit();
  UART2_Init((uint32_t) 9600,
	     UART2_WORDLENGTH_8D,
	     UART2_STOPBITS_1,
	     UART2_PARITY_NO,
	     UART2_SYNCMODE_CLOCK_DISABLE,
	     UART2_MODE_TXRX_ENABLE);

  UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
}

// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  static uint8_t _ui8_rx_buffer[21];


  if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    ui8_byte_received = UART2_ReceiveData8 ();

    switch (ui8_state_machine)
    {
      case 0:
      if (ui8_byte_received == 67) // see if we get start package byte
      {
        ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
_ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
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
_ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
      ui8_rx_counter++;

      // see if is the last byte of the package
      if (ui8_rx_counter > 21)
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

void clock_uart_data (void)
{
  struct_motor_controller_data *p_motor_controller_data;

  if (ui8_received_package_flag)
  {
    // validation of the 1st package data
    // last byte is the checksum
    ui8_checksum = 0;
    for (ui8_i = 0; ui8_i <= 7; ui8_i++)
    {
      ui8_checksum += ui8_rx_buffer[ui8_i];
    }
    ui8_checksum_1st_package = ui8_checksum % 256;
    if (ui8_checksum_1st_package == ui8_rx_buffer [8]) { ui8_checksum_1st_package = 1; }
    else { ui8_checksum_1st_package = 0; }

    // validation of the 2nd package data
    // last byte is the checksum
    ui8_checksum = 0;
    for (ui8_i = 9; ui8_i <= 19; ui8_i++)
    {
      ui8_checksum += ui8_rx_buffer[ui8_i];
    }
    ui8_checksum_2nd_package = ui8_checksum % 256;
    if (ui8_checksum_2nd_package == ui8_rx_buffer [20]) { ui8_checksum_2nd_package = 1; }
    else { ui8_checksum_2nd_package = 0; }

    // see if both checksum are ok...
    if (ui8_checksum_1st_package && ui8_checksum_2nd_package)
    {
      p_motor_controller_data = lcd_get_motor_controller_data ();

      p_motor_controller_data->ui8_battery_level = ui8_rx_buffer[1];
      p_motor_controller_data->ui8_motor_state = ui8_rx_buffer[2];
      p_motor_controller_data->ui8_pedal_torque_sensor_offset = ui8_rx_buffer[3];
      p_motor_controller_data->ui8_pedal_torque_sensor = ui8_rx_buffer[4];
      p_motor_controller_data->ui8_error_code = ui8_rx_buffer[5];
      p_motor_controller_data->ui16_wheel_rps = ui8_rx_buffer[6] << 8 + ui8_rx_buffer[7];
      p_motor_controller_data->ui8_battery_current = ui8_rx_buffer[10];
      p_motor_controller_data->ui8_brake_state = ui8_rx_buffer[11];

      // now send the data to the motor controller
      // start up byte
      ui8_tx_buffer[0] = 0x59;
      //byte contains flags for the selected step,light and 6kmh. 40=01000000=step1. bits from left to right as i know: unknown,step1,6kmh-active,off,step4,step3,step2,headlight
      ui8_tx_buffer[1] = 0x10;
      if (p_motor_controller_data->ui8_assist_level == 1) ui8_tx_buffer[1] = 0x40;
      if (p_motor_controller_data->ui8_assist_level == 2) ui8_tx_buffer[1] = 0x02;
      if (p_motor_controller_data->ui8_assist_level == 3) ui8_tx_buffer[1] = 0x04;
      if (p_motor_controller_data->ui8_assist_level == 4) ui8_tx_buffer[1] = 0x08;
      // not sure
      ui8_tx_buffer[2] = 0;
      // wheel size, 26 = 26inch
      ui8_tx_buffer[3] = 26;
      // not sure
      ui8_tx_buffer[4] = 0;
      // target max wheel speed, 25 = 25(kmh?)
      ui8_tx_buffer[5] = 45;

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
    }

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }
}

#if __SDCC_REVISION < 9624
void putchar(char c)
{
  //Write a character to the UART2
  UART2_SendData8(c);

  //Loop until the end of transmission
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
}
#else
int putchar(int c)
{
  //Write a character to the UART2
  UART2_SendData8(c);

  //Loop until the end of transmission
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);

  return((unsigned char)c);
}
#endif

#if __SDCC_REVISION < 9989
char getchar(void)
#else
int getchar(void)
#endif
{
  uint8_t c = 0;

  /* Loop until the Read data register flag is SET */
  while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET) ;

  c = UART2_ReceiveData8();

  return (c);
}
