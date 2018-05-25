/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <string.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "gpio.h"
#include "timers.h"
#include "ht162.h"
#include "lcd.h"

uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

uint8_t ui8_lcd_field_offset[] = { ASSIST_LEVEL_DIGIT_OFFSET, ODOMETER_DIGIT_OFFSET, 0, 0, 0, 0 };
uint8_t ui8_lcd_digit_mask[] = {
    NUMBER_0_MASK,
    NUMBER_1_MASK,
    NUMBER_2_MASK,
    NUMBER_3_MASK,
    NUMBER_4_MASK,
    NUMBER_5_MASK,
    NUMBER_6_MASK,
    NUMBER_7_MASK,
    NUMBER_8_MASK,
    NUMBER_9_MASK
};

void lcd_init (void)
{
  lcd_clear_frame_buffer ();
  lcd_send_frame_buffer();
}

void lcd_clear_frame_buffer (void)
{
  memset(ui8_lcd_frame_buffer, 0, LCD_FRAME_BUFFER_SIZE);
}

void lcd_send_frame_buffer (void)
{
  ht1622_send_frame_buffer (ui8_lcd_frame_buffer);
}

void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field)
{
  uint8_t ui8_digit;
  uint8_t ui8_counter;

  // first delete the field
  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;

    if (ui8_counter == 0 && ui8_lcd_field == 0) break;
    if (ui8_counter == 4 && ui8_lcd_field == 1) break;
    if (ui8_counter == 1 && ui8_lcd_field == 2) break;
    if (ui8_counter == 2 && ui8_lcd_field == 3) break;
    if (ui8_counter == 2 && ui8_lcd_field == 4) break;
    if (ui8_counter == 4 && ui8_lcd_field == 5) break;
  }

  // print digit by digit
  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    ui8_digit = ui32_number % 10;

    // print only first 2 zeros
    if (ui8_counter > 1 && ui32_number == 0)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
    }
    else
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
    }

    if (ui8_counter == 0 && ui8_lcd_field == 0) break;
    if (ui8_counter == 1 && ui8_lcd_field == 2) break;
    if (ui8_counter == 2 && ui8_lcd_field == 3) break;
    if (ui8_counter == 2 && ui8_lcd_field == 4) break;
    if (ui8_counter == 4 && ui8_lcd_field == 5) break;

    ui32_number /= 10;
  }
}

void lcd_enable_w_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 128;
  else
    ui8_lcd_frame_buffer[9] &= ~128;
}

void lcd_enable_odometer_point_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[6] |= 8;
  else
    ui8_lcd_frame_buffer[6] &= ~8;
}
