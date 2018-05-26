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
#include "adc.h"
#include "main.h"
#include "config.h"
#include "button.h"
#include "eeprom.h"

uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

uint8_t ui8_lcd_field_offset[] = {
    ASSIST_LEVEL_DIGIT_OFFSET,
    ODOMETER_DIGIT_OFFSET,
    WHEEL_SPEED_OFFSET,
    BATTERY_POWER_DIGIT_OFFSET,
    0,
    0
};

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

uint8_t ui8_lcd_digit_mask_inverted[] = {
    NUMBER_0_MASK_INVERTED,
    NUMBER_1_MASK_INVERTED,
    NUMBER_2_MASK_INVERTED,
    NUMBER_3_MASK_INVERTED,
    NUMBER_4_MASK_INVERTED,
    NUMBER_5_MASK_INVERTED,
    NUMBER_6_MASK_INVERTED,
    NUMBER_7_MASK_INVERTED,
    NUMBER_8_MASK_INVERTED,
    NUMBER_9_MASK_INVERTED
};

uint16_t ui16_adc_battery_voltage_accumulated = 0;
uint16_t ui16_adc_battery_voltage_filtered;

struct_motor_controller_data motor_controller_data;
struct_configuration_variables configuration_variables;

void read_battery_voltage (void);

void lcd_init (void)
{
  ht1622_init ();
  lcd_clear_frame_buffer ();
  lcd_send_frame_buffer();

  // init variables with the stored value on EEPROM
  motor_controller_data.ui8_assist_level = configuration_variables.ui8_assist_level;
  motor_controller_data.ui8_wheel_size = configuration_variables.ui8_wheel_size;
  motor_controller_data.ui8_max_speed = configuration_variables.ui8_max_speed;
  motor_controller_data.ui8_units_type = configuration_variables.ui8_units_type;
}

void clock_lcd (void)
{
  static uint8_t ui8_button_events;
  float f_battery_voltage;
  float f_battery_power;

  // read battery voltage and low pass filter
  read_battery_voltage ();

  ui8_button_events = button_get_events ();

  f_battery_voltage = ((float) ui16_adc_battery_voltage_filtered * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10);
  f_battery_power = (float) motor_controller_data.ui8_battery_current * 5.0 * f_battery_voltage;

  lcd_enable_vol_symbol (1);
  lcd_enable_w_symbol (1);
  lcd_print ((uint16_t) f_battery_voltage, ODOMETER_FIELD);
  lcd_print (f_battery_power, BATTERY_POWER_FIELD);
  lcd_send_frame_buffer ();

  // now write values to EEPROM, but only if one of them changed
//  configuration_variables.ui8_assist_level =
//  configuration_variables.ui8_max_speed
//  configuration_variables.ui8_wheel_size
//  configuration_variables.ui8_units_type
//  eeprom_write_if_values_changed ();
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
    if (ui8_lcd_field == ASSIST_LEVEL_FIELD ||
            ui8_lcd_field == ODOMETER_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD ||
        ui8_lcd_field == BATTERY_POWER_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;
    }


    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == 4) break;
    if (ui8_counter == 4 && ui8_lcd_field == 5) break;
  }

  // print digit by digit
  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    ui8_digit = ui32_number % 10;

    if (ui8_lcd_field == ASSIST_LEVEL_FIELD ||
        ui8_lcd_field == ODOMETER_FIELD)
    {
      // print only first 2 zeros
      if (ui8_counter > 1 && ui32_number == 0)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      else
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
      }
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD ||
        ui8_lcd_field == BATTERY_POWER_FIELD)
    {
      // print only first zero
      if (ui8_counter > 0 && ui32_number == 0)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      else
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
      }
    }

    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == 4) break;
    if (ui8_counter == 4 && ui8_lcd_field == 5) break;

    ui32_number /= 10;
  }
}

void lcd_enable_vol_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[2] |= 255;
  else
    ui8_lcd_frame_buffer[2] &= ~255;
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

void lcd_enable_brake_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 4;
  else
    ui8_lcd_frame_buffer[23] &= ~4;
}

void lcd_enable_lights_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 2;
  else
    ui8_lcd_frame_buffer[23] &= ~2;
}

void lcd_enable_battery_symbols (uint8_t ui8_state)
{

  ui8_lcd_frame_buffer[23] |= 16;  // empty
  ui8_lcd_frame_buffer[23] |= 128; // bar number 1
  ui8_lcd_frame_buffer[23] |= 1;   // bar number 2
  ui8_lcd_frame_buffer[23] |= 64;  // bar number 3
  ui8_lcd_frame_buffer[23] |= 32;  // bar number 4

//  switch (ui8_state)
//  {
//    case 0:
//
//
//  }
//
//
}


// : from timer label ui8_lcd_frame_buffer[23] |= 8


void read_battery_voltage (void)
{
  // low pass filter the voltage readed value, to avoid possible fast spikes/noise
  ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui16_adc_battery_voltage_accumulated += ((uint16_t) ui16_adc_read_battery_voltage_10b ());
  ui16_adc_battery_voltage_filtered = ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
}

struct_configuration_variables* get_configuration_variables (void)
{
  return &configuration_variables;
}

struct_motor_controller_data* lcd_get_motor_controller_data (void)
{
  return &motor_controller_data;
}
