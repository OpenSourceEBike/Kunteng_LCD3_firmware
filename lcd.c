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
#include "pins.h"
#include "uart.h"

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

static uint32_t ui32_battery_voltage_accumulated = 0;
static uint16_t ui16_battery_voltage_filtered_x10;

static uint16_t ui16_battery_current_accumulated = 0;
static uint16_t ui16_battery_current_filtered_x5;

static uint16_t ui16_battery_power_accumulated = 0;
static uint16_t ui16_battery_power_filtered_x50;
static uint16_t ui16_battery_power_filtered;

static uint32_t ui32_wh_sum_x5 = 0;
static uint32_t ui32_wh_sum_counter = 0;
static uint32_t ui32_wh_x10 = 0;

static uint32_t ui32_torque_sensor_force_x1000;
static uint32_t ui32_torque_accumulated = 0;
static uint16_t ui32_torque_accumulated_filtered_x10;

static uint8_t ui8_motor_controller_init = 1;

static uint8_t ui8_lights_state = 0;

static struct_motor_controller_data motor_controller_data;
static struct_configuration_variables configuration_variables;

void low_pass_filter_battery_voltage_current_power (void);
void lcd_enable_motor_symbol (uint8_t ui8_state);
void lcd_enable_lights_symbol (uint8_t ui8_state);
void calc_wh (void);
void assist_level_state (void);
void brake (void);
void odometer (void);
void wheel_speed (void);
void power (void);
void power_off_management (void);
uint8_t first_time_management (void);
void battery_soc (void);
void low_pass_filter_pedal_torque (void);
void lights_state (void);
void lcd_set_backlight_intensity (uint8_t ui8_intensity);

void clock_lcd (void)
{
  if (first_time_management ())
    return;
  low_pass_filter_battery_voltage_current_power ();
  low_pass_filter_pedal_torque ();
  calc_wh ();
  assist_level_state ();
  brake ();
  odometer ();
  wheel_speed ();
  power ();
  battery_soc ();
  lcd_send_frame_buffer (); // refresh LCD
  power_off_management ();
}

uint8_t first_time_management (void)
{
  uint8_t ui8_status = 0;

  // don't update LCD up to we get first communication package from the motor controller
  if (ui8_motor_controller_init &&
      (uart_received_first_package () == 0))
  {
    ui8_status = 1;
  }
  // this will be excuted only 1 time at startup
  else if (ui8_motor_controller_init)
  {
    ui8_motor_controller_init = 0;
    lcd_clear_frame_buffer (); // let's clear LCD

    // wait for a first good read value of ADC: voltage can't be 0
    while (ui16_adc_read_battery_voltage_10b () == 0) ;

    // reset Wh value if battery is over 54.4V (when battery is near fully charged)
    if (((uint32_t) ui16_adc_read_battery_voltage_10b () * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000) > 544000)
    {
      configuration_variables.ui32_wh_x10_offset = 0;
    }
  }

  return ui8_status;
}

void power_off_management (void)
{
  // turn off
  if (get_button_onoff_long_click_event ())
  {
    // save values to EEPROM
    configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
    eeprom_write_variables_values ();

    // clear LCD so it is clear to user what is happening
    lcd_clear_frame_buffer ();
    lcd_send_frame_buffer ();

    // now disable the power to all the system
    GPIO_WriteLow(LCD3_ONOFF_POWER__PORT, LCD3_ONOFF_POWER__PIN);

    // block here
    while (1) ;
  }
}

void battery_soc (void)
{
  static uint8_t ui8_timmer_counter;

  // show on LCD only at every 100ms / 10 times per second and this helps to visual filter the fast changing values
  if (ui8_timmer_counter++ >= 10)
  {
    ui8_timmer_counter = 0;

    // battery SOC
    lcd_enable_battery_symbols (motor_controller_data.ui8_battery_level);
  }
}
void power (void)
{
  lcd_print (ui16_battery_power_filtered, BATTERY_POWER_FIELD, 0);
  lcd_enable_motor_symbol (1);
  lcd_enable_w_symbol (1);
}

void assist_level_state (void)
{
  if (get_button_up_click_event ())
  {
    clear_button_up_click_event ();

    if (configuration_variables.ui8_assist_level < 5)
      configuration_variables.ui8_assist_level++;
  }

  if (get_button_down_click_event ())
  {
    clear_button_down_click_event ();

    if (configuration_variables.ui8_assist_level > 0)
      configuration_variables.ui8_assist_level--;
  }
  motor_controller_data.ui8_assist_level = configuration_variables.ui8_assist_level;

  lcd_print (configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD, 0);
  lcd_enable_assist_symbol (1);
}

void lights_state (void)
{
  if (get_button_up_long_click_event ())
  {
    clear_button_up_long_click_event ();

    if (ui8_lights_state == 0)
    {
      ui8_lights_state = 1;
      lcd_enable_lights_symbol (1);
      lcd_set_backlight_intensity (5); // TODO: implement backlight intensity control
      motor_controller_data.ui8_lights |= 1;
    }
    else
    {
      ui8_lights_state = 0;
      lcd_enable_lights_symbol (0);
      lcd_set_backlight_intensity (0);
      motor_controller_data.ui8_lights &= ~1;
    }
  }
}

void brake (void)
{
  if (motor_controller_data.ui8_motor_controller_state_2 & 1) { lcd_enable_brake_symbol (1); }
  else { lcd_enable_brake_symbol (0); }
}

void odometer (void)
{
  uint32_t ui32_temp;

  // odometer values
  if (get_button_onoff_click_event ())
  {
    clear_button_onoff_click_event ();
    configuration_variables.ui8_odometer_field_state = (configuration_variables.ui8_odometer_field_state + 1) % 6;
  }

  switch (configuration_variables.ui8_odometer_field_state)
  {
    // Wh value
    case 0:
      lcd_print (ui32_wh_x10, ODOMETER_FIELD, 0);
      lcd_enable_vol_symbol (0);
    break;

    // voltage value
    case 1:
      lcd_print (ui16_battery_voltage_filtered_x10, ODOMETER_FIELD, 0);
      lcd_enable_vol_symbol (1);
    break;

    // current value
    case 2:
      lcd_print (ui16_battery_current_filtered_x5 << 1, ODOMETER_FIELD, 0);
      lcd_enable_vol_symbol (0);
    break;

    // pedal torque in Nm
    case 3:
      lcd_print (ui32_torque_sensor_force_x1000 / 100, ODOMETER_FIELD, 1);
      lcd_enable_vol_symbol (0);
    break;

    // pedal power in watts
    case 4:
      lcd_print (ui32_torque_accumulated_filtered_x10, ODOMETER_FIELD, 1);
      lcd_enable_vol_symbol (0);
    break;

    // pedal cadence value
    case 5:
      lcd_print (motor_controller_data.ui8_pedal_cadence * 10, ODOMETER_FIELD, 1);
      lcd_enable_vol_symbol (0);
    break;

    default:
    configuration_variables.ui8_odometer_field_state = 0;
    break;
  }
}

void wheel_speed (void)
{
  float f_wheel_speed;

  // speed value
  // the value sent by the controller is for MPH and not KMH...
  // (1÷((controller_sent_time÷3600)÷wheel_perimeter)÷1.6)
  f_wheel_speed = 1.0 / (((float) motor_controller_data.ui16_wheel_inverse_rps * 1.6) / 7380);
  // if wheel is stopped, reset speed value
  if (motor_controller_data.ui8_motor_controller_state_2 & 128)  { f_wheel_speed = 0; }

  lcd_print (f_wheel_speed, WHEEL_SPEED_FIELD, 0);
  lcd_enable_kmh_symbol (1);
  lcd_enable_wheel_speed_point_symbol (1);
}

void lcd_clear_frame_buffer (void)
{
  memset(ui8_lcd_frame_buffer, 0, LCD_FRAME_BUFFER_SIZE);
}

void lcd_set_frame_buffer (void)
{
  memset(ui8_lcd_frame_buffer, 255, LCD_FRAME_BUFFER_SIZE);
}

void lcd_send_frame_buffer (void)
{
  ht1622_send_frame_buffer (ui8_lcd_frame_buffer);
}

void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options)
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
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == 4) break;
    if (ui8_counter == 4 && ui8_lcd_field == 5) break;
  }

  // enable only the "1" if power is >= 1000
  if (ui8_lcd_field == BATTERY_POWER_FIELD && ui32_number >= 1000) { lcd_enable_battery_power_1_symbol (1); }
  else { lcd_enable_battery_power_1_symbol (0); }

  // do not show odometer point symbol if number*10 is integer
  if ((ui8_lcd_field == ODOMETER_FIELD) && (ui8_options == 1)) { lcd_enable_odometer_point_symbol (0); }
  else if (ui8_lcd_field == ODOMETER_FIELD) { lcd_enable_odometer_point_symbol (1); }

  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    ui8_digit = ui32_number % 10;

    if (ui8_lcd_field == ASSIST_LEVEL_FIELD ||
        ui8_lcd_field == ODOMETER_FIELD)
    {

      if ((ui8_options == 1) &&
          (ui8_counter == 0))
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      // print only first 2 zeros
      else if (ui8_counter > 1 && ui32_number == 0)
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
      if (ui8_lcd_field == WHEEL_SPEED_FIELD)
      {
        // print only first 2 zeros
        if (ui8_counter > 1 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
      }

      if (ui8_lcd_field == BATTERY_POWER_FIELD)
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
    }

    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
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

void lcd_enable_cruise_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[0] |= 16;
  else
    ui8_lcd_frame_buffer[0] &= ~16;
}

void lcd_enable_assist_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[1] |= 8;
  else
    ui8_lcd_frame_buffer[1] &= ~8;
}

void lcd_enable_vol_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[2] |= 8;
  else
    ui8_lcd_frame_buffer[2] &= ~8;
}

void lcd_enable_odo_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[3] |= 8;
  else
    ui8_lcd_frame_buffer[3] &= ~8;
}

void lcd_enable_km_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[4] |= 8;
  else
    ui8_lcd_frame_buffer[4] &= ~8;
}

void lcd_enable_mil_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[5] |= 8;
  else
    ui8_lcd_frame_buffer[5] &= ~8;
}

void lcd_enable_temperature_1_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[7] |= 8;
  else
    ui8_lcd_frame_buffer[7] &= ~8;
}

void lcd_enable_battery_power_1_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[12] |= 8;
  else
    ui8_lcd_frame_buffer[12] &= ~8;
}

void lcd_enable_temperature_minus_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[8] |= 8;
  else
    ui8_lcd_frame_buffer[8] &= ~8;
}

void lcd_enable_temperature_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 16;
  else
    ui8_lcd_frame_buffer[9] &= ~16;
}

void lcd_enable_temperature_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 32;
  else
    ui8_lcd_frame_buffer[9] &= ~32;
}

void lcd_enable_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 1;
  else
    ui8_lcd_frame_buffer[9] &= ~1;
}

void lcd_enable_motor_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 2;
  else
    ui8_lcd_frame_buffer[9] &= ~2;
}

void lcd_enable_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 64;
  else
    ui8_lcd_frame_buffer[9] &= ~64;
}

void lcd_enable_kmh_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 1;
  else
    ui8_lcd_frame_buffer[13] &= ~1;
}

void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 8;
  else
    ui8_lcd_frame_buffer[13] &= ~8;
}

void lcd_enable_avs_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 16;
  else
    ui8_lcd_frame_buffer[13] &= ~16;
}

void lcd_enable_mxs_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 32;
  else
    ui8_lcd_frame_buffer[13] &= ~32;
}

void lcd_enable_walk_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 64;
  else
    ui8_lcd_frame_buffer[13] &= ~64;
}

void lcd_enable_mph_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 128;
  else
    ui8_lcd_frame_buffer[13] &= ~128;
}

void lcd_enable_dst_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[16] |= 8;
  else
    ui8_lcd_frame_buffer[16] &= ~8;
}

void lcd_enable_tm_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[17] |= 16;
  else
    ui8_lcd_frame_buffer[17] &= ~16;
}

void lcd_enable_ttm_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[17] |= 32;
  else
    ui8_lcd_frame_buffer[17] &= ~32;
}

void lcd_enable_battery_symbols (uint8_t ui8_state)
{
/*
  ui8_lcd_frame_buffer[23] |= 16;  // empty
  ui8_lcd_frame_buffer[23] |= 128; // bar number 1
  ui8_lcd_frame_buffer[23] |= 1;   // bar number 2
  ui8_lcd_frame_buffer[23] |= 64;  // bar number 3
  ui8_lcd_frame_buffer[23] |= 32;  // bar number 4
  */

  // first clean battery symbols
  ui8_lcd_frame_buffer[23] &= ~241;

  if (ui8_state <= 5)
    ui8_lcd_frame_buffer[23] |= 16;
  else if ((ui8_state > 5) && (ui8_state <= 10))
    ui8_lcd_frame_buffer[23] |= 144;
  else if ((ui8_state > 10) && (ui8_state <= 15))
    ui8_lcd_frame_buffer[23] |= 145;
  else if ((ui8_state > 15) && (ui8_state <= 20))
    ui8_lcd_frame_buffer[23] |= 209;
  else
    ui8_lcd_frame_buffer[23] |= 241;
}

void low_pass_filter_battery_voltage_current_power (void)
{
  // low pass filter battery voltage
  ui32_battery_voltage_accumulated -= ui32_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui32_battery_voltage_accumulated += (uint32_t) ui16_adc_read_battery_voltage_10b () * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
  ui16_battery_voltage_filtered_x10 = ((uint32_t) (ui32_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT)) / 1000;

  // low pass filter batery current
  ui16_battery_current_accumulated -= ui16_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
  ui16_battery_current_accumulated += (uint16_t) motor_controller_data.ui8_battery_current;
  ui16_battery_current_filtered_x5 = ui16_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;

  // battery power
  ui16_battery_power_filtered_x50 = ui16_battery_current_filtered_x5 * ui16_battery_voltage_filtered_x10;
  ui16_battery_power_filtered = ui16_battery_power_filtered_x50 / 50;

  // loose resolution under 10W
  if (ui16_battery_power_filtered < 200)
  {
    ui16_battery_power_filtered /= 10;
    ui16_battery_power_filtered *= 10;
  }
  // loose resolution under 20W
  else if (ui16_battery_power_filtered < 400)
  {
    ui16_battery_power_filtered /= 20;
    ui16_battery_power_filtered *= 20;
  }
  // loose resolution under 25W
  else
  {
    ui16_battery_power_filtered /= 25;
    ui16_battery_power_filtered *= 25;
  }
}

void low_pass_filter_pedal_torque (void)
{
  uint32_t ui32_torque_x10;
  uint32_t ui32_torque_filtered_x10;

  ui32_torque_sensor_force_x1000 = motor_controller_data.ui8_pedal_torque_sensor - motor_controller_data.ui8_pedal_torque_sensor_offset;
  if (ui32_torque_sensor_force_x1000 > 200) { ui32_torque_sensor_force_x1000 = 0; }
  ui32_torque_sensor_force_x1000 *= TORQUE_SENSOR_FORCE_SCALE_X1000;

  // calc now torque
  // P = force x rotations_seconds x 2 x pi
  ui32_torque_x10 = (ui32_torque_sensor_force_x1000 * motor_controller_data.ui8_pedal_cadence) / 955;

  // low pass filter
  ui32_torque_accumulated -= ui32_torque_accumulated >> TORQUE_FILTER_COEFFICIENT;
  ui32_torque_accumulated += ui32_torque_x10;
  ui32_torque_filtered_x10 = ((uint32_t) (ui32_torque_accumulated >> TORQUE_FILTER_COEFFICIENT));

  // loose resolution under 10W
  if (ui32_torque_filtered_x10 < 1000)
  {
    ui32_torque_accumulated_filtered_x10 = ui32_torque_filtered_x10 / 50;
    ui32_torque_accumulated_filtered_x10 *= 50;
  }
  // loose resolution under 20W
  else
  {
    ui32_torque_accumulated_filtered_x10 = ui32_torque_filtered_x10 / 100;
    ui32_torque_accumulated_filtered_x10 *= 100;
  }
}

void calc_wh (void)
{
  static uint8_t ui8_100ms_timmer_counter;
  static uint8_t ui8_1s_timmer_counter;
  uint32_t ui32_temp = 0;

  // calc wh every 100ms
  if (ui8_100ms_timmer_counter++ >= 10)
  {
    ui8_100ms_timmer_counter = 0;

    if (ui16_battery_power_filtered_x50 > 0)
    {
      ui32_wh_sum_x5 += ui16_battery_power_filtered_x50 / 10;
      ui32_wh_sum_counter++;
    }

    // calc at 1s rate
    if (ui8_1s_timmer_counter++ >= 10)
    {
      ui8_1s_timmer_counter = 0;

      // avoid  zero divisison
      if (ui32_wh_sum_counter != 0)
      {
        ui32_temp = ui32_wh_sum_counter / 36;
        ui32_temp = (ui32_temp * (ui32_wh_sum_x5 / ui32_wh_sum_counter)) / 500;
      }

      ui32_wh_x10 = configuration_variables.ui32_wh_x10_offset + ui32_temp;
    }
  }
}

struct_configuration_variables* get_configuration_variables (void)
{
  return &configuration_variables;
}

struct_motor_controller_data* lcd_get_motor_controller_data (void)
{
  return &motor_controller_data;
}

void lcd_init (void)
{
  ht1622_init ();
  lcd_set_frame_buffer ();
  lcd_send_frame_buffer();

  // init variables with the stored value on EEPROM
  eeprom_read_values_to_variables ();
}

void lcd_set_backlight_intensity (uint8_t ui8_intensity)
{
  if ((ui8_intensity >= 0) && (ui8_intensity <= 9))
  {
    TIM1_SetCompare4 (ui8_intensity); // set background light
  }
}
