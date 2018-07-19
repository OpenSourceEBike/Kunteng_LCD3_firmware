/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _LCD_H_
#define _LCD_H_

#include "main.h"
#include "stm8s_gpio.h"

//ui8_rx_buffer[2] == 8 if torque sensor
//ui8_rx_buffer[2] == 4 if motor running
typedef struct _motor_controller_data
{
  uint16_t ui16_adc_battery_voltage;
  uint8_t ui8_battery_state;
  uint8_t ui8_battery_current;
  uint8_t ui8_motor_controller_state_1;
  uint8_t ui8_adc_throttle;
  uint8_t ui8_throttle;
  uint8_t ui8_adc_pedal_torque_sensor;
  uint8_t ui8_pedal_torque_sensor;
  uint8_t ui8_pedal_human_power;
  uint8_t ui8_duty_cycle;
  uint8_t ui8_error_code;
  uint16_t ui16_wheel_speed_x10;
  uint8_t ui8_motor_controller_state_2;
  uint8_t ui8_pedal_cadence;
  uint8_t ui8_lights;
  uint8_t ui8_walk_assist_level;
  uint16_t ui16_motor_speed_erps;
  uint8_t ui8_foc_angle;
} struct_motor_controller_data;

typedef struct _configuration_variables
{
  uint8_t ui8_assist_level;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_max_speed;
  uint8_t ui8_units_type;
  uint32_t ui32_wh_x10_offset;
  uint32_t ui32_wh_x10_100_percent;
  uint8_t ui8_show_numeric_battery_soc;
  uint8_t ui8_odometer_field_state;
  uint8_t ui8_target_max_battery_power;
} struct_configuration_variables;

// LCD RAM has 32*8 bits
#define LCD_FRAME_BUFFER_SIZE 32

extern uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

#define ASSIST_LEVEL_FIELD     0
#define ODOMETER_FIELD         1
#define TEMPERATURE_FIELD      2
#define WHEEL_SPEED_FIELD      3
#define BATTERY_POWER_FIELD    4

// each digit needs 7 bits to be defined + 1 digit that can be another symbol like a "point"
#define ASSIST_LEVEL_DIGIT_OFFSET     1 // 8
#define ODOMETER_DIGIT_OFFSET         6
#define TEMPERATURE_DIGIT_OFFSET      8
#define WHEEL_SPEED_OFFSET            14
#define BATTERY_POWER_DIGIT_OFFSET    10

#define NUMBERS_MASK              8
#define NUMBER_0_MASK             119
#define NUMBER_1_MASK             66  // 2; 7
#define NUMBER_2_MASK             182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK             214
#define NUMBER_4_MASK             195
#define NUMBER_5_MASK             213
#define NUMBER_6_MASK             245
#define NUMBER_7_MASK             70
#define NUMBER_8_MASK             247
#define NUMBER_9_MASK             215
#define NUMBER_0_MASK_INVERTED    119
#define NUMBER_1_MASK_INVERTED    33  // 2; 7
#define NUMBER_2_MASK_INVERTED    182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK_INVERTED    181
#define NUMBER_4_MASK_INVERTED    225
#define NUMBER_5_MASK_INVERTED    213
#define NUMBER_6_MASK_INVERTED    215
#define NUMBER_7_MASK_INVERTED    49
#define NUMBER_8_MASK_INVERTED    247
#define NUMBER_9_MASK_INVERTED    245

// : from timer label ui8_lcd_frame_buffer[23] |= 8

void lcd_init (void);
void lcd_enable_vol_symbol (uint8_t ui8_state);
void lcd_enable_w_symbol (uint8_t ui8_state);
void lcd_enable_odometer_point_symbol (uint8_t ui8_state);
void lcd_enable_brake_symbol (uint8_t ui8_state);
void lcd_enable_assist_symbol (uint8_t ui8_state);
void lcd_enable_battery_power_1_symbol (uint8_t ui8_state);
void lcd_enable_temperature_1_symbol (uint8_t ui8_state);
void lcd_enable_kmh_symbol (uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state);
void lcd_enable_battery_symbols (uint8_t ui8_state);
void lcd_update (void);
void lcd_clear (void);
void lcd_set_frame_buffer (void);
void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options);
void clock_lcd (void);
struct_configuration_variables* get_configuration_variables (void);
struct_motor_controller_data* lcd_get_motor_controller_data (void);

#endif /* _LCD_H_ */
