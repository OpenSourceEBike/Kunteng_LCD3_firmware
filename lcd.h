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
  uint8_t ui8_battery_level;
  uint8_t ui8_motor_state;
  uint8_t ui8_pedal_torque_sensor_offset;
  uint8_t ui8_pedal_torque_sensor;
  uint8_t ui8_error_code;
  uint16_t ui16_wheel_rps;
  uint8_t ui8_battery_current;
  uint8_t ui8_brake_state;
  uint8_t ui8_assist_level;
  uint8_t ui8_wheel_size;
  uint8_t ui8_max_speed;
  uint8_t ui8_units_type;
} struct_motor_controller_data;

typedef struct _configuration_variables
{
  uint8_t ui8_assist_level;
  uint8_t ui8_wheel_size;
  uint8_t ui8_max_speed;
  uint8_t ui8_units_type;
} struct_configuration_variables;

// LCD RAM has 32*8 bits
#define LCD_FRAME_BUFFER_SIZE 32

extern uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

#define ASSIST_LEVEL_FIELD     0
#define ODOMETER_FIELD         1
#define WHEEL_SPEED_FIELD      2
#define BATTERY_POWER_FIELD    3

// each digit needs 7 bits to be defined + 1 digit that can be another symbol like a "point"
#define ASSIST_LEVEL_DIGIT_OFFSET     1 // 8
#define ODOMETER_DIGIT_OFFSET         6 // 48
#define WHEEL_SPEED_OFFSET            6 // 48
#define BATTERY_POWER_DIGIT_OFFSET    10 //

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

void lcd_init (void);
void lcd_enable_vol_symbol (uint8_t ui8_state);
void lcd_enable_w_symbol (uint8_t ui8_state);
void lcd_enable_odometer_point_symbol (uint8_t ui8_state);
void lcd_enable_brake_symbol (uint8_t ui8_state);
void lcd_send_frame_buffer (void);
void lcd_clear_frame_buffer (void);
void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field);
void clock_lcd (void);
struct_configuration_variables* get_configuration_variables (void);
struct_motor_controller_data* lcd_get_motor_controller_data (void);

#endif /* _LCD_H_ */
