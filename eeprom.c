/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_flash.h"
#include "eeprom.h"
#include "main.h"
#include "lcd.h"

static struct_configuration_variables *p_configuration_variables;

void eeprom_write_array (uint8_t *array_values);

void eeprom_init (void)
{
  uint8_t array_default_values [EEPROM_BYTES_STORED] = {
    KEY,
    DEFAULT_VALUE_ASSIST_LEVEL,
    DEFAULT_VALUE_WHEEL_SIZE,
    DEFAULT_VALUE_MAX_SPEED,
    DEFAULT_VALUE_UNITS_TYPE,
    DEFAULT_VALUE_WH_OFFSET,
    DEFAULT_VALUE_WH_OFFSET,
    DEFAULT_VALUE_WH_OFFSET,
    DEFAULT_VALUE_WH_OFFSET,
    DEFAULT_VALUE_HW_X10_100_PERCENT,
    DEFAULT_VALUE_HW_X10_100_PERCENT,
    DEFAULT_VALUE_HW_X10_100_PERCENT,
    DEFAULT_VALUE_HW_X10_100_PERCENT,
    DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC,
    DEFAULT_VALUE_ODOMETER_FIELD_STATE,
  };
  uint8_t ui8_data;

  p_configuration_variables = get_configuration_variables ();

  // start by reading address 0 and see if value is different from our key,
  // if so mean that eeprom memory is clean and we need to populate: should happen after erasing the microcontroller
  ui8_data = FLASH_ReadByte (ADDRESS_KEY);
  if (ui8_data != KEY) // verify if our key exist
  {
    eeprom_write_array (array_default_values);
  }
}

void eeprom_read_values_to_variables (void)
{
  static uint8_t ui8_temp;
  static uint32_t ui32_temp;

  p_configuration_variables->ui8_assist_level = FLASH_ReadByte (ADDRESS_ASSIST_LEVEL);
  p_configuration_variables->ui8_wheel_size = FLASH_ReadByte (ADDRESS_WHEEL_SIZE);
  p_configuration_variables->ui8_max_speed = FLASH_ReadByte (ADDRESS_MAX_SPEED);
  p_configuration_variables->ui8_units_type = FLASH_ReadByte (ADDRESS_UNITS_TYPE);

  ui32_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_3);
  ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
  p_configuration_variables->ui32_wh_x10_offset = ui32_temp;

  ui32_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_3);
  ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
  p_configuration_variables->ui32_wh_x10_100_percent = ui32_temp;

  p_configuration_variables->ui8_show_numeric_battery_soc = FLASH_ReadByte (ADDRESS_SHOW_NUMERIC_BATTERY_SOC);
  p_configuration_variables->ui8_odometer_field_state = FLASH_ReadByte (ADDRESS_ODOMETER_FIELD_STATE);
  p_configuration_variables->ui8_target_max_battery_power = FLASH_ReadByte (ADDRESS_TARGET_MAX_BATTERY_POWER);
}

void eeprom_write_variables_values (void)
{
  static uint8_t array_values [EEPROM_BYTES_STORED];

  array_values [0] = KEY;
  array_values [1] = p_configuration_variables->ui8_assist_level;
  array_values [2] = p_configuration_variables->ui8_wheel_size;
  array_values [3] = p_configuration_variables->ui8_max_speed;
  array_values [4] = p_configuration_variables->ui8_units_type;
  array_values [5] = p_configuration_variables->ui32_wh_x10_offset & 255;
  array_values [6] = (p_configuration_variables->ui32_wh_x10_offset >> 8) & 255;
  array_values [7] = (p_configuration_variables->ui32_wh_x10_offset >> 16) & 255;
  array_values [8] = (p_configuration_variables->ui32_wh_x10_offset >> 24) & 255;
  array_values [9] = p_configuration_variables->ui32_wh_x10_100_percent & 255;
  array_values [10] = (p_configuration_variables->ui32_wh_x10_100_percent >> 8) & 255;
  array_values [11] = (p_configuration_variables->ui32_wh_x10_100_percent >> 16) & 255;
  array_values [12] = (p_configuration_variables->ui32_wh_x10_100_percent >> 24) & 255;
  array_values [13] = p_configuration_variables->ui8_show_numeric_battery_soc;
  array_values [14] = p_configuration_variables->ui8_odometer_field_state;
  array_values [15] = p_configuration_variables->ui8_target_max_battery_power;

  eeprom_write_array (array_values);
}

void eeprom_write_array (uint8_t *array_values)
{
  uint8_t ui8_i;

  if (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == 0)
  {
    FLASH_Unlock (FLASH_MEMTYPE_DATA);
  }

  for (ui8_i = 0; ui8_i < EEPROM_BYTES_STORED; ui8_i++)
  {
    FLASH_ProgramByte (EEPROM_BASE_ADDRESS + ui8_i, *array_values++);
  }

  FLASH_Lock (FLASH_MEMTYPE_DATA);
}

