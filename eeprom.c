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

void eeprom_write_array (uint8_t *array_values);

void eeprom_init (void)
{
  uint8_t array_default_values [EEPROM_BYTES_STORED] = {
    KEY,
    DEFAULT_VALUE_ASSIST_LEVEL,
    DEFAULT_VALUE_WHEEL_SIZE,
    DEFAULT_VALUE_MAX_SPEED,
    DEFAULT_VALUE_UNITS_TYPE,
    DEFAULT_VALUE_WH,
    DEFAULT_VALUE_WH,
    DEFAULT_VALUE_WH,
    DEFAULT_VALUE_WH,
    DEFAULT_VALUE_ODOMETER_FIELD_STATE,
  };
  uint8_t ui8_data;

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
  struct_configuration_variables *p_configuration_variables = get_configuration_variables ();

  p_configuration_variables->ui8_assist_level = FLASH_ReadByte (ADDRESS_ASSIST_LEVEL);
  p_configuration_variables->ui8_wheel_size = FLASH_ReadByte (ADDRESS_WHEEL_SIZE);
  p_configuration_variables->ui8_max_speed = FLASH_ReadByte (ADDRESS_MAX_SPEED);
  p_configuration_variables->ui8_units_type = FLASH_ReadByte (ADDRESS_UNITS_TYPE);
  p_configuration_variables->ui32_wh_x10 = (uint32_t) (FLASH_ReadByte (ADDRESS_HW_X10_0) |
      ((uint32_t) FLASH_ReadByte (ADDRESS_HW_X10_1)) << 8 |
      ((uint32_t) FLASH_ReadByte (ADDRESS_HW_X10_2)) << 16 |
      ((uint32_t) FLASH_ReadByte (ADDRESS_HW_X10_3)) << 24);
  p_configuration_variables->ui8_odometer_field_state = FLASH_ReadByte (ADDRESS_ODOMETER_FIELD_STATE);
}

void eeprom_write_variables_values (void)
{
  static uint8_t array_values [EEPROM_BYTES_STORED];
  struct_configuration_variables *p_configuration_variables = get_configuration_variables ();

  array_values [0] = KEY;
  array_values [1] = p_configuration_variables->ui8_assist_level;
  array_values [2] = p_configuration_variables->ui8_wheel_size;
  array_values [3] = p_configuration_variables->ui8_max_speed;
  array_values [4] = p_configuration_variables->ui8_units_type;
  array_values [5] = p_configuration_variables->ui32_wh_x10 & 255;
  array_values [6] = (p_configuration_variables->ui32_wh_x10 >> 8) & 255;
  array_values [7] = (p_configuration_variables->ui32_wh_x10 >> 16) & 255;
  array_values [8] = (p_configuration_variables->ui32_wh_x10 >> 24) & 255;
  array_values [9] = p_configuration_variables->ui8_odometer_field_state;

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
