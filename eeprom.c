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

struct_configuration_variables *p_configuration_variables;

void eeprom_read_values_to_variables (void);
void eeprom_read_values_to_variables (void);
void eeprom_write_array (uint8_t *array_values);

void eeprom_init (void)
{
  uint8_t array_default_values [EEPROM_BYTES_STORED] = {
    KEY,
    DEFAULT_VALUE_ASSIST_LEVEL,
    DEFAULT_VALUE_WHEEL_SIZE,
    DEFAULT_VALUE_MAX_SPEED,
  };
  uint8_t ui8_data;
  p_configuration_variables = get_configuration_variables ();

  // start by reading address 0 and see if value is different from our key,
  // if so mean that eeprom memory is clean and we need to populate: should happen after erasing the microcontroller
  ui8_data = FLASH_ReadByte (ADDRESS_KEY);
  if (ui8_data != KEY) // verify if our key exist
  {
    eeprom_write_array (array_default_values);
    eeprom_read_values_to_variables ();
  }
  else // values on eeprom memory should be ok, now use them
  {
    eeprom_read_values_to_variables ();
  }
}

void eeprom_read_values_to_variables (void)
{
  p_configuration_variables->ui8_assist_level = FLASH_ReadByte (ADDRESS_ASSIST_LEVEL);
  p_configuration_variables->ui8_wheel_size = FLASH_ReadByte (ADDRESS_WHEEL_SIZE);
  p_configuration_variables->ui8_max_speed = FLASH_ReadByte (ADDRESS_MAX_SPEED);
}

void eeprom_write_if_values_changed (void)
{
  static uint8_t array_values [EEPROM_BYTES_STORED];

  // see if the values differ from the ones on EEPROM and if so, write all of them to EEPROM
  if ((p_configuration_variables->ui8_assist_level != FLASH_ReadByte (ADDRESS_ASSIST_LEVEL)) ||
      (p_configuration_variables->ui8_wheel_size != FLASH_ReadByte (ADDRESS_WHEEL_SIZE)) ||
      (p_configuration_variables->ui8_max_speed != FLASH_ReadByte (ADDRESS_MAX_SPEED)))
  {
    array_values [0] = KEY;
    array_values [1] = p_configuration_variables->ui8_assist_level;
    array_values [2] = p_configuration_variables->ui8_wheel_size;
    array_values [3] = p_configuration_variables->ui8_max_speed;

    eeprom_write_array (array_values);
  }
}

void eeprom_write_array (uint8_t *array_values)
{
  uint8_t ui8_i;

  FLASH_SetProgrammingTime (FLASH_PROGRAMTIME_STANDARD);
  FLASH_Unlock (FLASH_MEMTYPE_DATA);
  while (!FLASH_GetFlagStatus (FLASH_FLAG_DUL)) ;

  for (ui8_i = 0; ui8_i < EEPROM_BYTES_STORED; ui8_i++)
  {
    FLASH_ProgramByte (EEPROM_BASE_ADDRESS + ui8_i, *array_values++);
    while (!FLASH_GetFlagStatus (FLASH_FLAG_EOP)) ;
  }
  FLASH_Lock (FLASH_MEMTYPE_DATA);
}
