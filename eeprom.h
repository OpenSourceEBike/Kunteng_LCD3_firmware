/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"

#define KEY                                 0xca

#define EEPROM_BASE_ADDRESS 			          0x4000
#define ADDRESS_KEY 				                0 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL 			          1 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SIZE	 		            2 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_SPEED	 		              3 + EEPROM_BASE_ADDRESS
#define ADDRESS_UNITS_TYPE                  4 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_0             5 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_1             6 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_2             7 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_3             8 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_FIELD_STATE        9 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                 10

void eeprom_init (void);
void eeprom_read_values_to_variables (void);
void eeprom_write_variables_values (void);

#endif /* _EEPROM_H_ */
