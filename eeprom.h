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
#define ADDRESS_KEY 				                EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL 			          1 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SIZE	 		            2 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_SPEED	 		              3 + EEPROM_BASE_ADDRESS
#define ADDRESS_UNITS_TYPE                  4 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                 5

void eeprom_init (void);
void eeprom_write_if_values_changed (void);

#endif /* _EEPROM_H_ */
