/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _HT1622_H_
#define _HT1622_H_

#include "main.h"
#include "stm8s_gpio.h"

void ht1622_init (void);
void ht1622_enable_all_segments(uint8_t state);

#endif /* _HT1622_H_ */
