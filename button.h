/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "main.h"
#include "stm8s_gpio.h"

uint8_t get_button_up_state (void);
uint8_t get_button_down_state (void);
uint8_t get_button_onnoff_state (void);
void clock_button (void);
uint8_t button_get_events (void);

#endif /* _BUTTON_H_ */
