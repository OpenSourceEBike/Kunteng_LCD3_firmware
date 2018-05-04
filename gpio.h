/*
 * LCD3 firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

/* Connections:
 *
 * PIN		      | IN/OUT|Function
 * ----------------------------------------------------------
 * PB2     | button down, low level
 * PB1     | button up, low level
 * PB0     | button up, low level
 * PC4     | enables LCD backlight
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "main.h"
#include "stm8s_gpio.h"

#define LCD_BACKLIGHT__PIN        GPIO_PIN_4
#define LCD_BACKLIGHT__PORT       GPIOC

#define LCD_BUTTON_DOWN__PIN        GPIO_PIN_2
#define LCD_BUTTON_DOWN__PORT       GPIOB

void gpio_init (void);

#endif /* GPIO_H_ */
