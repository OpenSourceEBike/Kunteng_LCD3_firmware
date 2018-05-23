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

// HT1622 commands (Start with 0b100, ends with one "don't care" byte)
#define  CMD_SYS_DIS  0x00  // SYS DIS    (0000-0000-X) Turn off system oscillator, LCD bias gen [Default]
#define  CMD_SYS_EN   0x01  // SYS EN     (0000-0001-X) Turn on  system oscillator
#define  CMD_LCD_OFF  0x02  // LCD OFF    (0000-0010-X) Turn off LCD display [Default]
#define  CMD_LCD_ON   0x03  // LCD ON     (0000-0011-X) Turn on  LCD display
#define  CMD_RC_INT   0x10  // RC INT     (0001-10XX-X) System clock source, on-chip RC oscillator


#define ASSIST_LEVEL_FIELD     0
#define ODOMETER_FIELD         1

// each digit needs 7 bits to be defined + 1 digit that can be another symbol like a "point"
#define ASSIST_LEVEL_DIGIT_OFFSET     1 // 8
#define ODOMETER_DIGIT_OFFSET         6 // 48

#define NUMBERS_MASK     8
#define NUMBER_0_MASK    119
#define NUMBER_1_MASK    66  // 2; 7
#define NUMBER_2_MASK    182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK    214
#define NUMBER_4_MASK    195
#define NUMBER_5_MASK    213
#define NUMBER_6_MASK    245
#define NUMBER_7_MASK    70
#define NUMBER_8_MASK    247
#define NUMBER_9_MASK    215

void lcd_init (void);
void lcd_enable_w_symbol (uint8_t ui8_state);
void lcd_enable_odometer_point_symbol (uint8_t ui8_state);
void lcd_send_frame_buffer (void);
void lcd_clear_frame_buffer (void);
void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field);

#endif /* _LCD_H_ */
