/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "gpio.h"
#include "timers.h"
#include "ht1622.h"

// HT1622 commands (Start with 0b100, ends with one "don't care" byte)
#define CMD_SYS_DIS  0   // SYS DIS    (0000-0000-X) Turn off system oscillator, LCD bias gen [Default]
#define CMD_SYS_EN   1   // SYS EN     (0000-0001-X) Turn on  system oscillator
#define CMD_LCD_OFF  2   // LCD OFF    (0000-0010-X) Turn off LCD display [Default]
#define CMD_LCD_ON   3   // LCD ON     (0000-0011-X) Turn on  LCD display
#define CMD_RC_INT   24  // RC INT     (0001-10XX-X) System clock source, on-chip RC oscillator

void ht1622_send_bits(uint16_t data, uint8_t bits);
void ht1622_send_command(uint8_t command);
void ht1622_write_data(uint8_t address, uint16_t data, uint8_t bits);

// Init HT1622:
// 1. select system clock source
// 2. system enable
// 3. set the display data in data memory
// 4. turn on the LCD display
// 5. update the display data in data memory.
void ht1622_init (void)
{
  GPIO_Init(LCD3_HT1622_CS__PORT,
            LCD3_HT1622_CS__PIN,
            GPIO_MODE_OUT_OD_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);

  GPIO_Init(LCD3_HT1622_WRITE__PORT,
            LCD3_HT1622_WRITE__PIN,
            GPIO_MODE_OUT_OD_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

  GPIO_Init(LCD3_HT1622_DATA__PORT,
            LCD3_HT1622_DATA__PIN,
            GPIO_MODE_OUT_OD_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);

  GPIO_Init(LCD3_HT1622_READ__PORT,
            LCD3_HT1622_READ__PIN,
            GPIO_MODE_IN_PU_NO_IT);

  // reset
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
  GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);
  GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);
  GPIO_WriteLow(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
  GPIO_WriteLow(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

  delay_8us (100); // 50ms
  ht1622_send_command(CMD_RC_INT);
  delay_8us (100); // 50ms
  ht1622_send_command(CMD_SYS_EN);
  delay_8us (100); // 50ms
  ht1622_enable_all_segments(1);
  delay_8us (100); // 50ms
  ht1622_send_command(CMD_LCD_ON);
  delay_8us (100); // 50ms
}

void ht1622_send_bits(uint16_t data, uint8_t bits)
{
  uint16_t ui16_mask;

  ui16_mask = 1 << (bits - 1);

  for (uint8_t i = bits; i > 0; i--)
  {
    GPIO_WriteLow(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);
    delay_8us (4); // 8 us

    if (data && ui16_mask) { GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN); }
    else { GPIO_WriteLow(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN); }
    delay_8us (4); // 8 us

    GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);
    delay_8us (4); // 8 us

    data <<= 1;
  }
}

void ht1622_send_command(uint8_t command)
{
  GPIO_WriteLow(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
  ht1622_send_bits(4, 3);
  ht1622_send_bits(command, 8);
  ht1622_send_bits(1, 1);
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
}

void ht1622_write_data(uint8_t address, uint16_t data, uint8_t bits)
{
  GPIO_WriteLow(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
  ht1622_send_bits(5, 3);
  ht1622_send_bits(address, 6);
  ht1622_send_bits(data, bits);
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
}

void ht1622_enable_all_segments(uint8_t state)
{
  for (uint8_t address = 0; address < 63; address++)
  {
    ht1622_write_data(address, (state ? 0xff : 0x00), 4);
  }

  delay_8us(100); // ~500ms
}
