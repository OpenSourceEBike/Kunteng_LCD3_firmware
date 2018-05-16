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

uint8_t ui8_counter = 0;
uint8_t ui8_data = 0;
uint8_t ui8_address = 0;

void ht1622_send_bits(uint16_t ui16_data, uint8_t ui8_bits);
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
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);

  GPIO_Init(LCD3_HT1622_WRITE__PORT,
            LCD3_HT1622_WRITE__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

  GPIO_Init(LCD3_HT1622_DATA__PORT,
            LCD3_HT1622_DATA__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);

  GPIO_Init(LCD3_HT1622_READ__PORT,
            LCD3_HT1622_READ__PIN,
            GPIO_MODE_IN_PU_NO_IT);

  ht1622_send_command(CMD_SYS_EN);
  ht1622_send_command(CMD_RC_INT);
  ht1622_send_command(CMD_LCD_ON);
}

void ht1622_send_bits(uint16_t ui16_data, uint8_t ui8_bits)
{
  static uint16_t ui16_mask;

  ui16_mask = 1 << (ui8_bits - 1);

  for (uint8_t i = ui8_bits; i > 0; i--)
  {
    GPIO_WriteLow(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

    if (ui16_data & ui16_mask)
    {
      GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);
    }
    else
    {
      GPIO_WriteLow(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);
    }

    GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

    ui16_data <<= 1;
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

  ui8_counter = 0;
  ui8_data = 0;
  ui8_address = 0;
}

void ht1622_increase_symbols(void)
{
  ui8_counter++;
  if (ui8_counter >= 4)
  {
    ui8_counter = 0;
    ui8_data = 0;
    ui8_address = (ui8_address + 1) % 64;
  }

  ui8_data += 1 << ui8_counter;

  ht1622_write_data(ui8_address, ui8_data, 4);
}
