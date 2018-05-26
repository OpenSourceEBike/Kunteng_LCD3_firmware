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
#include "pins.h"

uint8_t ui8_buttons_events;
uint8_t ui8_onoff_button_state;
uint8_t ui8_onoff_button_last_state;
uint8_t ui8_down_button_state;
uint8_t ui8_down_button_last_state;
uint8_t ui8_up_button_state;
uint8_t ui8_up_button_last_state;

uint8_t get_button_up_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN) != 0 ? 0: 1;
}

uint8_t get_button_down_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN) != 0 ? 0: 1;
}

uint8_t get_button_onnoff_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN) != 0 ? 1: 0;
}

uint8_t button_get_events (void)
{
  return ui8_buttons_events;
}

void clock_button (void)
{
  ui8_onoff_button_state = get_button_onnoff_state ();
  if (ui8_onoff_button_last_state == 0 &&
      ui8_onoff_button_state == 1)
  {
    // start counting for long press
  }
  // look for release of the button: click event
  else if (ui8_onoff_button_last_state == 1 &&
      ui8_onoff_button_state == 0)
  {
    ui8_buttons_events |= 1 << 0;
  }
  ui8_onoff_button_last_state = ui8_onoff_button_state;


  ui8_down_button_state = get_button_down_state ();
  if (ui8_down_button_last_state == 0 &&
      ui8_down_button_state == 1)
  {
    // start counting for long press
  }
  // look for release of the button: click event
  else if (ui8_down_button_last_state == 1 &&
      ui8_down_button_state == 0)
  {
    ui8_buttons_events |= 1 << 2;
  }
  ui8_down_button_last_state = ui8_down_button_state;


  ui8_up_button_state = get_button_up_state ();
  if (ui8_up_button_last_state == 0 &&
      ui8_up_button_state == 1)
  {
    // start counting for long press
  }
  // look for release of the button: click event
  else if (ui8_up_button_last_state == 1 &&
      ui8_up_button_state == 0)
  {
    ui8_buttons_events |= 1 << 4;
  }
  ui8_up_button_last_state = ui8_up_button_state;
}
