/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "adc.h"

void adc_init (void)
{
  //init GPIO for the used ADC pins
  GPIO_Init(GPIOE,
      GPIO_PIN_7,
      GPIO_MODE_IN_FL_NO_IT);

//  GPIO_Init(GPIOB,
//      GPIO_PIN_4,
//      GPIO_MODE_IN_FL_NO_IT);

  //init ADC1 peripheral
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
      ADC1_CHANNEL_8,
      ADC1_PRESSEL_FCPU_D18,
      ADC1_EXTTRIG_TIM,
      DISABLE,
      ADC1_ALIGN_LEFT,
      0,
      DISABLE);

  ADC1_ScanModeCmd (ENABLE);
  ADC1_Cmd (ENABLE);
  ADC1_StartConversion ();
}

uint16_t ui16_adc_read_battery_voltage_10b (void)
{
  uint16_t temph;
  uint8_t templ;

  // 0x53E0 + 2*8 = 0x53F8
  templ = *(uint8_t*)(0x53F9);
  temph = *(uint8_t*)(0x53F8);

  return ((uint16_t) temph) << 2 | ((uint16_t) templ);
}

uint16_t ui16_adc_read_ain4_10b (void)
{
  uint16_t temph;
  uint8_t templ;

  // 0x53E0 + 2*4 = 0x53F0
  templ = *(uint8_t*)(0x53E9);
  temph = *(uint8_t*)(0x53E8);

  return ((uint16_t) temph) << 2 | ((uint16_t) templ);
}
