/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#define EXTI_PORTA_IRQHANDLER 3
#define EXTI_PORTC_IRQHANDLER 5
#define EXTI_PORTD_IRQHANDLER 6
#define EXTI_PORTE_IRQHANDLER 7
#define TIM1_CAP_COM_IRQHANDLER   12
#define TIM2_UPD_OVF_TRG_BRK_IRQHANDLER 13
#define UART2_IRQHANDLER 21
#define ADC1_IRQHANDLER 22

// *************************************************************************** //
// EEPROM memory variables default values
#define DEFAULT_VALUE_ASSIST_LEVEL   1
#define DEFAULT_VALUE_WHEEL_SIZE    20 // 26''
#define DEFAULT_VALUE_MAX_SPEED     25
// *************************************************************************** //

typedef struct _configuration_variables
{
  uint8_t ui8_assist_level;
  uint8_t ui8_wheel_size;
  uint8_t ui8_max_speed;
} struct_configuration_variables;

struct_configuration_variables* get_configuration_variables (void);

#endif // _MAIN_H_
