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
#define DEFAULT_VALUE_ASSIST_LEVEL          2
#define DEFAULT_VALUE_WHEEL_SIZE            26
#define DEFAULT_VALUE_MAX_SPEED             45
#define DEFAULT_VALUE_UNITS_TYPE            1 // 1 = km/h
#define DEFAULT_VALUE_WH                    0
#define DEFAULT_VALUE_ODOMETER_FIELD_STATE  0
// *************************************************************************** //

#endif // _MAIN_H_
