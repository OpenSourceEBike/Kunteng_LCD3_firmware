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
#define DEFAULT_VALUE_ASSIST_LEVEL                                  3
#define DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS                       5
#define DEFAULT_VALUE_WHEEL_PERIMETER_0                             2 // 26'' wheel: 2050mm perimeter (2 + (8 << 8))
#define DEFAULT_VALUE_WHEEL_PERIMETER_1                             8
#define DEFAULT_VALUE_WHEEL_MAX_SPEED                               50
#define DEFAULT_VALUE_UNITS_TYPE                                    0 // 0 = km/h
#define DEFAULT_VALUE_WH_OFFSET                                     0
#define DEFAULT_VALUE_HW_X10_100_PERCENT                            0
#define DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC                      0
#define DEFAULT_VALUE_ODOMETER_FIELD_STATE                          0
#define DEFAULT_VALUE_BATTERY_MAX_CURRENT                           16 // 16 amps
#define DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER                      50 // 500 watts x10
#define DEFAULT_VALUE_BATTERY_CELLS_NUMBER                          13 // 13 --> 48V
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0             134 // 48v battery, LVC = 39.0 (3.0 * 13): (134 + (1 << 8))
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1             1
#define DEFAULT_VALUE_PAS_MAX_CADENCE                               110 // 110 RPM
#define DEFAULT_VALUE_CONFIG_0                                      0 // ui8_motor_voltage_type = 0; ui8_motor_assistance_startup_config = 0
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_1                         5 // 0.5
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_2                         9
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_3                         13
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_4                         16
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_5                         255 // here, get max assistance when pressing a little the pedals
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_6                         0
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_7                         0
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_8                         0
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_9                         0
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_STATE               1
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1      25 // 50W
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2      35
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3      55
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4      75
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_5      195
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_6      195
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_7      195
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_8      195
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_9      195
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_TIME                25 // 2.5 seconds


// *************************************************************************** //

// Torque sensor value found experimentaly
// measuring with a cheap digital hook scale, we found that each torque sensor unit is equal to 0.556 Nm
// using the scale, was found that each 1kg was measured as 3 torque sensor units
// Force (Nm) = Kg * 9.18 * 0.17 (arm cranks size)
#define TORQUE_SENSOR_FORCE_SCALE_X1000 556

// *************************************************************************** //
// BATTERY

// ADC Battery voltage
// 0.344 per ADC_8bits step: 17.9V --> ADC_8bits = 52; 40V --> ADC_8bits = 116; this signal atenuated by the opamp 358
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 44
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 (ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 >> 1)
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP 0.344

// ADC Battery current
// 1A per 5 steps of ADC_10bits
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512 102
// *************************************************************************** //

#endif // _MAIN_H_
