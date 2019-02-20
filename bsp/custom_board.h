/***
** Created by Aleksey Volkov on 23/11/2018.
***/

#ifndef SOILSENSOR_CUSTOM_BOARD_H
#define SOILSENSOR_CUSTOM_BOARD_H

#include "nrf_gpio.h"

/* RGB LED */
#define               LED_R NRF_GPIO_PIN_MAP(0,22)
#define               LED_G NRF_GPIO_PIN_MAP(0,23)
#define               LED_B NRF_GPIO_PIN_MAP(0,24)

/* BAT Sensor */
#define               BAT_ON NRF_GPIO_PIN_MAP(0,11)
#define               CHARGE_SENSE_PIN NRF_GPIO_PIN_MAP(0,30)

/* I2C Interface pin */
#define               SDA_PIN NRF_GPIO_PIN_MAP(0,26)
#define               SCL_PIN NRF_GPIO_PIN_MAP(0,27)

#endif //SOILSENSOR_CUSTOM_BOARD_H
