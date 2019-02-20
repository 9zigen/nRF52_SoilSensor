/***
** Created by Aleksey Volkov on 15/11/2018.
***/

#ifndef SOILSENSOR_SOIL_SENSOR_H
#define SOILSENSOR_SOIL_SENSOR_H

#include "nrfx.h"

void soil_sensor_init(void);
void soil_sensor_deinit(void);

/* Start PPI + TIMER + ADC. PWM + ADC Sync sampling  */
void read_soil(void);

/* Return soil level (Dry, Fine, Wet)*/
uint16_t get_soil_level(void);

#endif //SOILSENSOR_SOIL_SENSOR_H
