/***
** Created by Aleksey Volkov on 14/11/2018.
***/

#ifndef SOILSENSOR_BAT_SENSOR_H
#define SOILSENSOR_BAT_SENSOR_H

#include "nrfx.h"

void charge_sense_init(void);

void bat_sensor_init(void);
void bat_sensor_deinit(void);

void read_battery_voltage();
uint8_t get_battery_capacity();

#endif //SOILSENSOR_BAT_SENSOR_H
