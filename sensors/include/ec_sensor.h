/***
** Created by Aleksey Volkov on 14/11/2018.
***/

#ifndef SOILSENSOR_EC_SENSOR_H
#define SOILSENSOR_EC_SENSOR_H
#include <boards.h>

void ec_sensor_init(void);
void ec_sensor_deinit(void);

void process_conductivity();

uint16_t get_raw_salinity();
uint16_t get_conductivity();
void do_one_point_calibration();

#endif //SOILSENSOR_EC_SENSOR_H
