/***
** Created by Aleksey Volkov on 2019-03-21.
***/

#ifndef SOILSENSOR_LIGHT_SENSOR_H
#define SOILSENSOR_LIGHT_SENSOR_H

typedef enum {
  ISL29035, OPT3001
} light_sensor_chip_t;

void init_light_sensor();
uint32_t get_visible_lux();
void read_light_sensor();

#endif //SOILSENSOR_LIGHT_SENSOR_H
