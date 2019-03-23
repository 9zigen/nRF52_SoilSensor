/***
** Created by Aleksey Volkov on 13/11/2018.
***/

#ifndef SOILSENSOR_TIMERS_H
#define SOILSENSOR_TIMERS_H

#include <sensors/include/sensors.h>

void timers_init(void);
void advertising_update_timer_start(void);
void advertising_update_timer_stop();

/* Always running slow periodical sensor reading */
void read_sensor_timer_start(bool fast);
void read_sensor_timer_stop(void);

/* If connected start update characteristics */
void update_sensor_characteristic_start(sensor_t sensor_id);

/* If disconnected stop update characteristics */
void update_sensor_characteristic_stop(sensor_t sensor_id);

#endif //SOILSENSOR_TIMERS_H
