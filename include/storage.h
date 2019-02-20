/***
** Created by Aleksey Volkov on 25/11/2018.
***/

#ifndef SOILSENSOR_STORAGE_H
#define SOILSENSOR_STORAGE_H


#include "nrfx.h"

typedef struct {
  uint16_t dry_mv;
  uint16_t normal_mv;
  uint16_t weet_mv;
} soil_sensor_config_t;

typedef struct {
  uint16_t ref_low_us;
  uint16_t ref_hight_us;
  uint16_t cal_hight;
  uint16_t cal_low;
  float cal_one_point;
  bool use_one_poit_calibration;
  bool use_two_poit_calibration;
} ec_sensor_config_t;

typedef struct {
  ec_sensor_config_t ec_sensor_config;
  soil_sensor_config_t soil_sensor_config;
} storage_t;

void init_storage();
void get_storage(storage_t * config);

#endif //SOILSENSOR_STORAGE_H
