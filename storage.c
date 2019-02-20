/***
** Created by Aleksey Volkov on 25/11/2018.
***/

#include "storage.h"

storage_t storage;

void init_storage()
{
  storage.soil_sensor_config.dry_mv       = 3200;
  storage.soil_sensor_config.normal_mv    = 2400;
  storage.soil_sensor_config.weet_mv      = 1200;

  storage.ec_sensor_config.ref_hight_us   = 2000;
  storage.ec_sensor_config.ref_low_us     = 1000;
  storage.ec_sensor_config.cal_one_point  = 0;
  storage.ec_sensor_config.cal_hight      = 0;
  storage.ec_sensor_config.cal_low        = 0;
  storage.ec_sensor_config.use_one_poit_calibration = false;
  storage.ec_sensor_config.use_two_poit_calibration = false;
}

void get_storage(storage_t * config)
{
  config = &storage;
}