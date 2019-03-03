/***
** Created by Aleksey Volkov on 16/11/2018.
***/

#include <libraries/timer/app_timer.h>
#include <ble_bas.h>
#include <ble_services.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_clock.h"
#include "nrfx_ppi.h"

#include "bat_sensor.h"
#include "soil_sensor.h"
#include "sensors.h"
#include "ec_sensor.h"
#include "timers.h"
#include "shtc3.h"
#include "isl29035.h"

uint8_t sensor_index      = 0;
bool bat_sensor_ready     = false;
bool ec_sensor_ready      = false;
bool soil_sensor_ready    = false;
bool light_sensor_ready   = false;
bool th_sensor_ready      = false;

bool battery_sensor_notify     = false;
bool temperature_sensor_notify = false;
bool humidity_sensor_notify    = false;
bool salinity_sensor_notify    = false;
bool soil_sensor_notify        = false;
bool light_sensor_notify       = false;


sensors_db_t sensors_db = {0, 0, 0, 0, false};

/* save sensor values */
void store_sensor_to_db(int16_t temp, uint16_t hum, uint16_t soil, uint16_t conductivity)
{
  sensors_db.sensor_values_changed = false;

  if (temp != sensors_db.temperature)
  {
    sensors_db.temperature = temp;
    sensors_db.sensor_values_changed = true;
  }

  if (hum != sensors_db.humidity)
  {
    sensors_db.humidity = hum;
    sensors_db.sensor_values_changed = true;
  }

  if (soil != sensors_db.soil_milli_volts)
  {
    sensors_db.soil_milli_volts = soil;
    sensors_db.sensor_values_changed = true;
  }

  if (conductivity != sensors_db.conductivity_us)
  {
    sensors_db.conductivity_us = conductivity;
    sensors_db.sensor_values_changed = true;
  }

}

/* Slow read sensors timer handler */
void read_sensors_timer_handler(void *p_context)
{
  if(p_context)
  {
    /* fast run */
    if (all_sensors_ready())
    {
      /* restart read after delay 30 min */
      read_sensor_timer_stop();
      read_sensor_timer_start(false);

      NRF_LOG_INFO("update advertising data");
      /* update advertising data */
      advertising_update();
      reset_all_sensors();
      return;
    }

    NRF_LOG_INFO("-------------- READ SENSOR --------------");
    read_sensors();
  } else {
    NRF_LOG_INFO("-------------- DELAY ELAPSED ------------");
    /* delay elapsed, setup timer to fast read sensors */
    read_sensor_timer_stop();
    read_sensor_timer_start(true);
  }

}

void update_advertising_timer_handler(void *p_context)
{
  UNUSED_PARAMETER(p_context);
  if (all_sensors_ready())
  {
    advertising_update();
  }
}

/* update characteristics timer handler */
void update_characteristics_timer_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);

  /* read only one sensor, and SET Sensor Ready */
  read_sensors();

  /* update only one sensor, readied in previous call */
  if (battery_sensor_notify && sensor_ready(BATTERY_SENSOR))
  {
    update_sensors_service(BATTERY_SENSOR);
    reset_sensor(BATTERY_SENSOR);
  }
  else if (temperature_sensor_notify && sensor_ready(TEMPERATURE_SENSOR))
  {
    update_sensors_service(TEMPERATURE_SENSOR);
    reset_sensor(TEMPERATURE_SENSOR);
  }
  else if (humidity_sensor_notify && sensor_ready(HUMIDITY_SENSOR))
  {
    update_sensors_service(HUMIDITY_SENSOR);
    reset_sensor(HUMIDITY_SENSOR);
  }
  else if (salinity_sensor_notify && sensor_ready(SALINITY_SENSOR))
  {
    update_sensors_service(SALINITY_SENSOR);
    reset_sensor(SALINITY_SENSOR);
  }
  else if (soil_sensor_notify && sensor_ready(SOIL_SENSOR))
  {
    update_sensors_service(SOIL_SENSOR);
    reset_sensor(SOIL_SENSOR);
  }
  else if (light_sensor_notify && sensor_ready(LIGHT_SENSOR))
  {
    update_sensors_service(LIGHT_SENSOR);
    reset_sensor(LIGHT_SENSOR);
  }

}

/* Sensors Order
 * 0 - battery SAADC
 * 1 - soil SAADC
 * 2 - salinity SAADC
 * 3 - light i2c
 * 4 - temp + hum i2c
 * */
void read_sensors() {
  NRF_LOG_INFO("---Sensor INDEX %d", sensor_index);

  if (nrfx_saadc_is_busy()) {
    NRF_LOG_INFO("SAADC is Busy");
    return;
  }

  if (sensor_index == 0) {
    read_battery_voltage();
    sensor_index++;
  } else if (sensor_index == 1) {
    read_soil();
    sensor_index++;
  } else if (sensor_index == 2) {
    process_conductivity();
    sensor_index++;
  } else if (sensor_index == 3) {
    isl29035_read_sensor();
    sensor_index++;
  } else if (sensor_index == 4) {
    shtc3_process();
    sensor_index = 0;

    /* all sensors was asked, setup timer to delay */
    read_sensor_timer_stop();
    read_sensor_timer_start(false);
  }
}

void set_sensor_ready(sensor_ready_t sensor)
{
  switch (sensor)
  {
    case BAT_SENSOR_READY:
      bat_sensor_ready = true;
      break;

    case SOIL_SENSOR_READY:
      soil_sensor_ready = true;
      break;

    case EC_SENSOR_READY:
      ec_sensor_ready = true;
      break;

    case LIGHT_SENSOR_READY:
      light_sensor_ready = true;
      break;

    case TH_SENSOR_READY:
      th_sensor_ready = true;
      break;

    default:
      break;
  }
}

/* Check if sensor data ready to read */
bool sensor_ready(sensor_t sensor_id)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR: return bat_sensor_ready;
    case TEMPERATURE_SENSOR: return th_sensor_ready;
    case HUMIDITY_SENSOR: return th_sensor_ready;
    case SALINITY_SENSOR: return ec_sensor_ready;
    case SOIL_SENSOR: return soil_sensor_ready;
    case LIGHT_SENSOR: return light_sensor_ready;
    default: return false;
  }
}

bool all_sensors_ready()
{
  return soil_sensor_ready && ec_sensor_ready && light_sensor_ready && th_sensor_ready;
}

void reset_sensor(sensor_t sensor_id)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR:
      bat_sensor_ready = false;
      break;
    case TEMPERATURE_SENSOR:
      th_sensor_ready = false;
      break;
    case HUMIDITY_SENSOR:
      th_sensor_ready = false;
      break;
    case SALINITY_SENSOR:
      ec_sensor_ready = false;
      break;
    case SOIL_SENSOR:
      soil_sensor_ready = false;
      break;
    case LIGHT_SENSOR:
      light_sensor_ready = false;
      break;
    default:
      break;
  }
}

void reset_all_sensors()
{
  soil_sensor_ready = false;
  ec_sensor_ready = false;
  light_sensor_ready = false;
  th_sensor_ready = false;
}

bool is_any_sensor_notify()
{
  if (battery_sensor_notify || temperature_sensor_notify || humidity_sensor_notify ||
  salinity_sensor_notify || soil_sensor_notify || light_sensor_notify)
    return true;
  else
    return false;
}

void set_sensor_notify(sensor_t sensor_id, bool notify)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR:
      battery_sensor_notify = notify;
      break;
    case TEMPERATURE_SENSOR:
      temperature_sensor_notify = notify;
      break;
    case HUMIDITY_SENSOR:
      humidity_sensor_notify = notify;
      break;
    case SALINITY_SENSOR:
      salinity_sensor_notify = notify;
      break;
    case SOIL_SENSOR:
      soil_sensor_notify = notify;
      break;
    case LIGHT_SENSOR:
      light_sensor_notify = notify;
      break;
  }
}