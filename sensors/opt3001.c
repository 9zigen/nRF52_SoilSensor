/***
** Created by Aleksey Volkov on 2019-03-21.
***/

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <sensors/include/sensors.h>
#include <include/i2c.h>
#include "nrfx_timer.h"

#include "app_timer.h"
#include "timers.h"

#include "opt3001.h"

#define SENSOR_DELAY_READ                             APP_TIMER_TICKS(800) /* 800 msec */

APP_TIMER_DEF(m_ready_light_timer_id);
uint32_t opt_visible_lux = 0;

static uint16_t read_register(uint8_t reg_address)
{
  uint8_t data[2];
  i2c_write_byte(OPT3001_I2C_ADDRESS, reg_address, true);
  i2c_read_bytes(OPT3001_I2C_ADDRESS, data, 2);

  /* MSB First */
  return (data[0] << 8) | data[1];
}

static void write_register(uint8_t reg_address, uint16_t val)
{
  uint8_t data[3];
  data[0] = reg_address;
  data[1] = val >> 8; /* MSB */
  data[2] = val;      /* LSB */

  i2c_write_bytes(OPT3001_I2C_ADDRESS, data, 3, false);
}

static void opt3001_get_id()
{
  uint16_t manufacturer_id = read_register(MANUFACTURER_ID);
  uint16_t device_id       = read_register(DEVICE_ID);

  NRF_LOG_INFO("OPT3001 MANUFACTURER_ID: 0x%x", manufacturer_id);
  NRF_LOG_INFO("OPT3001 DEVICE_ID      : 0x%x", device_id);
  NRF_LOG_FLUSH();
}

/* lux = 0.01 × (2E[3:0]) × R[11:0] */
static uint32_t calc_lux(uint16_t raw)
{
  uint16_t    exponent = 0;
  uint32_t    result = 0;

  /* extract result & exponent data from raw readings */
  result = raw & 0x0FFF;
  exponent = (uint16_t)(raw>>12) & 0x000F;

  /* Convert to Lux */
  double lux = 0.01 * (1 << exponent) * result;
  
  return (uint32_t)lux;
}

/* Slow read sensors timer handler */
static void read_sensor_timer_handler(void *p_context)
{
  uint16_t raw_result = read_register(OPT3001_RESULT_REG);
  uint32_t lux = calc_lux(raw_result);
  NRF_LOG_INFO("OPT3001 RAW: 0x%x", raw_result);
  NRF_LOG_INFO("OPT3001 LUX: %d", lux);

  opt_visible_lux = lux;

  /* Disable twi interface */
  twi_disable();
  set_sensor_ready(LIGHT_SENSOR_READY);
}

void opt3001_init()
{
  /* Print device info */
  opt3001_get_id();

  NRF_LOG_INFO("OPT3001 GET CONFIG     : 0x%x", read_register(OPT3001_CONFIG_REG));

  /* Create timer to delay read after start conversion */
  ret_code_t err_code = app_timer_create(&m_ready_light_timer_id, APP_TIMER_MODE_SINGLE_SHOT, read_sensor_timer_handler);
  APP_ERROR_CHECK(err_code);
}

void opt3001_read_sensor()
{
  twi_enable();

  /* Configure device and start Single conversion
   * Set: One Shot mode and 800-ms conversion time */
  write_register(OPT3001_CONFIG_REG, (DEFAULT_CONFIG | OPMODE_SINGLE_SHOT | CONVERSION_800MS));

  ret_code_t err_code;
  err_code = app_timer_start(m_ready_light_timer_id, SENSOR_DELAY_READ, NULL);
  APP_ERROR_CHECK(err_code);
}

uint32_t opt3001_get_visible_lux()
{
  return opt_visible_lux;
}