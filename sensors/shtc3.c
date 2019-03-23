/***
** Created by Aleksey Volkov on 2019-01-20.
***/

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_delay.h>
#include <sensors/include/sensors.h>
#include "nrfx_timer.h"

#include "i2c.h"
#include "shtc3.h"

static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(4);

/* Config, communication stage */
shtc3_instance_t shtc3 = {
    /* x100 values */
    .temperature  = 0,
    .humidity     = 0,
    .mode         = SHTC_NORMAL,
    .next_step    = 0
};

static float shtc3_calc_temperature(uint16_t raw) {
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / 2^16
  return 175 * (float)raw / 65536.0f - 45.0f;
}

static float shtc3_calc_humidity(uint16_t raw) {
  // calculate relative humidity [%RH]
  // RH = rawValue / 2^16 * 100
  return 100 * (float)raw / 65536.0f;
}

static bool CRC8(int8_t data[], uint8_t n_bytes, uint8_t checksum)
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t counter;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(counter = 0; counter < n_bytes; counter++) {
    crc ^= (data[counter]);
    for(bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ (uint8_t)CRC_POLYNOMINAL;
      } else {
        crc = (crc << 1);
      }
    }
  }

  // verify checksum
  if(crc != checksum) {
    return false;
  } else {
    return true;
  }
}

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
  if (event_type == NRF_TIMER_EVENT_COMPARE0)
  {
    if (shtc3.next_step)
    {
      shtc3.next_step();
    }
  }
}

/* Delay use hardware timer */
static void shtc3_delay(uint32_t ms)
{
  nrfx_timer_clear(&m_timer);
  uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, ms);
  nrfx_timer_extended_compare(&m_timer,
                              NRF_TIMER_CC_CHANNEL0,
                              ticks,
                              NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                              true);

  nrfx_timer_enable(&m_timer);
}

void timer_init(void)
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
  APP_ERROR_CHECK(err_code);
}

void shtc3_write_command(uint16_t command)
{
  uint8_t data[2];
  data[0] = (uint8_t)(command >> 8);
  data[1] = (uint8_t)command;
  i2c_write_bytes(SHTC3_I2C_ADDRESS, data, 2, false);

}

/* Set sensor measure mode */
void shtc_set_mode(shtc3_mode_t mode)
{
  shtc3.mode = mode;
}

void init_shtc3()
{
  /* wake up sensor */
  shtc3_wakeup();

  /* wait wake up */
  nrf_delay_ms(1);

  shtc3_write_command(GET_CHIP_ID);

  uint8_t data[3];
  i2c_read_bytes(SHTC3_I2C_ADDRESS, data, 3);

  uint8_t checksum = data[2];
  if (!CRC8((int8_t*)data, 2, checksum))
    return;

  uint16_t chip_id = (data[0] << 8) | data[1];
  NRF_LOG_INFO("SHTC3 CHIP ID %d", chip_id);

  shtc3_sleep();

}

void shtc3_deinit()
{
  nrfx_timer_uninit(&m_timer);

  /* de initialize twi interface for low power consumption */
  twi_disable();
}

void shtc3_reset()
{
  shtc3_write_command(RESET);
}

void shtc3_sleep()
{
  shtc3_write_command(SLEEP);
}

void shtc3_process()
{
  timer_init();
  twi_enable();

  /* schedule measure */
  shtc3.next_step = &shtc3_measure;

  /* wake up sensor */
  shtc3_wakeup();

  /* setup m_timer for compare event on 1ms, sensor Wake Up delay */
  shtc3_delay(1);
}

void shtc3_wakeup()
{
  shtc3_write_command(WAKEUP);
}

void shtc3_measure()
{
  uint16_t delay = 2;
  switch (shtc3.mode)
  {
    case SHTC_NORMAL:
      shtc3_write_command(READ_T_FIRST_NORMAL);
      delay = 13; /* 13 ms delay */
      break;
    case SHTC_LOWPOWER:
      shtc3_write_command(READ_T_FIRST_LP);
      delay = 1; /* 1 ms delay */
      break;
    default:
      break;
  }

  /* schedule read sensor data */
  shtc3.next_step = &shtc3_read;
  /* setup m_timer for compare event to 1ms or 13ms,
   * measurement duration in low power/normal mode */
  shtc3_delay(delay);

}

/* I2C Read sensor result of measurement, then send command to go_to_sleep and release timer */
void shtc3_read()
{
  uint8_t data[3];
  /* Read first 3 byte */
  i2c_read_bytes(SHTC3_I2C_ADDRESS, data, 3);
  uint8_t checksum = data[2];
  if (!CRC8((int8_t*)data, 2, checksum))
    return;

  uint16_t raw_temperature = (data[0] << 8) | data[1];
  shtc3.temperature = (int16_t)(shtc3_calc_temperature(raw_temperature) * 100);
  NRF_LOG_INFO("SHTC3 temperature x100 %d", shtc3.temperature);

  /* Read second 3 byte */
  i2c_read_bytes(SHTC3_I2C_ADDRESS, data, 3);
  checksum = data[2];
  if (!CRC8((int8_t*)data, 2, checksum))
    return;

  uint16_t raw_humidity = (data[0] << 8) | data[1];
  shtc3.humidity = (uint16_t)(shtc3_calc_humidity(raw_humidity) * 100);
  NRF_LOG_INFO("SHTC3 humidity x100 %d", shtc3.humidity);

  shtc3_sleep();
  shtc3_deinit();

  set_sensor_ready(TH_SENSOR_READY);
}

/* Return last sensor temperature x100, tu use simple divide by 100 */
int16_t shtc_get_temperature()
{
  return shtc3.temperature;
}

/* Return last sensor humidity x100, tu use simple divide by 100 */
uint16_t shtc_get_humidity()
{
  return shtc3.humidity;
}