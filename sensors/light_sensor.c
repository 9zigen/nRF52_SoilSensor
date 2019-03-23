/***
** Created by Aleksey Volkov on 2019-03-21.
***/


#include <nrf_log.h>
#include "opt3001.h"
#include "isl29035.h"
#include "light_sensor.h"

/* make sdk_config to select sensor */
const light_sensor_chip_t sensor_chip = LIGHT_SENSOR_CHIP;

void init_light_sensor()
{
  switch (sensor_chip)
  {
    case ISL29035:
      /* Init ISL29035 Ambient Light Sensor */
      isl29035_init();
      break;
    case OPT3001:
      /* Init OPT3001 Ambient Light Sensor */
      opt3001_init();
      break;
    default:
      break;
  }
}

/* Setup sensor and start conversion */
void read_light_sensor()
{
  switch (sensor_chip)
  {
    case ISL29035:
      isl29035_read_sensor();
      break;
    case OPT3001:
      opt3001_read_sensor();
      break;
    default:
      break;
  }
}

/* Return last Visible lux measure */
uint32_t get_visible_lux()
{
//  NRF_LOG_INFO("Light Sensor: %d", sensor_chip);

  switch (sensor_chip)
  {
    case ISL29035:
      return isl29035_get_visible_lux();
    case OPT3001:
      return opt3001_get_visible_lux();
    default:
      return 0;
  }
}
