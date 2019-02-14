/***
** Created by Aleksey Volkov on 2019-01-20.
***/

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <sensors/include/sensors.h>
#include "nrfx_timer.h"

#include "i2c.h"
#include "isl29035.h"

uint8_t full_scale_lux_range = FULL_SCALE_LUX_RANGE2;
uint8_t adc_resolution_bit_index  = ADC_RESOLUTION_16BIT;

isl29035_config_t isl29035_params = {
    .ranges = {
        1000,
        4000,
        16000,
        64000
    },
    .resolution = {
        16,
        12,
        8,
        4
    },
    .adc_count_max = {
        65536,
        4096,
        256,
        16
    }
};

/* Config, communication stage */
isl29035_instance_t isl29035 = {
    .visible_lux  = 0,
    .ir_lux       = 0,
    .ev           = 0,
    .next_step    = 0,
};

static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(3);

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
  if (event_type == NRF_TIMER_EVENT_COMPARE0)
  {
    if (isl29035.next_step)
    {
      isl29035.next_step();
    }
  }
}

/* Delay use hardware timer */
static void isl2935_delay(uint32_t ms)
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

static void timer_init()
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
  APP_ERROR_CHECK(err_code);
}

uint8_t read_register(uint8_t device_address, uint8_t reg_address)
{
  uint8_t value;

  value = i2c_read_byte(device_address, reg_address);
  return value;
}

void write_register(uint8_t device_address, uint8_t reg_address, uint8_t val)
{
  uint8_t data[2];
  data[0] = reg_address;
  data[1] = val;
  i2c_write_bytes(device_address, data, 2, true);
}

uint16_t read_data()
{
  uint8_t data[2];
  i2c_write_byte(ISL29035_I2C_ADDRESS, DATA_L, true);
  i2c_read_bytes(ISL29035_I2C_ADDRESS, data, 2);

  return (data[1] << 8) | data[0];
}

void isl29035_init()
{
  uint8_t reg = read_register(ISL29035_I2C_ADDRESS, CHIP_ID);
  NRF_LOG_INFO("ISL29035 CHIP_ID: 0x%x", reg);
  NRF_LOG_FLUSH();
  uint8_t chip_id = (reg >> 3) & 0x7;
  if (chip_id != 0x5)
  {
    NRF_LOG_INFO("ISL29035 CHIP_ID: FAIL");
    NRF_LOG_FLUSH();
    return;
  }

  //clear the BOUT bit
  write_register(ISL29035_I2C_ADDRESS, CHIP_ID, reg & 0x7f);

  //ensure the chip is under stop mode
  write_register(ISL29035_I2C_ADDRESS, COMMAND_I, 0);

  //set the default full scale lux range, and the integration time
  write_register(ISL29035_I2C_ADDRESS, COMMAND_II, full_scale_lux_range | (adc_resolution_bit_index << 2));
}

void isl29035_deinit() {
  write_register(ISL29035_I2C_ADDRESS, COMMAND_I, 0);
  nrfx_timer_uninit(&m_timer);
}

void isl29035_read_sensor()
{
  timer_init();
  twi_enable();

  /* read visible lux data after delay 105 ms */
  isl29035.next_step = &isl29035_read_visible_lux;
  isl29035_request_visible_lux();

  /* setup m_timer for compare event every 105ms integration time - 12bit sensor resolution */
  isl2935_delay(105);
}

void isl29035_measure(uint8_t op_mode)
{
  //start
  write_register(ISL29035_I2C_ADDRESS, COMMAND_I, op_mode);
}

void isl29035_request_visible_lux()
{
  isl29035_measure(OPMODE_ALS_ONCE);
}

void isl29035_request_ir_lux()
{
  isl29035_measure(OPMODE_IR_ONCE);
}

void isl29035_read_visible_lux() {
  uint32_t data = 0;
  data = read_data();
  isl29035.visible_lux = isl29035_params.ranges[full_scale_lux_range] *
      data / isl29035_params.adc_count_max[adc_resolution_bit_index];
  NRF_LOG_INFO("ISL29035 Visible Lux %d", isl29035.visible_lux);

  isl29035.next_step = &isl29035_read_ir_lux;
  isl29035_request_ir_lux();

  /* setup m_timer for compare event every 105ms integration time - 12bit sensor resolution */
  isl2935_delay(105);
}

void isl29035_read_ir_lux()
{
  uint32_t data = read_data();
  isl29035.ir_lux = isl29035_params.ranges[full_scale_lux_range] *
      data / isl29035_params.adc_count_max[adc_resolution_bit_index];

  /* calc EV */
  isl29035.ev = isl29035_get_ev();
  NRF_LOG_INFO("ISL29035 IR Lux %d", isl29035.ir_lux);
  NRF_LOG_INFO("ISL29035 EV %d", isl29035.ev);
  // power down sensor
  write_register(ISL29035_I2C_ADDRESS, COMMAND_I, 0);

  /* Sleep sensor an deinit timer */
  isl29035_deinit();

  /* Disable twi interface */
  twi_disable();

  set_sensor_ready(LIGHT_SENSOR_READY);
}

/* Return EV calculation by last Visible and IR lux measure */
int32_t isl29035_get_ev()
{
  float k = 0.82;
  float beta = -11292.86f;
  if(adc_resolution_bit_index > ADC_RESOLUTION_12BIT) beta = 2137.14f;
  return (int32_t)(k*isl29035.visible_lux + isl29035.ir_lux/beta);
}

/* Return last Visible lux measure */
uint32_t isl29035_get_visible_lux()
{
  return isl29035.visible_lux;
}

/* Return last IR lux measure */
uint32_t isl29035_get_ir_lux()
{
  return isl29035.ir_lux;
}