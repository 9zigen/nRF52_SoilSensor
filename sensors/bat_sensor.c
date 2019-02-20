/***
** Created by Aleksey Volkov on 14/11/2018.
***/

#include <libraries/timer/app_timer.h>
#include <boards.h>
#include <ble_bas.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_clock.h"
#include "nrfx_ppi.h"
#include <legacy/nrf_drv_ppi.h>
#include "nrfx_gpiote.h"
#include "nrfx_lpcomp.h"
#include "nrf_drv_saadc.h"

#include "ble_cus.h"
#include "ble_services.h"
#include "pwm.h"
#include "timers.h"

#include "bat_sensor.h"
#include "sensors.h"

/* Macro to convert the result of ADC conversion in millivolts. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_MV) / ADC_RES_12BIT) * ADC_COMPENSATION)

#define CALIBRATION_INTERVAL  5
#define SAMPLES_IN_BUFFER     2
#define ADC_COMPENSATION      6
#define ADC_RES_12BIT         4095
#define ADC_REF_VOLTAGE_MV    600
#define BAT_RESISTOR_DEVIDER  2

/* ADC */
static nrf_saadc_value_t      m_buffer_pool[2][SAMPLES_IN_BUFFER];
uint16_t bat_milli_volts    = 0;
uint8_t bat_capacity        = 0;

/* PPI */
static nrf_ppi_channel_t      m_ppi_channel;
static const nrfx_timer_t     m_timer = NRFX_TIMER_INSTANCE(1);

static void timer_handler(nrf_timer_event_t event_type, void * p_context) {}

static void go_to_sleep()
{

  /* wake up pin go to LOW when charge adapter attached */
  nrf_gpio_cfg_sense_set(CHARGE_SENSE_PIN, NRF_GPIO_PIN_SENSE_LOW);

  /* power off */
  ret_code_t err_code;
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

void in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (pin == CHARGE_SENSE_PIN)
  {
    bool charge = !nrf_gpio_pin_read(CHARGE_SENSE_PIN);
    if (charge)
      led_indication_set_color(0x8000, 0, 0);
    else
      led_indication_set_color(0, 0, 0x8000);

    NRF_LOG_INFO("CHARGE event: %d", charge);

  }
}

void charge_sense_init(void)
{
  ret_code_t err_code;

  if (!nrfx_gpiote_is_init())
  {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.hi_accuracy = false; /* Low Power */
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrfx_gpiote_in_init(CHARGE_SENSE_PIN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_in_event_enable(CHARGE_SENSE_PIN, true);

}


/* Standard discharge curve
 * http://www.farnell.com/datasheets/1475807.pdf
 * */
static uint8_t battery_level(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 4200)
  {
    battery_level = 100;
  }
  else if (mvolts > 3900)
  {
    battery_level = (uint8_t) (100 - (float)(4200 - mvolts) / 15);
  }
  else if (mvolts > 3850)
  {
    battery_level = (uint8_t) (80 - (float)(3900 - mvolts) / 2.45);
  }
  else if (mvolts > 3750)
  {
    battery_level = (uint8_t) (60 - (float)(3850 - mvolts) / 4.95);
  }
  else if (mvolts > 3650)
  {
    battery_level = (uint8_t) (40 - (float)(3750 - mvolts) / 4.95);
  }
  else if (mvolts > 3300)
  {
    battery_level = (uint8_t) (20 - (float)(3650 - mvolts) / 17.45);
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}

static void saadc_sampling_event_init(void)
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
  APP_ERROR_CHECK(err_code);

  /* setup m_timer for compare event every 1000us - 1 khz */
  uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 1000);
  nrfx_timer_extended_compare(&m_timer,
                              NRF_TIMER_CC_CHANNEL0,
                              ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                              true);
  nrfx_timer_enable(&m_timer);

  uint32_t timer_compare_event_addr = nrfx_timer_compare_event_address_get(&m_timer,
                                                                           NRF_TIMER_CC_CHANNEL0);
  uint32_t saadc_sample_task_addr   = nrfx_saadc_sample_task_get();

  /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel,
                                     timer_compare_event_addr,
                                     saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_enable(void)
{
  ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel);

  APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_disable(void)
{
  nrfx_timer_disable(&m_timer);
  nrfx_timer_uninit(&m_timer);

  ret_code_t err_code = nrfx_ppi_channel_disable(m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel);
  APP_ERROR_CHECK(err_code);
}


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
  if (p_event->type == NRFX_SAADC_EVT_DONE)
  {
    ret_code_t err_code;

    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    /* use only second value */
    bat_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[1]) * BAT_RESISTOR_DEVIDER;
    bat_capacity = battery_level(bat_milli_volts);

    NRF_LOG_INFO("BAT  mv: %d", bat_milli_volts);
    NRF_LOG_INFO("BAT cap: %d", bat_capacity);

    set_sensor_ready(BAT_SENSOR_READY);
    bat_sensor_deinit();

    if (bat_capacity == 0)
    {
      NRF_LOG_INFO("---> BAT LOW! ---> POWER DOWN!");
      go_to_sleep();
    }
  }
}

/* SAADC setup for BATTERY calculation */
static void bat_saadc_init() {
  nrfx_saadc_uninit();

  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
  channel_config.acq_time = NRF_SAADC_ACQTIME_10US;

  /* Configure SAADC */
  nrfx_saadc_config_t saadc_config;
  saadc_config.low_power_mode = true;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT; //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;  //Set oversample to 128x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 128 times.
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

  err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

//  nrfx_saadc_calibrate_offset()

}

void bat_sensor_init(void)
{
  nrf_gpio_cfg_output(BAT_ON);
  nrf_gpio_pin_set(BAT_ON);

  bat_saadc_init();
  saadc_sampling_event_init();
  saadc_sampling_event_enable();
}

void bat_sensor_deinit(void)
{
  saadc_sampling_event_disable();
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  nrfx_saadc_uninit();

  nrf_gpio_cfg_input(BAT_ON, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_pin_clear(BAT_ON);

}

/* get Battery voltage */
void read_battery_voltage()
{
  bat_sensor_init();
}

uint8_t get_battery_capacity()
{
  return bat_capacity;
}