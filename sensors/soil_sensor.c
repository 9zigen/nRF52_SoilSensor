/***
** Created by Aleksey Volkov on 15/11/2018.
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
#include "nrfx_gpiote.h"

#include "ble_cus.h"
#include "ble_services.h"
#include "pwm.h"
#include "timers.h"

#include "soil_sensor.h"
#include "sensors.h"

/* Macro to convert the result of ADC conversion in millivolts. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_MV) / ADC_RES_12BIT) * ADC_COMPENSATION)

#define SOIL_PWM_PIN          NRF_GPIO_PIN_MAP(0,5)

#define SAMPLES_IN_BUFFER     10
#define ADC_COMPENSATION      6
#define ADC_RES_12BIT         4095
#define ADC_REF_VOLTAGE_MV    600

/* ADC */
static nrf_saadc_value_t      m_buffer_pool[2][SAMPLES_IN_BUFFER];
uint16_t soil_milli_volts     = 0;
uint8_t  isDry                = 0;

/* TIMER */
static const nrfx_timer_t     m_timer_1 = NRFX_TIMER_INSTANCE(1);
static const nrfx_timer_t     m_timer_2 = NRFX_TIMER_INSTANCE(2);

/* PPI */
static nrf_ppi_channel_t  m_ppi_channel_1; /* PWM 8mHz */
static nrf_ppi_channel_t  m_ppi_channel_2; /* SAADC Sample 10kHz */

static void timer_handler_1(nrf_timer_event_t event_type, void * p_context) {}
static void timer_handler_2(nrf_timer_event_t event_type, void * p_context) {}

static void saadc_sampling_event_init(void)
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg_1 = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg_1.frequency = NRF_TIMER_FREQ_16MHz;
  timer_cfg_1.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer_1, &timer_cfg_1, timer_handler_1);
  APP_ERROR_CHECK(err_code);

  nrfx_timer_config_t timer_cfg_2 = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg_2.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg_2.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer_2, &timer_cfg_2, timer_handler_2);
  APP_ERROR_CHECK(err_code);

  if (!nrfx_gpiote_is_init())
  {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }


  nrfx_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
  err_code = nrfx_gpiote_out_init(SOIL_PWM_PIN, &config);
  APP_ERROR_CHECK(err_code);

  /* setup m_timer for compare event every 10us */
  uint32_t cc_1 = 2; /* 8mHz */
  uint32_t cc_2 = nrfx_timer_us_to_ticks(&m_timer_2, 100); /* 100us - 10kHz */

  nrfx_timer_extended_compare(&m_timer_1, NRF_TIMER_CC_CHANNEL0, cc_1, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
  nrfx_timer_extended_compare(&m_timer_2, NRF_TIMER_CC_CHANNEL0, cc_2, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  /* get addresses for event and task short */
  uint32_t timer_compare_event_ch0_addr = nrfx_timer_event_address_get(&m_timer_1, NRF_TIMER_EVENT_COMPARE0);
  uint32_t gpiote_task_addr             = nrfx_gpiote_out_task_addr_get(SOIL_PWM_PIN);

  uint32_t timer_compare_event_ch1_addr = nrfx_timer_event_address_get(&m_timer_2, NRF_TIMER_EVENT_COMPARE0);
  uint32_t saadc_sample_task_addr       = nrfx_saadc_sample_task_get();

  /* setup ppi channel so that timer compare event is triggering sample task in GPIOTE */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel_1, timer_compare_event_ch0_addr, gpiote_task_addr);
  APP_ERROR_CHECK(err_code);

  /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_2);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel_2, timer_compare_event_ch1_addr, saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_task_enable(SOIL_PWM_PIN);
}

static void saadc_sampling_event_deinit(void)
{
  ret_code_t err_code;

  err_code = nrfx_ppi_channel_free(m_ppi_channel_1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel_2);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_uninit(SOIL_PWM_PIN);

  nrfx_timer_uninit(&m_timer_1);
  nrfx_timer_uninit(&m_timer_2);
}

static void saadc_sampling_event_enable(void)
{
  ret_code_t err_code;

  err_code = nrfx_ppi_channel_enable(m_ppi_channel_1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_enable(m_ppi_channel_2);
  APP_ERROR_CHECK(err_code);

  nrfx_timer_enable(&m_timer_1);
  nrfx_timer_enable(&m_timer_2);
}

static void saadc_sampling_event_disable(void)
{
  nrfx_gpiote_out_task_disable(SOIL_PWM_PIN);

  nrfx_timer_disable(&m_timer_1);
  nrfx_timer_disable(&m_timer_1);
}

static void soil_saadc_callback(nrfx_saadc_evt_t const * p_event)
{
  if (p_event->type == NRFX_SAADC_EVT_DONE)
  {
    ret_code_t err_code;

    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    int i;
    NRF_LOG_INFO("SOIL ADC event:");

    static uint64_t accumulated = 0;
    for (i = 0; i < SAMPLES_IN_BUFFER; i++)
    {
      accumulated += p_event->data.done.p_buffer[i];
    }

    accumulated = accumulated/SAMPLES_IN_BUFFER;
    soil_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(accumulated);
    NRF_LOG_INFO("SOIL RAW filtered %d", accumulated);
    NRF_LOG_INFO("SOIL milli volts  %d", soil_milli_volts);

    set_sensor_ready(SOIL_SENSOR_READY);
    soil_sensor_deinit();
  }
}

/* SAADC setup for EC calculation */
static void soil_saadc_init(void)
{
  nrfx_saadc_uninit();

  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

  /* Configure SAADC */
  nrfx_saadc_config_t saadc_config;
  saadc_config.low_power_mode = false;                                                   //Enable low power mode.
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_2X;                                    //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

  err_code = nrfx_saadc_init(&saadc_config, soil_saadc_callback);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);
}

void soil_sensor_init(void)
{
  soil_saadc_init();
  saadc_sampling_event_init();
  saadc_sampling_event_enable();
}

void soil_sensor_deinit(void)
{
  saadc_sampling_event_disable();
  saadc_sampling_event_deinit();

  nrf_gpio_cfg_input(SOIL_PWM_PIN, NRF_GPIO_PIN_NOPULL);

  nrfx_saadc_uninit();
  NVIC_ClearPendingIRQ(SAADC_IRQn);
}

void read_soil()
{
  soil_sensor_init();
}

uint16_t get_soil_level(void)
{
  NRF_LOG_INFO("Soil MV %d", soil_milli_volts);
  return soil_milli_volts;
}