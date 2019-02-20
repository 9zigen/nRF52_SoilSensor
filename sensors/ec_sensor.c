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
#include "nrfx_gpiote.h"

#include "ble_cus.h"
#include "ble_services.h"
#include "pwm.h"
#include "timers.h"

#include "shtc3.h"
#include "storage.h"
#include "ec_sensor.h"
#include "sensors.h"

/* Macro to convert the result of ADC conversion in millivolts. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_MV) / ADC_RES_12BIT) * ADC_COMPENSATION)

#define EC_1                  NRF_GPIO_PIN_MAP(0,4)
#define EC_2                  NRF_GPIO_PIN_MAP(0,8)

#define SAMPLES_IN_BUFFER     2
#define ADC_COMPENSATION      6
#define ADC_RES_12BIT         4095
#define ADC_REF_VOLTAGE_MV    600
#define TEMPERATURE_COMP      0.02f
#define ONE_POINT_CALIBRATION_SOLUTION_US 1000

/* 200mm (distance) / 2 * 3,14 * 1,25 * 0.4 + pi * 1,25^2 (effective area) = 20/8,05 = 2,48*/
#define PROBE_K               2.48

/* ADC */
static nrf_saadc_value_t      m_buffer_pool[2][SAMPLES_IN_BUFFER];
uint16_t ec_milli_volts     = 0;
uint16_t resistance         = 0;

/* PPI */
static nrf_ppi_channel_t  m_ppi_channel_ec1;    /* EC1 toggle */
static nrf_ppi_channel_t  m_ppi_channel_ec2;    /* EC2 toggle */
static nrf_ppi_channel_t  m_ppi_channel_saadc;  /* SAADC Sample Start */

static const nrfx_timer_t     m_timer = NRFX_TIMER_INSTANCE(1);

static void timer_handler(nrf_timer_event_t event_type, void * p_context) {}

static void saadc_sampling_event_init(void)
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
  APP_ERROR_CHECK(err_code);

  /* setup m_timer for compare event to 200us, start SAADC */
  uint32_t ticks_saadc = nrfx_timer_us_to_ticks(&m_timer, 200);
  nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks_saadc, true);

  /* setup m_timer for compare event to 400us, EC1 HIGHT, EC2 LOW */
  uint32_t ticks_gpio_reverse = nrfx_timer_us_to_ticks(&m_timer, 400);
  nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks_gpio_reverse, true);

  /* setup m_timer for compare event to 600us to leave reversed pin */
  uint32_t ticks_gpio_leave = nrfx_timer_us_to_ticks(&m_timer, 500);
  nrfx_timer_extended_compare(&m_timer,
                              NRF_TIMER_CC_CHANNEL2,
                              ticks_gpio_leave,
                              NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                              true);

  /* GPIOTE Init */
  if (!nrfx_gpiote_is_init())
  {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  /* GPIOTE Toggle task EC1, initial HIGH */
  nrfx_gpiote_out_config_t config_ec1 = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);
  err_code = nrfx_gpiote_out_init(EC_1, &config_ec1);
  APP_ERROR_CHECK(err_code);

  /* GPIOTE Toggle task EC2, initial LOW */
  nrfx_gpiote_out_config_t config_ec2 = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
  err_code = nrfx_gpiote_out_init(EC_2, &config_ec2);
  APP_ERROR_CHECK(err_code);

  /* SAADC
   * PPI Channel */
  uint32_t timer_compare_event_ch0_addr     = nrfx_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
  uint32_t saadc_sample_task_addr           = nrfx_saadc_sample_task_get();

  /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel_saadc, timer_compare_event_ch0_addr, saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);

  /* GPIOTE
   * PPI Channel EC1 */
  uint32_t timer_compare_event_ch1_addr     = nrfx_timer_event_address_get(&m_timer, NRF_TIMER_EVENT_COMPARE1);
  uint32_t ec1_gpiote_task_addr             = nrfx_gpiote_out_task_addr_get(EC_1);

  /* setup ppi channel so that timer compare event is triggering sample task in GPIOTE */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel_ec1, timer_compare_event_ch1_addr, ec1_gpiote_task_addr);
  APP_ERROR_CHECK(err_code);

  /* GPIOTE
   * PPI Channel EC2 */
  uint32_t ec2_gpiote_task_addr             = nrfx_gpiote_out_task_addr_get(EC_2);

  /* setup ppi channel so that timer compare event is triggering sample task in GPIOTE */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel_ec2, timer_compare_event_ch1_addr, ec2_gpiote_task_addr);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_task_enable(EC_1);
  nrfx_gpiote_out_task_enable(EC_2);

}

static void saadc_sampling_event_deinit(void)
{
  ret_code_t err_code;

  err_code = nrfx_ppi_channel_free(m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_uninit(EC_1);
  nrfx_gpiote_out_uninit(EC_2);

  nrfx_timer_uninit(&m_timer);
}

static void saadc_sampling_event_enable(void)
{
  ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_enable(m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_enable(m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);

  nrfx_timer_enable(&m_timer);
}

static void saadc_sampling_event_disable(void)
{
  ret_code_t err_code;

  err_code = nrfx_ppi_channel_disable(m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_disable(m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_disable(m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);
}


static void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
  if (p_event->type == NRFX_SAADC_EVT_DONE)
  {
    ret_code_t err_code;

    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    int i;
    for (i = 0; i < SAMPLES_IN_BUFFER; i++)
    {
      NRF_LOG_INFO("RAW %d", p_event->data.done.p_buffer[i]);
    }

    ec_milli_volts = (uint16_t)ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[0]);

    float volts = (float)ec_milli_volts/1000;
    NRF_LOG_INFO("EC volts " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(volts));
    if (ec_milli_volts != 0) {
      resistance = 1000 * (1 / ((3.30 / volts) - 1));
    }


    NRF_LOG_INFO("EC milli volts %d", ec_milli_volts);
    NRF_LOG_INFO("Resistance %d", resistance);
    set_sensor_ready(EC_SENSOR_READY);

    ec_sensor_deinit();
  }

}

/* SAADC setup for EC calculation */
static void ec_saadc_init(void)
{
  nrfx_saadc_uninit();

  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  channel_config.burst = NRF_SAADC_BURST_ENABLED;

  /* Configure SAADC */
  nrfx_saadc_config_t saadc_config;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;        //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=4096 (when input voltage is 3.6V for channel gain setting of 1/6).
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_16X;          //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

  err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);
}

void ec_sensor_init(void)
{
  ec_saadc_init();
  saadc_sampling_event_init();
  saadc_sampling_event_enable();
}

void ec_sensor_deinit(void)
{
  saadc_sampling_event_disable();
  saadc_sampling_event_deinit();

  nrf_gpio_cfg_input(EC_1, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(EC_2, NRF_GPIO_PIN_NOPULL);

  nrfx_saadc_uninit();
  NVIC_ClearPendingIRQ(SAADC_IRQn);
}

/* EC sensor ms calculation
 * cell constant K = distance between the electrodes [m] / effective area of the electrodes [m2])
 * effective area of the electrodes [m2] = 3.14*r^2+2*3.14*h
 *
 * resistance = Resistor * (K / ((inputV / outputV) - 1));
 * mS = ((100000 * K) / resistance);
 * */
void process_conductivity()
{
  ec_sensor_init();
}

/*
 * https://en.wikipedia.org/wiki/Conductivity_%28electrolytic%29
 * https://www.omega.co.uk/techref/ph-2.html
 * */
uint16_t get_raw_salinity() {
  static int32_t temperature;
  ret_code_t err_code;

  err_code = sd_temp_get(&temperature); /* in points of 0.25`C, need devise by 4 */
  APP_ERROR_CHECK(err_code);

  temperature = temperature / 4;

  float uS = 0.0f;
  if (resistance > 0)
  {
    uS = (float)(1000000 * PROBE_K) / resistance; /* 1 μS/cm = 100 μS/m;  10^6 μS/cm = 10^3 mS/cm = 1 S/cm */
  }
  uint16_t uS_compensated = uS + (uS * ((25 - temperature) * TEMPERATURE_COMP));
//
  NRF_LOG_INFO("Salinity   (uS) %d", uS);
  NRF_LOG_INFO("Salinity c (uS) %d", uS_compensated);
  NRF_LOG_INFO("Temperature (C) %d", temperature);

  return uS_compensated;
}

/* Return salinity in uS compensated by temp and corrected by one or two point calibration */
uint16_t get_conductivity()
{
  uint16_t raw_salinity = get_raw_salinity();
  uint16_t true_salinity = 0;

  storage_t cfg = {};
  get_storage(&cfg);

  if (cfg.ec_sensor_config.use_two_poit_calibration && cfg.ec_sensor_config.cal_hight && cfg.ec_sensor_config.cal_low)
  {
    uint16_t reference_range = (cfg.ec_sensor_config.ref_hight_us - cfg.ec_sensor_config.ref_low_us);
    uint16_t raw_range       = (cfg.ec_sensor_config.cal_hight - cfg.ec_sensor_config.cal_low);

    true_salinity = (((raw_salinity - cfg.ec_sensor_config.cal_low) * reference_range) / raw_range) + cfg.ec_sensor_config.ref_low_us;
  } else if (cfg.ec_sensor_config.use_one_poit_calibration && cfg.ec_sensor_config.cal_one_point) {
    true_salinity = (uint16_t)(raw_salinity * cfg.ec_sensor_config.cal_one_point);
  } else {
    true_salinity = raw_salinity;
  }

  return true_salinity;
}

/* 1000 uS calibration
 * 491 mg/L NaCl
 * */
void do_one_point_calibration()
{
  uint16_t current_uS = get_raw_salinity();
  storage_t cfg = {};
  get_storage(&cfg);

  static int32_t temperature;
  ret_code_t err_code;
  err_code = sd_temp_get(&temperature); /* in points of 0.25`C, need devise by 4 */
  APP_ERROR_CHECK(err_code);

  temperature = temperature / 4;

  float ref_compensated = ONE_POINT_CALIBRATION_SOLUTION_US + (ONE_POINT_CALIBRATION_SOLUTION_US * ((25 - temperature) * TEMPERATURE_COMP));
  cfg.ec_sensor_config.use_one_poit_calibration = true;
  cfg.ec_sensor_config.cal_one_point = (current_uS - ref_compensated) / current_uS;

  NRF_LOG_INFO("One Point EC calibration: CUR    (uS x 100) %d", (uint16_t)current_uS * 100);
  NRF_LOG_INFO("One Point EC calibration: REF    (uS x 100) %d", (uint16_t)ref_compensated * 100);
  NRF_LOG_INFO("One Point EC calibration: Offset (uS x 100) %d", (int16_t)cfg.ec_sensor_config.cal_one_point * 100);
}

/* 1000 uS calibration */
void do_two_point_low_calibration()
{
  uint16_t current_uS = get_raw_salinity();
  storage_t cfg = {};
  get_storage(&cfg);

  static int32_t temperature;
  ret_code_t err_code;
  err_code = sd_temp_get(&temperature); /* in points of 0.25`C, need devise by 4 */
  APP_ERROR_CHECK(err_code);

  temperature = temperature / 4;

  float ref_compensated = ONE_POINT_CALIBRATION_SOLUTION_US + (ONE_POINT_CALIBRATION_SOLUTION_US * ((25 - temperature) * TEMPERATURE_COMP));
  cfg.ec_sensor_config.use_one_poit_calibration = true;
  cfg.ec_sensor_config.cal_one_point = (current_uS - ref_compensated) / current_uS;

  NRF_LOG_INFO("One Point EC calibration: CUR    (uS x 100) %d", (uint16_t)current_uS * 100);
  NRF_LOG_INFO("One Point EC calibration: REF    (uS x 100) %d", (uint16_t)ref_compensated * 100);
  NRF_LOG_INFO("One Point EC calibration: Offset (uS x 100) %d", (int16_t)cfg.ec_sensor_config.cal_one_point * 100);
}

/* 2000 uS calibration */
void do_two_point_hight_calibration()
{
  uint16_t current_uS = get_raw_salinity();
  storage_t cfg = {};
  get_storage(&cfg);

  static int32_t temperature;
  ret_code_t err_code;
  err_code = sd_temp_get(&temperature); /* in points of 0.25`C, need devise by 4 */
  APP_ERROR_CHECK(err_code);

  temperature = temperature / 4;

  float ref_compensated = ONE_POINT_CALIBRATION_SOLUTION_US + (ONE_POINT_CALIBRATION_SOLUTION_US * ((25 - temperature) * TEMPERATURE_COMP));
  cfg.ec_sensor_config.use_one_poit_calibration = true;
  cfg.ec_sensor_config.cal_one_point = (current_uS - ref_compensated) / current_uS;

  NRF_LOG_INFO("One Point EC calibration: CUR    (uS x 100) %d", (uint16_t)current_uS * 100);
  NRF_LOG_INFO("One Point EC calibration: REF    (uS x 100) %d", (uint16_t)ref_compensated * 100);
  NRF_LOG_INFO("One Point EC calibration: Offset (uS x 100) %d", (int16_t)cfg.ec_sensor_config.cal_one_point * 100);
}