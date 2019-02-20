//
// Created by Алексей on 11/10/2018.
//

#include "pwm.h"

#include <stdio.h>
#include <string.h>
#include "nrfx_pwm.h"
#include "nrfx_timer.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrfx_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "custom_board.h"
#include "pwm.h"

APP_TIMER_DEF(led_timer_id);

#define LED_TIMER_INTERVAL            APP_TIMER_TICKS(2000)       /* 1 sec */
#define TOP                           2500

static led_indication_t indicating_mode = LED_INDICATE_IDLE;
static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(1);

static nrf_pwm_values_individual_t seq0_values[] =
    {
        { 0, 0, 0,      0 },
        { 0, 0, 0x8000, 0 },
        { 0, 0, 0,      0 },
        { 0, 0, 0,      0 },
    };

static nrf_pwm_sequence_t seq = {
    .values.p_individual = seq0_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq0_values),
    .repeats         = 1,
    .end_delay       = 0
};

/* Led timer handler, restart pwm seq */
void led_timer_handler(void *p_context)
{
  UNUSED_PARAMETER(p_context);

  /* play sequence */
  (void)nrfx_pwm_simple_playback(&m_pwm0, &seq, 2, NRFX_PWM_FLAG_STOP);
}

void init_led_pwm(void)
{

  nrfx_pwm_config_t const config0 =
    {
      .output_pins =
        {
          LED_R | NRFX_PWM_PIN_INVERTED, // channel 0
          LED_G | NRFX_PWM_PIN_INVERTED, // channel 1
          LED_B | NRFX_PWM_PIN_INVERTED, // channel 2
          NRFX_PWM_PIN_NOT_USED  // channel 3
        },
      .irq_priority = APP_IRQ_PRIORITY_LOWEST,
      .base_clock   = NRF_PWM_CLK_250kHz,
      .count_mode   = NRF_PWM_MODE_UP,
      .top_value    = TOP,
      .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
      .step_mode    = NRF_PWM_STEP_AUTO
    };

  APP_ERROR_CHECK(nrfx_pwm_init(&m_pwm0, &config0, NULL));

  ret_code_t err_code;

  err_code = app_timer_create(&led_timer_id, APP_TIMER_MODE_REPEATED, led_timer_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_start(led_timer_id, LED_TIMER_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}

void led_indication_set(led_indication_t mode)
{
  indicating_mode = mode;

  uint8_t speed = 1;

  ret_code_t err_code;
  err_code = app_timer_stop(led_timer_id);
  APP_ERROR_CHECK(err_code);

  switch (mode) {
    case LED_INDICATE_ADVERTISING:
      speed = 2;
      break;
    case LED_INDICATE_ADVERTISING_SLOW:
      speed = 8;
      break;
    case LED_INDICATE_CONNECTED:
      speed = 1;
      break;
    case LED_INDICATE_IDLE:break;
    case LED_INDICATE_ERROR:break;
  }

  err_code = app_timer_start(led_timer_id, LED_TIMER_INTERVAL * speed, NULL);
  APP_ERROR_CHECK(err_code);
}

/* RGB color 0 - 32768, 16bit PWM Value */
void led_indication_set_color(uint16_t r, uint16_t g, uint16_t b)
{
  /* double blink indication 0100 -> 0100*/
  for (int i = 0; i < 4; ++i) {
    seq0_values[i].channel_0 = 0;
    seq0_values[i].channel_1 = 0;
    seq0_values[i].channel_2 = 0;
    seq0_values[i].channel_3 = 0;
  }

  seq0_values[1].channel_0 = r;
  seq0_values[1].channel_1 = g;
  seq0_values[1].channel_2 = b;
  seq0_values[1].channel_3 = 0;
}
