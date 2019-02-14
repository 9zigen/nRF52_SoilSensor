/***
** Created by Aleksey Volkov on 2019-01-20.
***/

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include "nrfx_twim.h"

#include "i2c.h"

#define TWI_INSTANCE_ID     0
/* Number of possible TWI addresses. */
#define TWI_ADDRESSES      127
/* TWI instance. */
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(TWI_INSTANCE_ID);

/* TWI initialization. */
void twi_init (void)
{
  ret_code_t err_code;

  const nrfx_twim_config_t twi_config = {
      .scl                = SCL_PIN,
      .sda                = SDA_PIN,
      .frequency          = NRF_TWIM_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .hold_bus_uninit    = false
  };

  err_code = nrfx_twim_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrfx_twim_enable(&m_twi);
}

/* TWI uninitialization. */
void twi_uninit (void)
{
  if (!nrfx_twim_is_busy(&m_twi))
  {
    nrfx_twim_disable(&m_twi);
    nrfx_twim_uninit(&m_twi);
  }
}

/* TWI Enable */
void twi_enable (void)
{
  nrfx_twim_enable(&m_twi);
}

/* TWI Disable for power save */
void twi_disable (void)
{
  nrfx_twim_disable(&m_twi);
}

void i2c_write_byte(uint8_t dev_address, uint8_t data, bool no_stop)
{
  ret_code_t err_code;

  err_code = nrfx_twim_tx(&m_twi, dev_address, &data, 1, no_stop);
  APP_ERROR_CHECK(err_code);
}

void i2c_write_bytes(uint8_t dev_address, uint8_t *p_data, uint8_t length, bool no_stop)
{
  ret_code_t err_code;
  err_code = nrfx_twim_tx(&m_twi, dev_address, p_data, length, no_stop);
  APP_ERROR_CHECK(err_code);
}

uint8_t i2c_read_byte(uint8_t dev_address, uint8_t data_address)
{
  ret_code_t err_code;

  err_code = nrfx_twim_tx(&m_twi, dev_address, &data_address, 1, true);
  APP_ERROR_CHECK(err_code);

  uint8_t data;
  err_code = nrfx_twim_rx(&m_twi, dev_address, &data, 1);
  APP_ERROR_CHECK(err_code);

  return data;
}

void i2c_read_bytes(uint8_t dev_address, uint8_t * p_data, uint8_t length)
{
  ret_code_t err_code;
  err_code = nrfx_twim_rx(&m_twi, dev_address, p_data, length);
  APP_ERROR_CHECK(err_code);
}

void scan_i2c(void)
{
  ret_code_t err_code;
  uint8_t address;
  uint8_t sample_data;
  bool detected_device = false;

  NRF_LOG_INFO("TWI scanner started.");
  NRF_LOG_FLUSH();

  for (address = 1; address <= TWI_ADDRESSES; address++)
  {
    err_code = nrfx_twim_rx(&m_twi, address, &sample_data, sizeof(sample_data));
    if (err_code == NRF_SUCCESS)
    {
      detected_device = true;
      NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
    }
    NRF_LOG_FLUSH();
  }

  if (!detected_device)
  {
    NRF_LOG_INFO("No device was found.");
    NRF_LOG_FLUSH();
  }
}