/***
** Created by Aleksey Volkov on 2019-01-20.
***/

#ifndef SOILSENSOR_I2C_H
#define SOILSENSOR_I2C_H

#include <boards.h>

void twi_init (void);
void twi_uninit (void);

void twi_enable (void);
void twi_disable (void);

void scan_i2c(void);

void i2c_write_byte(uint8_t dev_address, uint8_t data, bool no_stop);
void i2c_write_bytes(uint8_t dev_address, uint8_t *p_data, uint8_t length, bool no_stop);

uint8_t i2c_read_byte(uint8_t dev_address, uint8_t data_address);
void i2c_read_bytes(uint8_t dev_address, uint8_t * p_data, uint8_t length);

#endif //SOILSENSOR_I2C_H
