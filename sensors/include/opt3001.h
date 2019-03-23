/***
** Created by Aleksey Volkov on 2019-03-21.
***/

#ifndef SOILSENSOR_OPT3001_H
#define SOILSENSOR_OPT3001_H

#include <boards.h>

#define OPT3001_I2C_ADDRESS     0x44   //the 7bits i2c address

#define OPT3001_RESULT_REG                  0x00
#define OPT3001_CONFIG_REG                  0x01
#define INT_LL                  0x02
#define INT_HL                  0x03
#define MANUFACTURER_ID         0x7E
#define DEVICE_ID               0x7F

#define CONVERSION_800MS        0x800 /* Set: 800 msec, Clear: 100 msec */
#define OPMODE_SHUTDOWN         0x00
#define OPMODE_SINGLE_SHOT      0x200
#define OPMODE_CONTINUOUS       0x400

#define DEFAULT_CONFIG          0xC810


void opt3001_init();
uint32_t opt3001_get_visible_lux();
void opt3001_read_sensor();

#endif //SOILSENSOR_OPT3001_H
