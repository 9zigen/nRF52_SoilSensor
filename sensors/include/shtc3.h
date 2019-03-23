/***
** Created by Aleksey Volkov on 2019-01-20.
***/

#ifndef SOILSENSOR_SHTC3_H
#define SOILSENSOR_SHTC3_H

#include <boards.h>

#define SHTC3_I2C_ADDRESS       0x70   //the 7bits i2c address

#define SLEEP                   0xB098
#define WAKEUP                  0x3517
#define RESET                   0x805D
#define READ_T_FIRST_NORMAL     0x7866
#define READ_RH_FIRST_NORMAL    0x58E0
#define READ_T_FIRST_LP         0x609C
#define READ_RH_FIRST_LP        0x401A
#define GET_CHIP_ID             0xEFC8

#define CRC_POLYNOMINAL         0x31 //Polynomial (x8 + x5 + x4 + 1)

/* Measurement duration
 * Normal Mode 10.8 - 12.1 ms
 * Low Power Mode 0.7 - 0.8 ms
 *
 * Wakeup time 240us
*/

typedef enum {
  SHTC_NORMAL, SHTC_LOWPOWER
} shtc3_mode_t;

typedef struct {
  int16_t temperature;
  uint16_t humidity;
  shtc3_mode_t mode;
  void (*next_step)(void);

} shtc3_instance_t;

void init_shtc3();
void shtc3_write_command(uint16_t command);
void shtc3_measure();
void shtc3_read();
void shtc3_sleep();
void shtc3_reset();
void shtc3_wakeup();
void shtc3_process();
void shtc_set_mode(shtc3_mode_t mode);

int16_t shtc_get_temperature();
uint16_t shtc_get_humidity();

#endif //SOILSENSOR_SHTC3_H
