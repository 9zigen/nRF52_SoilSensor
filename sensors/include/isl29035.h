/***
** Created by Aleksey Volkov on 2019-01-20.
***/

#ifndef SOILSENSOR_ISL29035_H
#define SOILSENSOR_ISL29035_H

#include <boards.h>

#define ISL29035_I2C_ADDRESS    0x44   //the 7bits i2c address

#define COMMAND_I               0x00
#define COMMAND_II              0x01
#define DATA_L                  0x02
#define DATA_H                  0x03
#define INT_LT_L                0x04
#define INT_LT_H                0x05
#define INT_HT_L                0x06
#define INT_HT_H                0x07
#define CHIP_ID                 0x0f

#define OPMODE_POWER_DOWN       0x00
#define OPMODE_ALS_ONCE         0x20
#define OPMODE_IR_ONCE          0x40
#define OPMODE_ALS_CONTI        0xA0
#define OPMODE_IR_CONTI         0xC0

#define FULL_SCALE_LUX_RANGE0   0x00       //1000 Lux
#define FULL_SCALE_LUX_RANGE1   0x01       //4000 Lux
#define FULL_SCALE_LUX_RANGE2   0x02       //16000 Lux
#define FULL_SCALE_LUX_RANGE3   0x03       //64000 Lux

#define ADC_RESOLUTION_16BIT    0x00
#define ADC_RESOLUTION_12BIT    0x01
#define ADC_RESOLUTION_8BIT     0x02
#define ADC_RESOLUTION_4BIT     0x03

#define DEFAULT_LUX_RANGE_INDEX 1       //should be [0,3]

#define INTEGRATION_TIME3       0.0256  //ms, this also configure the ADC to 4bits
#define INTEGRATION_TIME2       0.41  //ms, this also configure the ADC to 8bits
#define INTEGRATION_TIME1       6.5  //ms, this also configure the ADC to 12bits200
#define INTEGRATION_TIME0       105  //ms, this also configure the ADC to 16bits
#define DEFAULT_INTEGRATION_TIME_INDEX 1  //should be [0,3]

typedef struct
{
  uint32_t ranges[4];
  uint32_t resolution[4];
  uint32_t adc_count_max[4];

} isl29035_config_t;

typedef struct {
  uint32_t visible_lux;
  uint32_t ir_lux;
  int32_t ev;
  void (*next_step)(void);

} isl29035_instance_t;

void isl29035_init();
void isl29035_deinit();

uint8_t read_register(uint8_t device_address, uint8_t reg_address);
void write_register(uint8_t device_address, uint8_t reg_address, uint8_t val);
uint16_t read_data();

void isl29035_read_sensor();
void isl29035_measure(uint8_t op_mode);

void isl29035_request_visible_lux();
void isl29035_read_visible_lux();

void isl29035_request_ir_lux();
void isl29035_read_ir_lux();

int32_t isl29035_get_ev();

uint32_t isl29035_get_visible_lux();
uint32_t isl29035_get_ir_lux();

#endif //SOILSENSOR_ISL29035_H
