
#ifndef PROJECT_BLE_H
#define PROJECT_BLE_H

#include <sensors/include/sensors.h>

void init_ble();
void ble_stack_init(void);
void advertising_start();

void advertising_update();
void adv_manuf_data_update();

/* update characteristics */
void update_sensors_service(sensor_t sensor_id);


#endif //PROJECT_BLE_H