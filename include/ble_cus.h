#ifndef BLE_CUS_H__
#define BLE_CUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble_srv_common.h"
#include "ble_gatts.h"
#include "ble_types.h"

/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_DEF(_name)                                                                          \
static ble_cus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_on_ble_evt, &_name)



// CUSTOM_SERVICE_UUID_BASE f364adc9-b000-4042-ba50-05ca45bf8abc

#define CUSTOM_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, \
                                          0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}

#define CUSTOM_SERVICE_UUID               0x1400
#define STATUS_CHAR_UUID                  0x1401
#define TEMPERATURE_CHAR_UUID             0x1402
#define HUMIDITY_CHAR_UUID                0x1403
#define SALINITY_CHAR_UUID                0x1404
#define SOIL_CHAR_UUID                    0x1405
#define LIGHT_CHAR_UUID                   0x1406
																					
/* Custom Service event type. */
typedef enum
{
  BLE_CUS_EVT_TEMPERATURE_NOTIFICATION_ENABLED,                  /**< Tempereture value notification enabled event. */
  BLE_CUS_EVT_TEMPERATURE_NOTIFICATION_DISABLED,                 /**< Tempereture value notification disabled event. */
  BLE_CUS_EVT_HUMIDITY_NOTIFICATION_ENABLED,                     /**< Humidity value notification enabled event. */
  BLE_CUS_EVT_HUMIDITY_NOTIFICATION_DISABLED,                    /**< Humidity value notification disabled event. */
  BLE_CUS_EVT_SALINITY_NOTIFICATION_ENABLED,                     /**< Salinity value notification enabled event. */
  BLE_CUS_EVT_SALINITY_NOTIFICATION_DISABLED,                    /**< Salinity value notification disabled event. */
  BLE_CUS_EVT_SOIL_NOTIFICATION_ENABLED,                         /**< Tempereture value notification enabled event. */
  BLE_CUS_EVT_SOIL_NOTIFICATION_DISABLED,                        /**< Tempereture value notification disabled event. */
  BLE_CUS_EVT_LIGHT_NOTIFICATION_ENABLED,                         /**< Tempereture value notification enabled event. */
  BLE_CUS_EVT_LIGHT_NOTIFICATION_DISABLED,                        /**< Tempereture value notification disabled event. */
  BLE_CUS_EVT_DISCONNECTED,
  BLE_CUS_EVT_CONNECTED,
  BLE_CUS_EVT_CALIBRATE_SOIL_DRY,
  BLE_CUS_EVT_CALIBRATE_SOIL_NORMAL,
  BLE_CUS_EVT_CALIBRATE_SOIL_WET,
  BLE_CUS_EVT_CALIBRATE_CONDUCTIVITY_1000_US,

} ble_cus_evt_type_t;

/* Custom Service event. */
typedef struct
{
  ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_evt_t;

/* Forward declaration of the ble_cus_t type. */
typedef struct ble_cus_s ble_cus_t;


/* Custom Service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_bas, ble_cus_evt_t * p_evt);

/* Custom Service init structure. This contains all options and data needed for initialization of the service.*/
typedef struct
{
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  int32_t                       initial_status_value;           /**< Initial Temperature value */
  int32_t                       initial_temperature_value;      /**< Initial Temperature value */
  int32_t                       initial_humidity_value;         /**< Initial Humidity value */
  int32_t                       initial_salinity_value;         /**< Initial Salinity value */
  int32_t                       initial_soil_value;             /**< Initial Soil value */
  int32_t                       initial_light_value;            /**< Initial Light value */
  ble_srv_cccd_security_mode_t  status_char_attr_md;            /**< Initial security level for Custom characteristics attribute */
  ble_srv_cccd_security_mode_t  temperature_char_attr_md;       /**< Initial security level for Custom characteristics attribute */
  ble_srv_cccd_security_mode_t  humidity_char_attr_md;          /**< Initial security level for Custom characteristics attribute */
  ble_srv_cccd_security_mode_t  salinity_char_attr_md;          /**< Initial security level for Custom characteristics attribute */
  ble_srv_cccd_security_mode_t  soil_char_attr_md;              /**< Initial security level for Custom characteristics attribute */
  ble_srv_cccd_security_mode_t  light_char_attr_md;             /**< Initial security level for Custom characteristics attribute */

} ble_cus_init_t;

/* Custom Service structure. This contains various status information for the service. */
struct ble_cus_s
{
  ble_cus_evt_handler_t         evt_handler;                /**< Event handler to be called for handling events in the Custom Service. */
  uint16_t                      service_handle;             /**< Handle of Custom Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t      status_handles;             /**< Handles related to the Status Value characteristic. */
  ble_gatts_char_handles_t      temperature_handles;        /**< Handles related to the Temperature Value characteristic. */
  ble_gatts_char_handles_t      humidity_handles;           /**< Handles related to the Humidity Value characteristic. */
  ble_gatts_char_handles_t      salinity_handles;           /**< Handles related to the Salinity Value characteristic. */
  ble_gatts_char_handles_t      soil_handles;               /**< Handles related to the Soil Value characteristic. */
  ble_gatts_char_handles_t      light_handles;              /**< Handles related to the Light Value characteristic. */
  uint16_t                      conn_handle;                /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
  uint8_t                       uuid_type;
};

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_service_init(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_cus      Custom Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_bas          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_cus_temperature_update(ble_cus_t *p_cus, uint16_t new_value);
uint32_t ble_cus_humidity_update(ble_cus_t *p_cus, uint16_t new_value);
uint32_t ble_cus_salinity_update(ble_cus_t *p_cus, uint16_t new_value);
uint32_t ble_cus_soil_update(ble_cus_t *p_cus, uint16_t new_value);
uint32_t ble_cus_light_update(ble_cus_t *p_cus, uint32_t new_value);
#endif // BLE_CUS_H__
