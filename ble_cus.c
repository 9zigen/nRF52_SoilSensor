#include "sdk_common.h"
#include "ble_cus.h"
#include <string.h>
#include <sensors/include/ec_sensor.h>
#include "ble_services.h"
#include "ble_srv_common.h"
#include "ble_gatts.h"
#include "ble_types.h"
#include "nrf_gpio.h"
#include "nrf_log.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
  ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  ble_cus_evt_t evt;

  // Status Value Characteristic Written to.
  if (p_evt_write->handle == p_cus->status_handles.value_handle)
  {
    // ToDO: Soil Level Setup; EC Dual Poin calibration
    if(p_evt_write->data != NULL)
    {
      NRF_LOG_INFO("New Value received");
      NRF_LOG_INFO("0x%X", p_evt_write->data[0]);
      if (*p_evt_write->data == 1) {
        evt.evt_type = BLE_CUS_EVT_CALIBRATE_SOIL_DRY;
        NRF_LOG_INFO("CUS EVT: BLE_CUS_EVT_CALIBRATE_SOIL_DRY\r\n");
      } else if (*p_evt_write->data == 2) {
        evt.evt_type = BLE_CUS_EVT_CALIBRATE_SOIL_NORMAL;
        NRF_LOG_INFO("CUS EVT: BLE_CUS_EVT_CALIBRATE_SOIL_NORMAL\r\n");
      } else if (*p_evt_write->data == 3) {
        evt.evt_type = BLE_CUS_EVT_CALIBRATE_SOIL_WET;
        NRF_LOG_INFO("CUS EVT: BLE_CUS_EVT_CALIBRATE_SOIL_WET\r\n");
      } else if (*p_evt_write->data == 4) {
        evt.evt_type = BLE_CUS_EVT_CALIBRATE_CONDUCTIVITY_1000_US;
        NRF_LOG_INFO("CUS EVT: BLE_CUS_EVT_CALIBRATE_CONDUCTIVITY_1000_US\r\n");
        do_one_point_calibration();
      }
      p_cus->evt_handler(p_cus, &evt);
    }

  }

  // Check if the Temperature value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->temperature_handles.cccd_handle)
      && (p_evt_write->len == 2)
     )
  {
    // CCCD written, call application event handler
    if (p_cus->evt_handler != NULL)
    {
      ble_cus_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
          evt.evt_type = BLE_CUS_EVT_TEMPERATURE_NOTIFICATION_ENABLED;
      }
      else
      {
          evt.evt_type = BLE_CUS_EVT_TEMPERATURE_NOTIFICATION_DISABLED;
      }
      // Call the application event handler.
      p_cus->evt_handler(p_cus, &evt);
    }
  }

  // Check if the Humidity value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->humidity_handles.cccd_handle)
      && (p_evt_write->len == 2)
      )
  {
    // CCCD written, call application event handler
    if (p_cus->evt_handler != NULL)
    {
      ble_cus_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
        evt.evt_type = BLE_CUS_EVT_HUMIDITY_NOTIFICATION_ENABLED;
      }
      else
      {
        evt.evt_type = BLE_CUS_EVT_HUMIDITY_NOTIFICATION_DISABLED;
      }
      // Call the application event handler.
      p_cus->evt_handler(p_cus, &evt);
    }
  }

  // Check if the Salinity value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->salinity_handles.cccd_handle)
      && (p_evt_write->len == 2)
      )
  {
    // CCCD written, call application event handler
    if (p_cus->evt_handler != NULL)
    {
      ble_cus_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
        evt.evt_type = BLE_CUS_EVT_SALINITY_NOTIFICATION_ENABLED;
      }
      else
      {
        evt.evt_type = BLE_CUS_EVT_SALINITY_NOTIFICATION_DISABLED;
      }
      // Call the application event handler.
      p_cus->evt_handler(p_cus, &evt);
    }
  }

  // Check if the Salinity value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->soil_handles.cccd_handle)
      && (p_evt_write->len == 2)
      )
  {
    // CCCD written, call application event handler
    if (p_cus->evt_handler != NULL)
    {
      ble_cus_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
        evt.evt_type = BLE_CUS_EVT_SOIL_NOTIFICATION_ENABLED;
      }
      else
      {
        evt.evt_type = BLE_CUS_EVT_SOIL_NOTIFICATION_DISABLED;
      }
      // Call the application event handler.
      p_cus->evt_handler(p_cus, &evt);
    }
  }

  // Check if the Light value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->light_handles.cccd_handle)
      && (p_evt_write->len == 2)
      )
  {
    // CCCD written, call application event handler
    if (p_cus->evt_handler != NULL)
    {
      ble_cus_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
        evt.evt_type = BLE_CUS_EVT_LIGHT_NOTIFICATION_ENABLED;
      }
      else
      {
        evt.evt_type = BLE_CUS_EVT_LIGHT_NOTIFICATION_DISABLED;
      }
      // Call the application event handler.
      p_cus->evt_handler(p_cus, &evt);
    }
  }
}

void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;
    
    NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cus, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}

/** Function for adding the Status Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t status_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->status_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = STATUS_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->status_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->status_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 1;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->status_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/** Function for adding the Temperature Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t temperature_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Add Custom Value characteristic
  memset(&cccd_md, 0, sizeof(cccd_md));

  // Read  operation on cccd should be possible without authentication.
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.write_perm = p_cus_init->temperature_char_attr_md.cccd_write_perm;
  cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = TEMPERATURE_CHAR_UUID;

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->temperature_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->temperature_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 2;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = 2;

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                             &attr_char_value,
                                             &p_cus->temperature_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return NRF_SUCCESS;
}

/** Function for adding the Humidity Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t humidity_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Add Custom Value characteristic
  memset(&cccd_md, 0, sizeof(cccd_md));

  // Read  operation on cccd should be possible without authentication.
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.write_perm = p_cus_init->humidity_char_attr_md.cccd_write_perm;
  cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = HUMIDITY_CHAR_UUID;

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->humidity_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->humidity_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 2;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = 2;

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                             &attr_char_value,
                                             &p_cus->humidity_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return NRF_SUCCESS;
}

/** Function for adding the Salinity Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t salinity_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Add Custom Value characteristic
  memset(&cccd_md, 0, sizeof(cccd_md));

  // Read  operation on cccd should be possible without authentication.
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.write_perm = p_cus_init->salinity_char_attr_md.cccd_write_perm;
  cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = SALINITY_CHAR_UUID;

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->salinity_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->salinity_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 2;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = 2;

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                             &attr_char_value,
                                             &p_cus->salinity_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return NRF_SUCCESS;
}

/** Function for adding the Soil Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t soil_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Add Custom Value characteristic
  memset(&cccd_md, 0, sizeof(cccd_md));

  // Read  operation on cccd should be possible without authentication.
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.write_perm = p_cus_init->soil_char_attr_md.cccd_write_perm;
  cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = SOIL_CHAR_UUID;

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->soil_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->soil_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 2;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = 2;

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                             &attr_char_value,
                                             &p_cus->soil_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return NRF_SUCCESS;
}

/** Function for adding the Light Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t light_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Add Custom Value characteristic
  memset(&cccd_md, 0, sizeof(cccd_md));

  // Read  operation on cccd should be possible without authentication.
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.write_perm = p_cus_init->light_char_attr_md.cccd_write_perm;
  cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = LIGHT_CHAR_UUID;

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->light_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->light_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 4;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = 4;

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                             &attr_char_value,
                                             &p_cus->light_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return NRF_SUCCESS;
}

uint32_t ble_cus_service_init(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init)
{
  if (p_cus == NULL || p_cus_init == NULL)
  {
      return NRF_ERROR_NULL;
  }

  uint32_t   err_code;
  ble_uuid_t ble_uuid;

  // Initialize service structure
  p_cus->evt_handler               = p_cus_init->evt_handler;
  p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

  // Add Custom Service UUID
  ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
  err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = CUSTOM_SERVICE_UUID;

  // Add the Custom Service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
  if (err_code != NRF_SUCCESS)
  {
      return err_code;
  }

  /* Add Status Value characteristic */
  err_code = status_char_add(p_cus, p_cus_init);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /* Add Temperature Value characteristic */
  err_code = temperature_char_add(p_cus, p_cus_init);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /* Add Humidity Value characteristic */
  err_code = humidity_char_add(p_cus, p_cus_init);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /* Add Salinity Value characteristic */
  err_code = salinity_char_add(p_cus, p_cus_init);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /* Add Soil Value characteristic */
  err_code = soil_char_add(p_cus, p_cus_init);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /* Add Light Value characteristic */
  return light_char_add(p_cus, p_cus_init);
}

/* Temperature Value Update */
uint32_t ble_cus_temperature_update(ble_cus_t *p_cus, uint16_t new_value)
{
    NRF_LOG_INFO("In ble_cus_temperature_update. \r\n");
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&new_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->temperature_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->temperature_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }

    return err_code;
}

/* Humidity Value Update */
uint32_t ble_cus_humidity_update(ble_cus_t *p_cus, uint16_t new_value)
{
  NRF_LOG_INFO("In ble_cus_humidity_update. \r\n");
  if (p_cus == NULL)
  {
    return NRF_ERROR_NULL;
  }

  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  // Initialize value struct.
  memset(&gatts_value, 0, sizeof(gatts_value));

  gatts_value.len     = sizeof(uint16_t);
  gatts_value.offset  = 0;
  gatts_value.p_value = (uint8_t*)&new_value;

  // Update database.
  err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                    p_cus->humidity_handles.value_handle,
                                    &gatts_value);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Send value if connected and notifying.
  if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID))
  {
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_cus->humidity_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len  = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
    NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
  }

  return err_code;
}

/* Salinity Value Update */
uint32_t ble_cus_salinity_update(ble_cus_t *p_cus, uint16_t new_value)
{
  NRF_LOG_INFO("In ble_cus_salinity_update. \r\n");
  if (p_cus == NULL)
  {
    return NRF_ERROR_NULL;
  }

  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  // Initialize value struct.
  memset(&gatts_value, 0, sizeof(gatts_value));

  gatts_value.len     = sizeof(uint16_t);
  gatts_value.offset  = 0;
  gatts_value.p_value = (uint8_t*)&new_value;

  // Update database.
  err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                    p_cus->salinity_handles.value_handle,
                                    &gatts_value);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Send value if connected and notifying.
  if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID))
  {
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_cus->salinity_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len  = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
    NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
  }

  return err_code;
}

/* Soil Value Update */
uint32_t ble_cus_soil_update(ble_cus_t *p_cus, uint16_t new_value)
{
  NRF_LOG_INFO("In ble_cus_soil_update. \r\n");
  if (p_cus == NULL)
  {
    return NRF_ERROR_NULL;
  }

  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  // Initialize value struct.
  memset(&gatts_value, 0, sizeof(gatts_value));

  gatts_value.len     = sizeof(uint16_t);
  gatts_value.offset  = 0;
  gatts_value.p_value = (uint8_t*)&new_value;

  // Update database.
  err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                    p_cus->soil_handles.value_handle,
                                    &gatts_value);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Send value if connected and notifying.
  if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID))
  {
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_cus->soil_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len  = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
    NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
  }

  return err_code;
}

/* Light Value Update */
uint32_t ble_cus_light_update(ble_cus_t *p_cus, uint32_t new_value)
{
  NRF_LOG_INFO("In ble_cus_light_update. \r\n");
  if (p_cus == NULL)
  {
    return NRF_ERROR_NULL;
  }

  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  // Initialize value struct.
  memset(&gatts_value, 0, sizeof(gatts_value));

  gatts_value.len     = sizeof(uint32_t);
  gatts_value.offset  = 0;
  gatts_value.p_value = (uint8_t*)&new_value;

  // Update database.
  err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                    p_cus->light_handles.value_handle,
                                    &gatts_value);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Send value if connected and notifying.
  if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID))
  {
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_cus->light_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len  = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
    NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
    NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
  }

  return err_code;
}