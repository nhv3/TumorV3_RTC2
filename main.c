#include "ADC.h"        // adc
#include "AD5592R.h"
#include "SEGGER_RTT.h" // debugging
#include "SPIFlash.h"   // adc
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "nrfx_rtc.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "bsp_btn_ble.h"
#include "custom_board.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdm.h"
#include "peer_manager.h"
#include "sensor_service.h"
#include "sensorsim.h"
#include "sdk_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
// to change mouse change "Mouse1" and #include "sensor_service_X.h"in sensor_service.h
#define DEVICE_NAME "Mouse1"                    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "Stanford University" /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 24000                    //15sec - 24000                     /**< The advertising interval (in units of 0.625 ms (16 kHz). This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION 1800                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO 3                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1                            /**< A tag identifying the SoftDevice BLE configuration. */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(400, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(500)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */
#define BT0_SHORT_PUSH BSP_EVENT_KEY_0
#define BT0_LONG_PUSH BSP_EVENT_KEY_1

const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
#define COMPARE_COUNTERTIME (900UL)                /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. 15 MINUTE CONFIG */

#define DEAD_BEEF 0xDEADBEEF      /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define CONFIG_NFCT_PINS_AS_GPIOS // also needed to change the settings https://devzone.nordicsemi.com/f/nordic-q-a/35505/nrf52-enabling-gpio-on-nfc-pins
static uint8_t data[16];
//#if USE_BSP == 0

#define PIN_0 3
int hi = 0;
APP_TIMER_DEF(m_button_action);

#define BUTTON_STATE_POLL_INTERVAL_MS 100UL

#define LONG_PRESS(MS) (uint32_t)(MS) / BUTTON_STATE_POLL_INTERVAL_MS

uint8_t record_counter = 0;
static uint8_t flash_page_buffer[16 * 16];
static uint16_t GLOB_datastart = 0;
static uint32_t fb[4];

float s1 = 2.9; // 1.779 for OECTs, 1.995 for multi-channel
float s2 = 0;   // 1.779
float s3 = 0;   // 1.779
float s4 = 0;   // 1.779
float bias = 0.00;
static float volt;
//static float  resultFDC[4];
float adc2_val;
uint16_t adc2[602];
uint16_t i_off[300];
uint16_t i_on[300];
static char float_str[80];
int32_t ch1_val;
int32_t ch2_val;
int32_t ch3_val;
int32_t ch4_val;
int32_t ch5_val;
int32_t batt_val;
int32_t sample_count = 0;
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
int32_t update_ch1(void);
int32_t update_ch2(void);
int32_t update_ch3(void);
int32_t update_ch4(void);
int32_t update_batt(void);
int32_t line_count = 0;
int32_t page_count = 0;
int32_t sample_sent = 0;
bool streaming = false; //for testing
bool rtc_fired_bool = false;

// FROM_SERVICE_TUTORIAL: Declare a service structure for sensor application
ble_os_t m_sensor_service;

// SENSOR_JOB: Step 3.G, Declare an app_timer id variable and define sensor timer interval and define a timer interval
APP_TIMER_DEF(m_sensor_char_timer_id);
//APP_TIMER_DEF(meas_timer);
#define SENSOR_CHAR_TIMER_INTERVAL APP_TIMER_TICKS(100) //1000 = 1000 ms intervals
//#define MEAS_TIMER_INTERVAL APP_TIMER_TICKS(60000)
// Use UUIDs for service(s) used in ysensor application.
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {
        {BLE_UUID_SERVICE_UUID, BLE_UUID_TYPE_BLE} //BLE_UUID_TYPE_VENDOR_BEGIN
};

static void advertising_start(bool erase_bonds);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how ysensor product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

// ALREADY_DONE_FOR_YOU: This is a timer event handler
static void timer_timeout_handler(void *p_context) {
  // SENSOR_JOB: Step 3.F, Update temperature and characteristic value.

  FLASH_Line_Read(sample_sent, data);
  SEGGER_RTT_printf(0, "data , %d, \n", data);
  if (sample_sent >= sample_count) {
    sample_sent = 0;
    //                nrf_delay_ms(15000);
    //               uint32_t err_code;
    //                           err_code = sd_ble_gap_disconnect(m_conn_handle,
    //                                             BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    ////            if (err_code != NRF_ERROR_INVALID_STATE)
    ////            {
    //                APP_ERROR_CHECK(err_code);
    //            }
    // break; // BSP_EVENT_DISCONNECT
    //
    //if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    //{
    //    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
    //                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    //    if (err_code != NRF_ERROR_INVALID_STATE)
    //    {
    //        APP_ERROR_CHECK(err_code);
    //    }
    //}
    //
  }
  int32_t test[4];
  //memcpy(data, &test, sizeof(test));

  test[0] = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]; //Declare a variable holding temperature value
  test[1] = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
  test[2] = (data[11] << 24) | (data[10] << 16) | (data[9] << 8) | data[8];
  test[3] = (data[15] << 24) | (data[14] << 16) | (data[13] << 8) | data[12];

  //                uint32_t temperature = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];      //Declare a variable holding temperature value
  //                uint32_t ch1 = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4];
  //                uint32_t ch2 = (data[11]<<24)|(data[10]<<16)|(data[9]<<8)|data[8];
  //                uint32_t ch3 = (data[15]<<24)|(data[14]<<16)|(data[13]<<8)|data[12];
  //                uint32_t ch4 = (data[15]<<24)|(data[14]<<16)|(data[13]<<8)|data[12];
  //                for(int i = 0;   i <  16; i++) { SEGGER_RTT_printf(0, "%d ", data[i]); };
  SEGGER_RTT_printf(0, "batt , %d, \n", test[0]);
  SEGGER_RTT_printf(0, "c1 , %d, \n", test[1]);
  SEGGER_RTT_printf(0, "c2 , %d, \n", test[2]);
  SEGGER_RTT_printf(0, "c3 , %d, \n", test[3]);
  sample_sent++;

  //                //static int16_t previous_temperature=0;      //Declare a variable to store current temperature until next measurement.
  //
  //		//sd_temp_get(&temperature);    //Get temperature
  //                temperature = update_batt();
  //                ch1 = update_ch1();
  //                ch2 = update_ch2();
  //                ch3 = update_ch3();
  //                ch4 = update_ch4();

  sensor1_characteristic_update(&m_sensor_service, &test);
  //sensor2_characteristic_update(&m_sensor_service, &ch1);
  //sensor3_characteristic_update(&m_sensor_service, &ch2);
  //sensor4_characteristic_update(&m_sensor_service, &ch3);
  //sensor5_characteristic_update(&m_sensor_service, &ch4);

  // Save current temperature until next measurement
  //previous_temperature = temperature;
  //nrf_gpio_pin_toggle(LED_4);
}

uint32_t measure_res();
static void meas_timer_handler(void *p_context) {
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  ret_code_t err_code;

  switch (p_evt->evt_id) {
  case PM_EVT_BONDED_PEER_CONNECTED: {
    NRF_LOG_INFO("Connected to a previously bonded device.");
  } break;

  case PM_EVT_CONN_SEC_SUCCEEDED: {
    NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
        ble_conn_state_role(p_evt->conn_handle),
        p_evt->conn_handle,
        p_evt->params.conn_sec_succeeded.procedure);
  } break;

  case PM_EVT_CONN_SEC_FAILED: {
    /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
  } break;

  case PM_EVT_CONN_SEC_CONFIG_REQ: {
    // Reject pairing request from an already bonded peer.
    pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
    pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
  } break;

  case PM_EVT_STORAGE_FULL: {
    // Run garbage collection on the flash.
    err_code = fds_gc();
    if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES) {
      // Retry.
    } else {
      APP_ERROR_CHECK(err_code);
    }
  } break;

  case PM_EVT_PEERS_DELETE_SUCCEEDED: {
    advertising_start(false);
  } break;

  case PM_EVT_PEER_DATA_UPDATE_FAILED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
  } break;

  case PM_EVT_PEER_DELETE_FAILED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
  } break;

  case PM_EVT_PEERS_DELETE_FAILED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
  } break;

  case PM_EVT_ERROR_UNEXPECTED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
  } break;

  case PM_EVT_CONN_SEC_START:
  case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
  case PM_EVT_PEER_DELETE_SUCCEEDED:
  case PM_EVT_LOCAL_DB_CACHE_APPLIED:
  case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
    // This can happen when the local DB has changed.
  case PM_EVT_SERVICE_CHANGED_IND_SENT:
  case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
  default:
    break;
  }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  // Initialize timer module.
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  // SENSOR_JOB: Step 3.H, Initiate sensor timer
  app_timer_create(&m_sensor_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
  // app_timer_create(&meas_timer, APP_TIMER_MODE_REPEATED, meas_timer_handler);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {

  uint32_t err_code;
  nrf_ble_qwr_init_t qwr_init = {0};

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  //FROM_SERVICE_TUTORIAL: Add code to initialize the services used by the application.
  sensor_service_init(&m_sensor_service);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void) {

  // SENSOR_JOB: Step 3.I, Start sensor timer
  //app_timer_start(m_sensor_char_timer_id, SENSOR_CHAR_TIMER_INTERVAL, NULL);
  //app_timer_start(meas_timer, MEAS_TIMER_INTERVAL, NULL);
  //To update temperature only when in a connection then don't call app_timer_start() here, but in ble_event_handler()
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  ret_code_t err_code;

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    NRF_LOG_INFO("Fast advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_IDLE:
    sleep_mode_enter();
    break;

  default:
    break;
  }
}

//static void leds_init( void )
//{
//    ret_code_t err_code = bsp_init(BSP_INIT_LED, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code = NRF_SUCCESS;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected.");
    // LED indication will be changed when advertising starts.
    err_code = sd_ble_gap_adv_stop(p_ble_evt->evt.gattc_evt.conn_handle);
    if (err_code != NRF_ERROR_INVALID_STATE) {
      APP_ERROR_CHECK(err_code);
    }
    nrfx_rtc_enable(&rtc);
    app_timer_stop(m_sensor_char_timer_id);
    break;

  case BLE_GAP_EVT_CONNECTED:
  nrfx_rtc_disable(&rtc);
    NRF_LOG_INFO("Connected.");
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);

    //When connected; start sensor timer to start regular temperature measurements
    app_timer_start(m_sensor_char_timer_id, SENSOR_CHAR_TIMER_INTERVAL, NULL);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

  //SENSOR_JOB: Step 3.C Set up a BLE event observer to call ble_sensor_service_on_ble_evt() to do housekeeping of ble connections related to sensor service and characteristics.
  NRF_SDH_BLE_OBSERVER(m_sensor_service_observer, APP_BLE_OBSERVER_PRIO, ble_sensor_service_on_ble_evt, (void *)&m_sensor_service);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond = SEC_PARAM_BOND;
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event) {
  ret_code_t err_code;

  switch (event) {
  case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break; // BSP_EVENT_SLEEP

  case BSP_EVENT_DISCONNECT:
    err_code = sd_ble_gap_disconnect(m_conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE) {
      APP_ERROR_CHECK(err_code);
    }
    break; // BSP_EVENT_DISCONNECT

  case BSP_EVENT_WHITELIST_OFF:
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
      err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
      }
    }

    break; // BSP_EVENT_KEY_0
  case BT0_SHORT_PUSH:
    NRF_LOG_INFO("Short button press");

    break;

  case BT0_LONG_PUSH:
    NRF_LOG_INFO("Long button press");
    break;

  default:
    break;
  }
}

uint32_t get_rtc_counter(void) {
  return NRF_RTC1->COUNTER;
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
//static void buttons_leds_init(bool * p_erase_bonds)
//{
//    ret_code_t err_code;
//    bsp_event_t startup_event;
//
//    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
//
//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
//}

static void buttons_leds_init(bool *p_erase_bonds) {
  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_event_to_button_action_assign(BSP_BOARD_BUTTON_0, BSP_BUTTON_ACTION_PUSH, BT0_SHORT_PUSH);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_event_to_button_action_assign(BSP_BOARD_BUTTON_0, BSP_BUTTON_ACTION_LONG_PUSH, BT0_LONG_PUSH);
  APP_ERROR_CHECK(err_code);

  /*err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);*/
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

// Initialize the SoftDevice handler module.
//SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
  } else {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
  }
}

void button_timeout_handler(void *p_context) {
  uint32_t err_code;
  static uint32_t cnt;

  if (app_button_is_pushed(0)) {
    cnt++;
    if (cnt >= LONG_PRESS(750)) {
      cnt = 0;
      SEGGER_RTT_printf(0, "Long Button press"); //SEGGER_RTT_printf(0, "Done setup 2");
      flash_blue();
      flash_blue();
      flash_blue();
      FLASH_Erase();
      record_counter = 0;
      GLOB_datastart = 0;
    } else {
      err_code = app_timer_start(m_button_action,
          APP_TIMER_TICKS(BUTTON_STATE_POLL_INTERVAL_MS),
          NULL);
      APP_ERROR_CHECK(err_code);
    }
  } else {
    cnt = 0; // reset counter variable
    SEGGER_RTT_printf(0, "Short button press");
    flash_white();
    bool erase_bonds;
    advertising_start(erase_bonds); //moved from here
    sample_count = (16 * GLOB_datastart + record_counter);
    SEGGER_RTT_printf(0, "Sample collected, %d, \n", sample_count);
  }
}

void button_callback(uint8_t pin_no, uint8_t button_action) {
  uint32_t err_code;

  if ((pin_no == BUTTON_1) && (button_action == APP_BUTTON_PUSH)) {
    err_code = app_timer_start(m_button_action,
        APP_TIMER_TICKS(BUTTON_STATE_POLL_INTERVAL_MS),
        NULL);
    APP_ERROR_CHECK(err_code);
  }
}

static void buttons_init() {
  uint32_t err_code;

  static app_button_cfg_t button_cfg;

  button_cfg.pin_no = BUTTON_1;
  button_cfg.button_handler = button_callback;
  button_cfg.pull_cfg = NRF_GPIO_PIN_PULLUP;
  button_cfg.active_state = APP_BUTTON_ACTIVE_LOW;

  err_code = app_button_init(&button_cfg, 1, 100);
  APP_ERROR_CHECK(err_code);

  err_code = app_button_enable();
  APP_ERROR_CHECK(err_code);

  /*Enable an app timer instance to detect long button press*/
  err_code = app_timer_create(&m_button_action, APP_TIMER_MODE_SINGLE_SHOT, button_timeout_handler);
  APP_ERROR_CHECK(err_code);
}

void flash_blue(void) {
  nrf_gpio_pin_clear(LED_RGB_BLUE);
  nrf_delay_ms(250);
  nrf_gpio_pin_set(LED_RGB_BLUE);
}

void flash_green(void) {
  nrf_gpio_pin_clear(LED_RGB_GREEN);
  nrf_delay_ms(250);
  nrf_gpio_pin_set(LED_RGB_GREEN);
}

void flash_red(void) {
  nrf_gpio_pin_clear(LED_RGB_RED);
  nrf_delay_ms(250);
  nrf_gpio_pin_set(LED_RGB_RED);
}

void flash_white(void) {
  nrf_gpio_pin_clear(LED_RGB_BLUE);
  nrf_gpio_pin_clear(LED_RGB_GREEN);
  nrf_gpio_pin_clear(LED_RGB_RED);
  nrf_delay_ms(250);
  nrf_gpio_pin_set(LED_RGB_BLUE);
  nrf_gpio_pin_set(LED_RGB_GREEN);
  nrf_gpio_pin_set(LED_RGB_RED);
}

void flash_none(void) {
  nrf_gpio_pin_set(LED_RGB_BLUE);
  nrf_gpio_pin_set(LED_RGB_GREEN);
  nrf_gpio_pin_set(LED_RGB_RED);
}

int32_t update_batt() {
  return batt_val;
}

int32_t update_ch1() {
  return ch1_val;
}

int32_t update_ch2() {
  return ch2_val;
}

int32_t update_ch3() {
  return ch3_val;
}

int32_t update_ch4() {
  return ch4_val;
}

void select_ch1()

{
  flash_red();
  nrf_gpio_pin_clear(MUX_EN);
  nrf_gpio_pin_clear(MUX_A0);
  nrf_gpio_pin_clear(MUX_A1);
}

void select_ch2() {
  //flash_green();

  nrf_gpio_pin_clear(MUX_EN);
  nrf_gpio_pin_set(MUX_A0);
  nrf_gpio_pin_clear(MUX_A1);
}

void select_ch3() {
  //flash_blue();

  nrf_gpio_pin_clear(MUX_EN);
  nrf_gpio_pin_clear(MUX_A0);
  nrf_gpio_pin_set(MUX_A1);
}

void select_ch4() {
  //flash_white();

  nrf_gpio_pin_clear(MUX_EN);
  nrf_gpio_pin_set(MUX_A0);
  nrf_gpio_pin_set(MUX_A1);
}

void FLASH_Write_Record(uint8_t wp[]) {
  //we write in blocks of 16 lines.
  if (record_counter < 16) {
    //keep adding to buffer...
    for (uint8_t i = 0; i < 16; i++) {
      flash_page_buffer[record_counter * 16 + i] = wp[i];
    };

    record_counter++;
  }

  if (record_counter >= 16) {
    flash_green();
    flash_green();
    flash_green();

    //SEGGER_RTT_printf(0, "TH:%d VB:%d DP:%d\r\n", heartbeat16, battery_level8, GLOB_datastart);
    //SEGGER_RTT_printf(0, "Datapage:%d\r\n", GLOB_datastart);

    //flush to memory....
    FLASH_Page_Write(GLOB_datastart, flash_page_buffer);

    //advance counter
    GLOB_datastart++;

    //clear the buffer
    memset(flash_page_buffer, 0, sizeof(flash_page_buffer));

    //and reset the counter
    record_counter = 0;
  }
};

uint32_t measure_res() {
  //ad5592r_init();
  nrf_delay_ms(10);
  bias = 0.25;
  ad5592r_dac_out(1, 250); // 250 mV in ch1 250/1250/2500ad5593r_write_dac(0, dac_value_1); //
  nrf_delay_ms(100);
  adc2_val = return_adc3();
  volt = 1000 * ((adc2_val * .6 * 6) / 4096) / 5.0;
  //sprintf(float_str, "Volt1, %f\n", volt);
  //SEGGER_RTT_WriteString(0, float_str);
  if (volt < 50.0) {
    //SEGGER_RTT_printf(0, "Bias 1.65 ...\n");
    bias = 1.25;
    ad5592r_dac_out(1, 1250); // 250 mV in ch1 250/1250/2500ad5593r_write_dac(0, dac_value_1); //
    nrf_delay_ms(100);
    adc2_val = return_adc3();
    nrf_delay_ms(100);
    volt = 1000 * ((adc2_val * .6 * 6) / 4096) / 5.0;
    if (volt < 50.00) { //SEGGER_RTT_printf(0, "Bias 2.9 ...\n");
      bias = 2.5;
      ad5592r_dac_out(1, 2500); // 250 mV in ch1 250/1250/2500ad5593r_write_dac(0, dac_value_1); //
      nrf_delay_ms(100);
      adc2_val = return_adc3();
      nrf_delay_ms(100);
    }
  }
  volt = 1000 * ((adc2_val * .6 * 6) / 4096) / 5.0;
  sprintf(float_str, "Volt3, %f\n", volt);
  //SEGGER_RTT_WriteString(0, float_str);
  //float res_1 = 2200*((1-(volt/(bias*1000)))/ (volt/(bias*1000))) - 200;//370 was added by the internal res.
  float res_1 = 2200 * ((1 - (volt / (bias * 1000))) / (volt / (bias * 1000))) - 240; //adjusting the batt volt drop
  sprintf(float_str, "Bias, %f\n", bias);
  //SEGGER_RTT_WriteString(0, float_str);
  sprintf(float_str, "Res, %f\n", res_1);
  //SEGGER_RTT_WriteString(0, float_str);
  uint32_t resd1 = res_1;
  //SEGGER_RTT_printf(0, "Resd, %d, " , resd1);
  return resd1;
}

void read_data(void)
{
float adc7 = return_adc7();
    float batt = (adc7 * .6 * 6) / 4096;
    batt_val = batt * 1000;
    sprintf(float_str, "Batt, %f\n", batt);
    SEGGER_RTT_WriteString(0, float_str);
    //SEGGER_RTT_printf(0, "batt");
    //SEGGER_RTT_printf(0, "Batt, %d, \n" , batt_val);
    ad5592r_dac_out(1, 0);

    nrf_delay_ms(100);

    // channel 1
    select_ch1();
    uint32_t ress1 = measure_res();
    ch1_val = ress1;
    SEGGER_RTT_printf(0, "Res1, %d\r\n", ress1);
    ad5592r_dac_out(1, 0);
    nrf_delay_ms(100);
    flash_none();

    // channel 1
    select_ch2();
    uint32_t ress2 = measure_res();
    ch2_val = ress2;
    SEGGER_RTT_printf(0, "Res2, %d, \n", ress2);
    ad5592r_dac_out(1, 0);
    nrf_delay_ms(100);

    // channel 1
    select_ch3();
    uint32_t ress3 = measure_res();
    ch3_val = ress3;
    SEGGER_RTT_printf(0, "Res3, %d, \n", ress3);
    ad5592r_dac_out(1, 0);
    nrf_delay_ms(100);

    // channel 1
    select_ch4();
    uint32_t ress4 = measure_res();
    ch4_val = ress4;
    SEGGER_RTT_printf(0, "Res4, %d, \n", ress4);
    ad5592r_dac_out(1, 0);
    nrf_delay_ms(100);
    // flash 16 byte data

    fb[0] = batt_val;
    fb[1] = ress1;
    fb[2] = ress2;
    fb[3] = ress3;
    if (record_counter == 0 && GLOB_datastart == 0) {
      for (uint16_t i = 0; i < 15; i++) {
        FLASH_Write_Record(fb);
      }
    }

    FLASH_Write_Record(fb);
    SEGGER_RTT_printf(0, "record_counter , %d, \n", record_counter);
    SEGGER_RTT_printf(0, "GLOB_datastart, %d, \n", GLOB_datastart);

    //Here we start for the first time and need to put the chip to sleep, on wakeup via RTC external event, thats when we need to take the data again
    //So first, lets put the guy to sleep in SYSTEM ON mode. SYSTEM OFF mode shuts us entirely down to the point where we wake up, we actually have to endure
    //the normal power bootup routine which is arguably more hurtful than the advertising spikes. We also cant use RTC in SYSTEM OFF either...

    //We will use RTC2 FROM NRFX. I did something illegal because I did not want to spend hours on hours migrating drivers from legacy to the new nrfx versions. 
    //So if we ever need to change anything with the RTC drivers, please contact Nick Vitale. Esentially I have enabled both drivers but only attaching the nrfx 
    //to RTC2 and allowing the legacy driver to handle RTC1. This is becuase RTC1 is used by app_timer and the app_timer is scaled from RTC1. Thus if we 
    //prescale RTC1 like we do in RTC2, everyhting will be clocked slower than needed and lead to performance hangs. Sorry, but this was done on a tight timeline. 

    //I also modified the actual nrfx_rtc.c file to allow the COMPARE0 event to fire simultaneous and reload every 15 minutes by default. So be aware is a RELOAD TIMER CONFIG

}



//Invoke rct2 handle
static void rtc2_handler(nrfx_rtc_int_type_t int_type) {
  
  if (int_type == NRFX_RTC_INT_COMPARE2) 
  {  
      SEGGER_RTT_printf(0, "RTC2 Timer fired!");
      nrfx_rtc_counter_clear(&rtc);
      rtc_fired_bool = true;
  }

}

static void lfclk_config(void) {
  ret_code_t err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC2 driver instance.
 */
static void rtc_config(void) {
  uint32_t err_code;

      //Initialize RTC instance
      nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
      config.prescaler = 4095;
      err_code = nrfx_rtc_init(&rtc, &config, rtc2_handler);
      APP_ERROR_CHECK(err_code);

  //Set trigger time, COMPARE_COUNTER_TIME is set to 900Ul so 15 minutes! 
      err_code = nrfx_rtc_cc_set(&rtc, 2, COMPARE_COUNTERTIME * 8, true);
      APP_ERROR_CHECK(err_code);

  //Enable tick event & interrupt
      //nrfx_rtc_tick_enable(&rtc, true);

  //Power on RTC instance
      
}

static void idle_handle_state()
{ 

//If we dont have anything to do, just go to sleep 
  if(NRF_LOG_PROCESS() == 0)
  {

    nrf_pwr_mgmt_run();
  }

}

/**@brief Function for application main entry.
 */
int main(void) {

  log_init();
  lfclk_config();
  nrf_delay_ms(200); //Stab time for vlfclk
  rtc_config();
  timers_init();
  //buttons_leds_init(&erase_bonds);
  power_management_init();
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

#if USE_BSP == 0
  buttons_init();
#else
  buttons_leds_init(&erase_bonds);
#endif

  nrf_gpio_cfg_output(LED_RGB_RED);
  nrf_gpio_cfg_output(LED_RGB_BLUE);
  nrf_gpio_cfg_output(LED_RGB_GREEN);
  nrf_gpio_pin_set(LED_RGB_RED); // nrf_gpio_pin_toggle(LED_PIN); nrf_gpio_pin_set(LED_PIN);
  nrf_gpio_pin_set(LED_RGB_GREEN);
  nrf_gpio_pin_set(LED_RGB_BLUE);
  flash_green();

  // setting up mux
  nrf_gpio_pin_clear(MUX_EN);
  nrf_gpio_cfg_output(MUX_EN);
  nrf_gpio_pin_clear(MUX_EN);

  nrf_gpio_cfg_output(MUX_A0);
  nrf_gpio_pin_clear(MUX_A0);

  nrf_gpio_pin_clear(MUX_A1);
  nrf_gpio_cfg_output(MUX_A1);
  nrf_gpio_pin_clear(MUX_A1);
  //
  // setting up potentiostat

  //nrf_gpio_cfg_output(P1_MEN);
  //nrf_gpio_pin_clear(P1_MEN);

  // setup ble here
  //SEGGER_RTT_printf(0, "Done setup");

  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  //advertising_init();
  //segger_rtt_printf(0, "ble starting ...");
  conn_params_init();
  peer_manager_init();
  //SEGGER_RTT_printf(0, "Done setup 2");

  // start execution.
  //nrf_log_info("ourcharacteristics tutorial started.");

  // uint32_t err_code = NRF_LOG_INIT(get_rtc_counter);

  //  nrf_gpio_cfg_output(DAC_A0);
  //  nrf_gpio_pin_clear(DAC_A0);

  nrf_gpio_cfg_output(DAC_RST);
  nrf_gpio_pin_set(DAC_RST);
  nrf_delay_ms(1000);

  // dac code start
  nrf_gpio_cfg_output(DAC_RST);
  nrf_gpio_pin_clear(DAC_RST);
  nrf_delay_ms(10);
  nrf_gpio_pin_set(DAC_RST);
  nrf_delay_ms(10);

  select_ch4();
  ad5592r_init();
  ad5592r_dac_out(1, 500); // 250 mV in ch1 250/1250/2500
  SEGGER_RTT_printf(0, "Dac done");
  //     saadc_init();
  SEGGER_RTT_printf(0, "Done setup 4");
  //SEGGER_RTT_printf(0, "ADC 1 ...\n");
  // saadc_sampling_event_init();
  SEGGER_RTT_printf(0, "Done setup 5");

  FLASH_Init();
  FLASH_Print_ID();
  GLOB_datastart = FLASH_Get_First_Available_Location();
  SEGGER_RTT_printf(0, "First empty page: %d\r\n", GLOB_datastart);
  //measure_res();
  application_timers_start(); //

  ret_code_t err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  APP_ERROR_CHECK(err_code);
  err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  APP_ERROR_CHECK(err_code);
  nrf_pwr_mgmt_init();
  nrfx_rtc_enable(&rtc);

while(true) 
  {
    if(rtc_fired_bool)
    {
      read_data();
      rtc_fired_bool = false;
     }
    sd_app_evt_wait();
  }
}

