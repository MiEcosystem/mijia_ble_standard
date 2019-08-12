/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
//
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "fstorage.h"
#include "fds.h"
#include "ble_nus.h"

#define NRF_LOG_MODULE_NAME "MAIN"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "SDK12.3.0_patch/nrf_drv_twi_patched.h"

#include <time.h>
#include "SEGGER_RTT.h"
#include "nRF5_evt.h"
#include "mible_log.h"
#include "common/mible_beacon.h"
#include "secure_auth/mible_secure_auth.h"
#include "mijia_profiles/mi_service_server.h"
#include "mijia_profiles/lock_service_server.h"
#include "mi_config.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#ifdef NRF52
#define DEVICE_NAME                     "Secure_nRF52"                              /**< Name of device. Will be included in the advertising data. */
#else
#define DEVICE_NAME                     "Secure_nRF51"                              /**< Name of device. Will be included in the advertising data. */
#endif

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(200, UNIT_0_625_MS)           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         16                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (10 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (40 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(900, UNIT_10_MS)              /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(15000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define RTTCTRL_CLEAR                   "[2J"

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */


APP_TIMER_DEF(poll_timer);
APP_TIMER_DEF(m_bindconfirm_timer);

#define PAIRCODE_NUMS 6
bool need_kbd_input;
uint8_t pair_code_num;
uint8_t pair_code[PAIRCODE_NUMS];


#if defined(BOARD_PCA10028)
#define MSC_PWR_PIN 25
const iic_config_t iic_config = {
        .scl_pin  = 28,
        .sda_pin  = 29,
        .freq = IIC_100K
};
#elif defined(BOARD_PCA10040)
#define MSC_PWR_PIN 23
const iic_config_t iic_config = {
        .scl_pin  = 24,
        .sda_pin  = 25,
        .freq = IIC_100K
};
#endif


static void advertising_init(bool);
static void ble_lock_ops_handler(uint8_t opcode);
static void poll_timer_handler(void * p_context);
static void bind_confirm_timeout(void * p_context);
void time_init(struct tm * time_ptr);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
    ret_code_t err_code;
    err_code = app_timer_create(&poll_timer, APP_TIMER_MODE_REPEATED, poll_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_bindconfirm_timer, APP_TIMER_MODE_SINGLE_SHOT, bind_confirm_timeout);
    MI_ERR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */

}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
//    APP_ERROR_HANDLER(nrf_error);
    NRF_LOG_ERROR("conn param error %X\n", nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{

    mible_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event @ NRF_SOC_EVTS.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = {.source        = NRF_CLOCK_LF_SRC_XTAL,
                                       .rc_ctiv       = 0,
                                       .rc_temp_ctiv  = 0,
                                       .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM};

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    ble_enable_params.common_enable_params.vs_uuid_count = 4;
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif

#if (IS_SRVC_CHANGED_CHARACT_PRESENT == 1)
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif

    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Subscribe for SOC events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_KEY_2:
            mi_scheduler_start(SYS_MSC_SELF_TEST);
            break;

        case BSP_EVENT_KEY_3:
            mi_scheduler_start(SYS_KEY_DELETE);
            break;

        case BSP_EVENT_KEY_4:
            advertising_init(true);
            err_code = app_timer_start(m_bindconfirm_timer, APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER), NULL);
            MI_ERR_CHECK(err_code);
            break;

        default:
            break;
    }
}



/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(bool need_bind_confirm)
{
    MI_LOG_INFO("advertising init...\n");
    mibeacon_frame_ctrl_t frame_ctrl = {
        .secure_auth    = 1,
        .version        = 5,
        .bond_confirm   = need_bind_confirm,
    };

    mibeacon_capability_t cap = {.connectable = 1,
                                 .encryptable = 1,
                                 .bondAbility = 1};
    mibeacon_cap_sub_io_t IO = {.in_digits = 1};
    mible_addr_t dev_mac;
    mible_gap_address_get(dev_mac);

    mibeacon_config_t mibeacon_cfg = {
        .frame_ctrl = frame_ctrl,
        .pid = PRODUCT_ID,
        .p_mac = (mible_addr_t*)dev_mac, 
        .p_capability = &cap,
        .p_cap_sub_IO = &IO,
        .p_obj = NULL,
    };

    uint8_t adv_data[31];
    uint8_t adv_len;

    // ADV Struct: Flags: LE General Discoverable Mode + BR/EDR Not supported.
    adv_data[0] = 0x02;
    adv_data[1] = 0x01;
    adv_data[2] = 0x06;
    adv_len     = 3;
    
    uint8_t service_data_len;
    if(MI_SUCCESS != mible_service_data_set(&mibeacon_cfg, adv_data+3, &service_data_len)){
        MI_LOG_ERROR("encode service data failed. \r\n");
        return;
    }
    adv_len += service_data_len;
    
    MI_LOG_HEXDUMP(adv_data, adv_len);

    mible_gap_adv_data_set(adv_data, adv_len, adv_data, 0);

    return;
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
//    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    /* assign BUTTON 2 to initate MSC_SELF_TEST, for more details to check bsp_event_handler()*/
    err_code = bsp_event_to_button_action_assign(1,
                                             BSP_BUTTON_ACTION_PUSH,
                                             BSP_EVENT_KEY_2);
    APP_ERROR_CHECK(err_code);

    /* assign BUTTON 3 to clear KEYINFO in the FLASH, for more details to check bsp_event_handler()*/
    err_code = bsp_event_to_button_action_assign(2,
                                             BSP_BUTTON_ACTION_PUSH,
                                             BSP_EVENT_KEY_3);
    APP_ERROR_CHECK(err_code);

    /* assign BUTTON 4 to set the bind confirm bit in mibeacon, for more details to check bsp_event_handler()*/
    err_code = bsp_event_to_button_action_assign(3,
                                             BSP_BUTTON_ACTION_PUSH,
                                             BSP_EVENT_KEY_4);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    mible_gap_adv_param_t adv_param =(mible_gap_adv_param_t){
        .adv_type = MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED,
        .adv_interval_min = MSEC_TO_UNITS(200, UNIT_0_625_MS),
        .adv_interval_max = MSEC_TO_UNITS(300, UNIT_0_625_MS),
    };
    uint32_t err_code = mible_gap_adv_start(&adv_param);
    if(MI_SUCCESS != err_code){
        MI_LOG_ERROR("adv failed. %d \n", err_code);
    }
}

void poll_timer_handler(void * p_context)
{
    time_t utc_time = time(NULL);
    NRF_LOG_RAW_INFO(NRF_LOG_COLOR_CODE_GREEN"%s", nrf_log_push(ctime(&utc_time)));

    if (get_mi_reg_stat()) {
        uint8_t battery_stat = 100;
        mibeacon_obj_enque(MI_STA_BATTERY, sizeof(battery_stat), &battery_stat);
    }
}


static void bind_confirm_timeout(void * p_context)
{
    MI_LOG_WARNING("bind confirm bit clear.\n");
    advertising_init(0);
}

int scan_keyboard(uint8_t *pdata, uint8_t len)
{
    if (pdata == NULL)
        return 0;

    return SEGGER_RTT_ReadNoLock(0, pdata, len);
}

void flush_keyboard_buffer(void)
{
    uint8_t tmp[16];
    while(SEGGER_RTT_ReadNoLock(0, tmp, 16));
}

void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch (p_event->id) {
    case SCHD_EVT_OOB_REQUEST:
        MI_LOG_INFO("App selected IO cap is 0x%04X\n", p_event->data.IO_capability);
        switch (p_event->data.IO_capability) {
        case 0x0001:
            need_kbd_input = true;
            flush_keyboard_buffer();
            MI_LOG_INFO(MI_LOG_COLOR_GREEN "Please input your pair code ( MUST be 6 digits ) : \n");
            break;

        default:
            MI_LOG_ERROR("Selected IO cap is not supported.\n");
            mible_gap_disconnect(0);
        }
        break;

    case SCHD_EVT_KEY_DEL_SUCC:
        // device has been reset, restart adv mibeacon contains IO cap.
        advertising_init(0);
        break;

    default:
        break;
    }
}


int mijia_secure_chip_power_manage(bool power_stat)
{
    if (power_stat == 1) {
        nrf_gpio_cfg_output(MSC_PWR_PIN);
        nrf_gpio_pin_set(MSC_PWR_PIN);
    } else {
        nrf_gpio_pin_clear(MSC_PWR_PIN);
    }
    return 0;
}


static void ble_lock_ops_handler(uint8_t opcode)
{
    switch(opcode) {
    case 0:
        MI_LOG_INFO(" unlock \n");
        bsp_board_led_off(1);
        bsp_board_led_off(2);
        break;

    case 1:
        MI_LOG_INFO(" lock \n");
        bsp_board_led_on(1);
        break;

    case 2:
        MI_LOG_INFO(" bolt \n");
        bsp_board_led_on(2);
        break;

    default:
        MI_LOG_ERROR("lock opcode error %d", opcode);
    }

    lock_event_t lock_event;
    lock_event.action = opcode;
    lock_event.method = 0;
    lock_event.user_id= get_mi_key_id();
    lock_event.time   = time(NULL);

    mibeacon_obj_enque(MI_EVT_LOCK, sizeof(lock_event), &lock_event);

    reply_lock_stat(opcode);
    send_lock_log(MI_EVT_LOCK, sizeof(lock_event), &lock_event);
}


/**@brief Application main function.
 */
int main(void)
{
    NRF_LOG_INIT(NULL);
    MI_LOG_INFO(RTTCTRL_CLEAR "Compiled  %s %s\n", (uint32_t)__DATE__, (uint32_t)__TIME__);

    // Initialize stack and SDK modules.
    timers_init();
    buttons_leds_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init(false);
    conn_params_init();

    time_init(NULL);

    // Initialize mi scheduler
    mible_libs_config_t config = {
        .msc_onoff        = mijia_secure_chip_power_manage,
        .p_msc_iic_config = (void*)&iic_config
    };

    /* <!> mi_scheduler_init() must be called after ble_stack_init(). */
    mi_service_init();
    mi_scheduler_init(10, mi_schd_event_handler, &config);
    mi_scheduler_start(SYS_KEY_RESTORE);

    
    lock_init_t lock_config;
    lock_config.opcode_handler = ble_lock_ops_handler;
    lock_service_init(&lock_config);

    // Start execution.
    app_timer_start(poll_timer, APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER), NULL);
    advertising_start();

    // Enter main loop.
    for (;;) {
        // Scan keyboard if needed.
        if (need_kbd_input) {
            if (pair_code_num < PAIRCODE_NUMS) {
                pair_code_num += scan_keyboard(pair_code + pair_code_num, PAIRCODE_NUMS - pair_code_num);
            }
            if (pair_code_num == PAIRCODE_NUMS) {
                pair_code_num = 0;
                need_kbd_input = false;
                mi_input_oob(pair_code, sizeof(pair_code));
            }
        }

        // Process secure auth procedure.
#if (MI_SCHD_PROCESS_IN_MAIN_LOOP==1)
        mi_schd_process();
#endif

        // Print logs info in RTT, then enter sleep mode.
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    error_info_t* pinfo = (error_info_t *)info;
    char * const p_str = (void *)(pinfo->p_file_name);
    NRF_LOG_ERROR(" Oops ! ");

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
            NRF_LOG_RAW_INFO("ERROR at %s : %d\n", nrf_log_push(p_str),
                                                   pinfo->line_num  );
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            NRF_LOG_RAW_INFO("ERRNO %d at %s : %d\n", pinfo->err_code,
                                                      nrf_log_push(p_str),
                                                      pinfo->line_num);
            break;
    }

    NRF_LOG_FINAL_FLUSH();

    // On assert, the system can only recover with a reset.
#ifndef DEBUG
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}

/**
 * @}
 */
