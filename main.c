/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <time.h>
#include "SEGGER_RTT.h"
#include "mible_log.h"
#include "mible_api.h"
#include "nRF5_evt.h"
#include "common/mible_beacon.h"
#include "standard_auth/mible_standard_auth.h"
#include "mijia_profiles/mi_service_server.h"
#include "mijia_profiles/stdio_service_server.h"
#include "mi_config.h"

#include "mible_beacon_internal.h"
#include "mible_beacon.h"

#define DEVICE_NAME                     "stand_demo"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Xiaomi Inc."                           /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS) //zl 老 MSEC_TO_UNITS(100, UNIT_1_25_MS)	        /**< Minimum acceptable connection interval (15 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS) //zl 老 MSEC_TO_UNITS(200, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (30 ms). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(900,  UNIT_10_MS) //zl 老 MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (0.9 s). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000)		//zl 老 APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(60000)		//zl 老 APP_TIMER_TICKS(30000)               /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


#define ADV_FAST_PAIR_TIME 5000
#define ADV_OBJECTS_TIME 500

static void* button_timer;
static void* fastpair_timer;

//static uint8_t counter = 0;
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
APP_TIMER_DEF(m_poll_timer);
APP_TIMER_DEF(m_solicited_timer);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static void advertising_init(bool need_bind_confirm);
static void advertising_start(void);
static void poll_timer_handler(void * p_context);
static void solicited_timeout(void * p_context);
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
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
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
    err_code = app_timer_create(&m_poll_timer, APP_TIMER_MODE_REPEATED, poll_timer_handler);
    MI_ERR_CHECK(err_code);

    err_code = app_timer_create(&m_solicited_timer, APP_TIMER_MODE_SINGLE_SHOT, solicited_timeout);
    MI_ERR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode); //安全模式设置，1：无安全要求

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL; //0.5sec 连接间隔时间，指定一个最大值和最小值，以供Master 建立连接
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;     //从机潜伏，允许设备跳过的最大连接次数，为0，能快速收到Master发送过来的数据
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;  //监督超时时间，超时没有收到数据则认为连接断开

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
	//一从 对 多主 模式初始化
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
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
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

//只搭建了一个框架，还没有建立蓝牙服务。
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

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
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
    ret_code_t err_code = app_timer_start(m_poll_timer, APP_TIMER_TICKS(60000), NULL);
    MI_ERR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
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




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
			NRF_LOG_INFO("[zl] Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
			NRF_LOG_INFO("[zl] Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE:
            NRF_LOG_INFO("PHY update tx %dM rx %dM",
                p_ble_evt->evt.gap_evt.params.phy_update.tx_phy,
                p_ble_evt->evt.gap_evt.params.phy_update.rx_phy);
            break;

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

    mible_on_ble_evt(p_ble_evt);
}

void mible_gap_address_set(void)
{
    uint32_t errno;
    ble_gap_addr_t gap_addr;	
    #if (NRF_SD_BLE_API_VERSION >= 3)
	    errno = sd_ble_gap_addr_get(&gap_addr);
	    gap_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
	    gap_addr.addr_id_peer = 0;
	    //gap_addr.addr[5] &= 0x7F; //Public Device Address
        errno = sd_ble_gap_addr_set(&gap_addr);
    #else
        errno = sd_ble_gap_address_get(&gap_addr);
    #endif
    
    MI_LOG_DEBUG("\nmible_gap_address_set %d\n", errno);
    return;// err_code_convert(errno);
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
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
	
	// This is a demo. A real product must has a valid public address.
    mible_gap_address_set();  //zl
}

static void enqueue_new_objs()
{
    static int8_t  battery;

    battery = battery < 100 ? battery + 1 : 0;
    mibeacon_obj_enque(MI_STA_BATTERY, sizeof(battery), &battery, 0);
}

static void ble_fastpair_event(void)
{
    mible_timer_stop(fastpair_timer);
    
    MI_LOG_INFO("ble_fastpair advertising init...\n");
    mibeacon_frame_ctrl_t frame_ctrl = {
        .is_encrypt = 0,
        .mac_include = 1,
        .cap_include = 1,
        .obj_include = 1,
        .solicite = 0,
        .version = 0x03,
    };
    mibeacon_capability_t cap = {.connectable = 1,
                                 .encryptable = 1,
                                 .bondAbility = 1};

    mible_addr_t dev_mac;
    mible_gap_address_get(dev_mac);
                                                                
    mibeacon_obj_t fastpair_obj = {.type = MI_EVT_SIMPLE_PAIR,
                                   .len = 2,
                                   .val[0] = 0x01,
                                   .val[1] = 0x10,};
    
    mibeacon_config_t mibeacon_cfg = {
        .frame_ctrl = frame_ctrl,
        .pid = PRODUCT_ID,
        .p_mac = (mible_addr_t*)dev_mac, 
        .p_capability = &cap,
        .p_obj = (mibeacon_obj_t*)&fastpair_obj,
        .obj_num = 1,
    };
    
    uint8_t service_data[31];
    uint8_t service_data_len = 0;
    
//    if(MI_SUCCESS != mible_service_data_set(&mibeacon_cfg, service_data, &service_data_len)){
//        MI_LOG_ERROR("mibeacon_data_set failed. \r\n");
//        return;
//    }

	if(MI_SUCCESS != fastpair_mible_service_data_set(&mibeacon_cfg, service_data, &service_data_len)){
        MI_LOG_ERROR("mibeacon_data_set failed. \r\n");
        return;
    }
	
	
    uint8_t adv_data[23]={0};
    uint8_t adv_len=0;
    // add flags
    adv_data[0] = 0x02;
    adv_data[1] = 0x01;
    adv_data[2] = 0x06;
    
    memcpy(adv_data+3, service_data, service_data_len);
    adv_len = service_data_len + 3;
    
    mible_gap_adv_data_set(adv_data,adv_len,NULL,0);
    
    MI_LOG_INFO("fastpair adv data:");
    MI_LOG_HEXDUMP(adv_data, adv_len);
    MI_PRINTF("\r\n");
        
    mible_timer_start(fastpair_timer, ADV_FAST_PAIR_TIME, NULL);
        
    return;
}

static void push_key_mibeacon(uint8_t key)
{
    mible_timer_stop(button_timer);
    
    mibeacon_obj_t pushObj = {.type = MI_STA_BUTTON,
                              .len = 3,
                              .val[0] = 0x00,
                              .val[1] = 0x00,
                              .val[2] = key,};
            
    uint32_t errno;

    uint8_t adv_data[31];
    uint8_t adv_dlen = 0;

    mibeacon_config_t beacon_cfg = {
            .frame_ctrl.version = 5,
            .frame_ctrl.is_encrypt = 1,
            .pid = PRODUCT_ID, //156, //beacon_nonce.pid,
            .p_obj = &pushObj,
            .obj_num = 1,
        };

    //mible_manu_data_set(&beacon_cfg, adv_data, &adv_dlen);
    fastpair_mible_service_data_set(&beacon_cfg, adv_data, &adv_dlen);
	//mible_service_data_set(&beacon_cfg, adv_data, &adv_dlen);
	MI_LOG_INFO("mibeacon event adv ...\n");
    MI_LOG_HEXDUMP(adv_data, adv_dlen);
    
    errno = mible_gap_adv_data_set( adv_data, adv_dlen, NULL, 0);
    MI_ERR_CHECK(errno);

    mible_timer_start(button_timer, ADV_OBJECTS_TIME, NULL);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
	//char buf[3] = {0x00,0x00,0x00};
    switch (event)
    {
        case BSP_EVENT_SLEEP:
		MI_LOG_INFO("\n BSP_EVENT_SLEEP\n"); 
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
		MI_LOG_INFO("\n BSP_EVENT_DISCONNECT\n"); 
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_KEY_0:
		MI_LOG_INFO("\n BSP_EVENT_KEY_0\n");    
            //app_timer_start(m_solicited_timer, APP_TIMER_TICKS(5000), NULL);
            //advertising_init(1);
			mi_scheduler_start(SYS_KEY_DELETE);
			advertising_init(1);
            break;

        case BSP_EVENT_KEY_1:
//            if (get_mi_reg_stat()) {
//                enqueue_new_objs();
//            }
		
		MI_LOG_INFO("\n BSP_EVENT_KEY_1\n");    
                    //push_key_mibeacon(1);
                    ble_fastpair_event();
            break;

        case BSP_EVENT_KEY_2:
			MI_LOG_INFO("\n BSP_EVENT_KEY_2\n");    
//            mi_scheduler_start(SYS_KEY_DELETE);
//            advertising_init(0);
			push_key_mibeacon(2);
//			buf[2] = 0x02;
//			mibeacon_obj_enque(MI_STA_BUTTON, 3, buf, 1);
            break;
		case BSP_EVENT_KEY_3:
			MI_LOG_INFO("\n BSP_EVENT_KEY_3\n");    
			push_key_mibeacon(3);
//			buf[2] = 0x03;
//			mibeacon_obj_enque(MI_STA_BUTTON, 3, buf, 1);
            break;
        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(bool solicite_bind)
{
//	solicite_bind = 1;
//    MI_LOG_INFO("advertising init...\n");

//    uint8_t user_data[31], user_dlen;
//    user_data[0] = 1 + strlen(DEVICE_NAME);
//    user_data[1] = 9;  // complete local name
//    strcpy((char*)&user_data[2], DEVICE_NAME);
//    user_dlen = 2 + strlen(DEVICE_NAME);
//    if (MI_SUCCESS != mibeacon_adv_data_set(solicite_bind, 0, user_data, user_dlen)) {
//        MI_LOG_ERROR("encode mibeacon data failed. \r\n");
//    }
//    return;
/////////////////////////////////////////////////////////////////////////////////////////
	MI_LOG_INFO("advertising init...\n");
    mibeacon_frame_ctrl_t frame_ctrl = {
        .is_encrypt = 0,
        .mac_include = 1,
        .cap_include = 1,
        .obj_include = 0,
        .solicite = 0,
        .version = 0x05,
    };
    mibeacon_capability_t cap = {.connectable = 1,
                                 .encryptable = 1,
                                 .bondAbility = 1};

    mible_addr_t dev_mac;
    mible_gap_address_get(dev_mac);
    
    mibeacon_config_t mibeacon_cfg = {
        .frame_ctrl = frame_ctrl,
        .pid =PRODUCT_ID,
		//.frame_counter = ((counter + 4) % 512),
        .p_mac = (mible_addr_t*)dev_mac, 
        .p_capability = &cap,
        .p_obj = NULL,
    };
    
//	counter += 4;
//	if(counter >= 504)
//	{
//		counter = 0;
//	}
//	
    uint8_t service_data[31];
    uint8_t service_data_len=0;
    
    if(MI_SUCCESS != fastpair_mible_service_data_set(&mibeacon_cfg, service_data, &service_data_len)){
        MI_LOG_ERROR("mibeacon_data_set failed. \r\n");
        return;
    }
    
    uint8_t adv_data[23] = {0};
    uint8_t adv_len = 0;
    // add flags
    adv_data[0] = 0x02;
    adv_data[1] = 0x01;
    adv_data[2] = 0x06;
    
    memcpy(adv_data+3, service_data, service_data_len);
    adv_len = service_data_len + 3;
    
    mible_gap_adv_data_set(adv_data,adv_len,NULL,0);
    
    MI_LOG_INFO("adv mi service data:");
    MI_LOG_HEXDUMP(adv_data, adv_len);
    MI_PRINTF("\r\n");
    return;	
	
	
}

/**@brief Function for initializing buttons and leds.
 *
 */
static void buttons_leds_init()
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    /* assign BUTTON 1 to set solicite bit in mibeacon, for more details to check bsp_event_handler()*/
    err_code = bsp_event_to_button_action_assign(0,
                                             BSP_BUTTON_ACTION_PUSH,
                                             BSP_EVENT_KEY_0);
    APP_ERROR_CHECK(err_code);

    /* assign BUTTON 2 to initate mibeacon object, for more details to check bsp_event_handler()*/
    err_code = bsp_event_to_button_action_assign(1,
                                             BSP_BUTTON_ACTION_PUSH,
                                             BSP_EVENT_KEY_1);
    APP_ERROR_CHECK(err_code);

    /* assign BUTTON 3 to delete KEYINFO, for more details to check bsp_event_handler()*/
    err_code = bsp_event_to_button_action_assign(2,
                                             BSP_BUTTON_ACTION_LONG_PUSH,
                                             BSP_EVENT_KEY_2);
	/* BUTTON4 */								 
	err_code = bsp_event_to_button_action_assign(3,
                                             BSP_BUTTON_ACTION_LONG_PUSH,
                                             BSP_EVENT_KEY_3);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t errno = mibeacon_adv_start(30);
    MI_ERR_CHECK(errno);
}


static void solicited_timeout(void * p_context)
{
    MI_LOG_WARNING("solicte bit clear.\n");
    advertising_init(0);
}


static void poll_timer_handler(void * p_context)
{
    time_t utc_time = time(NULL);
    MI_LOG_INFO("[zl] %s", ctime(&utc_time));

    // if device has been registered, it will advertise the mibeacon contains object.
    if (get_mi_reg_stat()) {
        enqueue_new_objs();
    }
}

void time_init(struct tm * time_ptr);

static uint8_t qr_code[16] = {
0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xa0,0xaa,0xab,0xac,0xad,0xae,0xaf,
};

void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch (p_event->id) {
    case SCHD_EVT_OOB_REQUEST:
        MI_LOG_INFO("App selected IO cap is 0x%04X\n", p_event->data.IO_capability);
        switch (p_event->data.IO_capability) {
        case 0x0080:
            mi_schd_oob_rsp(qr_code, 16);
            MI_LOG_INFO(MI_LOG_COLOR_GREEN "Please scan QR code label.\n");
            break;

        default:
            MI_LOG_ERROR("Selected IO cap is not supported.\n");
            mible_gap_disconnect(0);
        }
        break;

    case SCHD_EVT_KEY_DEL_SUCC:
		MI_LOG_INFO("[zl] mi_schd_event_handler SCHD_EVT_KEY_DEL_SUCC(%x) \n", SCHD_EVT_KEY_DEL_SUCC);
        // device has been reset, restart adv mibeacon contains IO cap.
        advertising_init(0);
        break;

    default:
        break;
    }
}

void stdio_rx_handler(uint8_t* p, uint8_t l)
{
    int errno;
    /* RX plain text (It has been decrypted) */
    MI_LOG_INFO("RX raw data\n");
    MI_LOG_HEXDUMP(p, l);

    /* TX plain text (It will be encrypted before send out.) */
    errno = stdio_tx(p, l);
    MI_ERR_CHECK(errno);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
	uint8_t ret;
	
    log_init();
    MI_LOG_INFO(RTT_CTRL_CLEAR);
    MI_LOG_INFO("Compiled  %s %s\n", (uint32_t)__DATE__, (uint32_t)__TIME__);
    timers_init();
    buttons_leds_init();
    power_management_init(); //有一个关机的标志位
    ble_stack_init(); 		 //注册蓝牙处理调度事件  ble_evt_handler
    gap_params_init();
    gatt_init();
    services_init();
    conn_params_init(); //连接参数
    time_init(NULL);

    /* <!> mi_scheduler_init() must be called after ble_stack_init(). */
    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    mi_scheduler_start(SYS_KEY_RESTORE);

    mi_service_init();
    stdio_service_init(stdio_rx_handler);

    advertising_init(0);
    
    // Start execution.
    advertising_start();
    application_timers_start();
    

//	mible_status_t ret = mible_timer_create(&button_timer, (mible_handler_t)advertising_init, MIBLE_TIMER_SINGLE_SHOT);
//    if(ret != MI_SUCCESS){
//        MI_LOG_ERROR("button_timer_create failed. code = %x .\r\n",ret);
//    }else{
//        MI_LOG_DEBUG("button_timer_create success. \r\n");
//    }      		

    ret = mible_timer_create(&fastpair_timer, (mible_handler_t)advertising_init, MIBLE_TIMER_SINGLE_SHOT);
    if(ret != MI_SUCCESS){
        MI_LOG_ERROR("fastpair_timer_create failed. code = %x .\r\n",ret);
    }else{
        MI_LOG_DEBUG("fastpair_timer_create success. \r\n");
    }	
	
	ret = mible_timer_create(&button_timer, (mible_handler_t)advertising_init, MIBLE_TIMER_SINGLE_SHOT);
    if(ret != MI_SUCCESS){
        MI_LOG_ERROR("button_timer_create failed. code = %x .\r\n",ret);
    }else{
        MI_LOG_DEBUG("button_timer_create success. \r\n");
    }  
	
	MI_LOG_INFO("[zl] enter main loop!\n");
    // Enter main loop.
	
//	char buf[10];
//	char ret = mible_record_write(0x20, "12345", 6);
//	MI_LOG_INFO("ret = %d\n", ret);
//	ret=mible_record_read(0x20, buf, 6);
//	MI_LOG_INFO("ret = %d buf = %s\n", ret, buf);
//	
		
    for (;;) 
	{
#if (MI_SCHD_PROCESS_IN_MAIN_LOOP==1)
        // Process mi scheduler
		//MI_LOG_INFO("main\n");
        mi_schd_process();
#endif
        // Enter Sleep mode
        idle_state_handle();
    }
}


/**
 * @}
 */
