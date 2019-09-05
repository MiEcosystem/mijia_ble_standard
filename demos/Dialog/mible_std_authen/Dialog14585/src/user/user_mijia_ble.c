#include "user_mijia_ble.h"
#include "mible_api.h"
#include "arch_console.h"
#include "mible_log.h"
#include "arch_console.h"
#include "app_easy_timer.h"

#include "common/mible_beacon_internal.h"
#include "standard_auth/mible_standard_auth.h"
#include "mijia_profiles/mi_service_server.h"
#include "mijia_profiles/stdio_service_server.h"
#include "mi_config.h"

#define DEVICE_NAME                     "stand_demo"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Xiaomi Inc."                           /**< Manufacturer. Will be passed to Device Information Service. */

enum
{
    UNIT_0_625_MS = 625,        /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS  = 1250,       /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS    = 10000       /**< Number of microseconds in 10 milliseconds. */
};

/**@brief Macro for converting milliseconds to ticks.
 *
 * @param[in] TIME          Number of milliseconds to convert.
 * @param[in] RESOLUTION    Unit to be converted to in [us/ticks].
 */
#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

static uint8_t qr_code[16] = {
0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
};

extern mible_status_t mible_service_data_set(mibeacon_config_t const * const config, uint8_t *p_output, uint8_t *p_output_len);
/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(bool solicite_bind)
{
    MI_LOG_INFO("advertising init...\n");
    mibeacon_frame_ctrl_t frame_ctrl = {
        .auth_mode      = 2,
        .version        = 5,
        .solicite       = solicite_bind,
			  .registered     = 0,
    };

    mibeacon_capability_t cap = {.bondAbility = 1};
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
        MI_LOG_ERROR("encode service data failed. \n");
        return;
    }
    adv_len += service_data_len;
    
    MI_LOG_HEXDUMP(adv_data, adv_len);
		
		MI_LOG_HEXDUMP(dev_mac, 6);

    mible_gap_adv_data_set(&(adv_data[3]), adv_len-3, &(adv_data[3]), 0);

    return;
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    mible_gap_adv_param_t adv_param =(mible_gap_adv_param_t){
        .adv_type = MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED, 
        .adv_interval_min = 0x00a0,//MSEC_TO_UNITS(200, UNIT_0_625_MS),
        .adv_interval_max = 0x00b0,//MSEC_TO_UNITS(300, UNIT_0_625_MS),
        .ch_mask = {0},
    };
    uint32_t err_code = mible_gap_adv_start(&adv_param);
    if(MI_SUCCESS != err_code){
        MI_LOG_ERROR("adv failed. %d \n", err_code);	  
    }
		else
		{
		    MI_LOG_INFO("advertising_start OK!");
		}
		return;
}

void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch (p_event->id) {
    case SCHD_EVT_OOB_REQUEST:
        MI_LOG_INFO("App selected IO cap is 0x%04X\n", p_event->data.IO_capability);
        switch (p_event->data.IO_capability) {
        case 0x0080:
            mi_schd_oob_rsp(qr_code, 16);
            MI_LOG_INFO(MI_LOG_COLOR_GREEN "Please scan device QR code.\n");
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

//void stdio_rx_handler(uint8_t* p, uint8_t l)
//{
//    int errno;
//    /* RX plain text (It has been decrypted) */
//    MI_LOG_INFO("RX raw data\n");
//    MI_LOG_HEXDUMP(p, l);

//    /* TX plain text (It will be encrypted before send out.) */
//    errno = stdio_tx(p, l);
//    MI_ERR_CHECK(errno);
//}

void simulation_miserver_test(void)
{
    MI_LOG_INFO("mible standard auth demo\r\n");

    /* <!> mi_scheduler_init() must be called after ble_stack_init(). */
    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    mi_scheduler_start(SYS_KEY_RESTORE);
    
    // Start execution.
    advertising_init(0);
    advertising_start();
}

void mijia_service_init(void)
{
    mi_service_init();
    
    //with the limit of mible_gatts_service_init API, we can only init one service here
    //stdio_service_init(stdio_rx_handler);
}
