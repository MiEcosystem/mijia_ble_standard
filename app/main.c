/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/
#include <time.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_rtcc.h"
#include "gpiointerrupt.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

//#include "bsp_trace.h"

/* Mijia BLE API and Middleware */
#include "efr32_api.h"
#include "mible_api.h"
#include "mible_trace.h"
#include "common/mible_beacon.h"
#include "standard_auth/mible_standard_auth.h"
#include "mijia_profiles/mi_service_server.h"
#include "mijia_profiles/stdio_service_server.h"

#include "ble_spec/gatt_spec.h"

#include "third_party/SEGGER_RTT/SEGGER_RTT.h"

#undef  MI_LOG_MODULE_NAME
#define MI_LOG_MODULE_NAME __FILE__
#include "mible_log.h"

#include "mi_config.h"

#include "key.h"

#include "miio_user_api.h"

#define TEST_GATT_SPEC  1

#define DEVICE_NAME                    "stand_demo666"
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS                1
#endif

#ifndef MAX_ADVERTISERS
#define MAX_ADVERTISERS                2
#endif

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

/// Number of ticks after which press is considered to be long (3s)
#define LONG_PRESS_TIME_TICKS           (32768*3)

#define SOFT_TIMER_TIMEOUT  60

#define BIND_INIT            0
#define BIND_CONFIRMED       1


static uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* Bluetooth stack configuration parameters (see "UG136: Silicon Labs Bluetooth C Application Developer's Guide" for details on each parameter) */
static gecko_configuration_t config = {
  .config_flags = 0,                                   /* Check flag options from UG136 */
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,        /* Sleep is enabled */
  .sleep.flags = 0,
  .bluetooth.max_connections = MAX_CONNECTIONS,        /* Maximum number of simultaneous connections */
  .bluetooth.max_advertisers = MAX_ADVERTISERS,        /* Maximum number of advertisement sets */
  .bluetooth.heap = bluetooth_stack_heap,              /* Bluetooth stack memory for connection management */
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap), /* Bluetooth stack memory for connection management */
  .bluetooth.sleep_clock_accuracy = 100,               /* Accuracy of the Low Frequency Crystal Oscillator in ppm. *
                                                        * Do not modify if you are using a module                  */
  .gattdb = &bg_gattdb_data,                           /* Pointer to GATT database */
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1,                               /* Set this to be a valid PA config */
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT,               /* Configure PA input to VBAT */
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,               /* Configure PA input to DCDC */
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,                 /* Enable antenna configuration. */
  .rf.antenna = GECKO_RF_ANTENNA,                      /* Select antenna path! */
  .max_timers = 10,
};

/// button press timestamp for very long/long/short Push Button 0 press detection
static uint32 pb0_press;

static void * mibeacon_period_adv_timer = NULL;
static void * mibeacon_bind_confirm_timer = NULL;

//extern void time_init(struct tm * time_ptr);

//static void spec_recv_event_handler(mible_spec_command_t evt, uint16_t tid,
//	uint8_t p_num, spec_property_t *param);

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


void gpio_irq_handler(uint8_t pin)
{
    uint32_t t_diff;

    if (pin == BSP_BUTTON0_PIN) {
        if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0) {
            // PB0 pressed - record RTCC timestamp
            pb0_press = RTCC_CounterGet();
        } else {
            // PB0 released - check if it was short or long press
            t_diff = RTCC_CounterGet() - pb0_press;
            if (t_diff < LONG_PRESS_TIME_TICKS) {
                gecko_external_signal(EXT_SIGNAL_PB0_SHORT_PRESS);
            } else {
                gecko_external_signal(EXT_SIGNAL_PB0_LONG_PRESS);
            }
        }
    }
}

static void enqueue_new_objs(void)
{
	MI_LOG_INFO("\n enqueue_new_objs \n");
#if 0
	//Battery ---- siid 6
    static int8_t  battery;
    battery = battery < 100 ? battery + 1 : 0;

    miio_ble_property_changed(3, 1101, property_value_new_float(battery), 0);
#else
    //static uint16_t tem = 50;
    //miio_ble_property_changed(2, 1001, property_value_new_float(tem), 0);
    //tem = tem < 1000 ? tem + 50 : 50;

    //static uint16_t hum = 50;
    //miio_ble_property_changed(2, 1002, property_value_new_uchar(hum), 0);
    //hum = hum < 1000 ? hum + 50 : 50;

    static uint8_t buff[3] = {0};
    static uint32_t time = 0;
    miio_ble_obj_enque(0x0003, 0, NULL, 0);
    miio_ble_obj_enque(0x0004, 0, NULL, 1);
    miio_ble_obj_enque(0x000f, 3, buff, 0);
    miio_ble_obj_enque(0x1017, 4, &time, 1);
    time += 2;
#endif
}

#include "standard_auth/mible_standard_auth.h"
static void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch(p_event->id) {
    case SCHD_EVT_OOB_REQUEST:
        MI_LOG_INFO(MI_LOG_COLOR_GREEN " scan QR code. \n");
        const uint8_t qr_code[16] = {
                0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xa0,0xaa,0xab,0xac,0xad,0xae,0xaf,
        };
        mi_schd_oob_rsp(qr_code, 16);
        break;

    case SCHD_EVT_REG_SUCCESS:
    	miio_system_reboot();
        break;

    case SCHD_EVT_KEY_DEL_SUCC:
    	miio_system_reboot();
        break;

    default:
        break;
    }
}

static void stdio_rx_handler(uint8_t* data, uint8_t len)
{
    int errno;
    /* RX plain text (It has been decrypted.) */
    MI_LOG_INFO("RX plain data\n");
    MI_LOG_HEXDUMP(data, len);

    /* TX plain text (It will be encrypted before send out.) */
    errno = stdio_tx(data, len);
    MI_ERR_CHECK(errno);
}

static void mibeacon_period_adv_handler(void * p_context)
{
	enqueue_new_objs();
}

static void mibeacon_bind_confirm_handler(void * p_context)
{
	MI_LOG_INFO("clear bind confirm bit !\n");
	miio_ble_user_adv_init(BIND_INIT);
	miio_timer_stop(mibeacon_bind_confirm_timer);
}

#if TEST_GATT_SPEC
void on_property_set(property_operation_t *o)
{
    MI_LOG_INFO("[on_property_set] siid %d, piid %d\n", o->siid, o->piid);

    if (o->value == NULL)
    {
        MI_LOG_ERROR("value is NULL\n");
        return;
    }
    o->code = 0;
}
void on_property_get(property_operation_t *o)
{
    MI_LOG_INFO("[on_property_get] siid %d, piid %d\n", o->siid, o->piid);
    o->value = property_value_new_float(888.88);
}
void on_action_invoke(action_operation_t *o)
{
    MI_LOG_INFO("[on_action_invoke] siid %d, aiid %d\n", o->siid, o->aiid);
    o->code = 0;
}
#endif
static void process_system_boot(struct gecko_cmd_packet *evt)
{
    struct gecko_msg_system_boot_evt_t boot_info = evt->data.evt_system_boot;
    MI_LOG_INFO("system stack %d.%0d.%0d-%d, heap %d bytes\n", boot_info.major, boot_info.minor, boot_info.patch, boot_info.build,sizeof(bluetooth_stack_heap));

    gecko_cmd_system_set_tx_power(0);

    mi_service_init();
    stdio_service_init(stdio_rx_handler);

    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    mi_scheduler_start(SYS_KEY_RESTORE);

    mible_gap_scan_stop();

    /* Start general advertising and enable connections. */
    miio_ble_user_adv_init(BIND_INIT);

    miio_timer_create(&mibeacon_bind_confirm_timer, mibeacon_bind_confirm_handler, MIBLE_TIMER_SINGLE_SHOT);

    if(miio_ble_get_registered_state())
    {
    	MI_LOG_INFO("reg 1\n");
    	//miio_ble_user_adv_stop();
    	miio_ble_user_adv_start(500);

        // start periodic advertise objects.
    	miio_timer_create(&mibeacon_period_adv_timer, mibeacon_period_adv_handler, MIBLE_TIMER_REPEATED);
    	mible_timer_start(mibeacon_period_adv_timer, 60*1000*2, NULL);
    }
    else
    {
    	MI_LOG_INFO("reg 0\n");
    	miio_ble_user_adv_start(500);
    	mibeacon_set_adv_timeout(1000*60*30);
    }

#if TEST_GATT_SPEC
    miio_gatt_spec_init(on_property_set, on_property_get, on_action_invoke, 0);
#endif

}

void gatt_prop()
{
    miio_gatt_properties_changed(1, 2, property_value_new_boolean(1));
    miio_gatt_properties_changed(2, 4, property_value_new_string("properties_changed test 2.4!!!"));
    miio_gatt_properties_changed(3, 6, property_value_new_float(123.456));
}
void gatt_event()
{
    miio_gatt_event_occurred(6, 1, NULL);

    arguments_t *newArgs = arguments_new();
    newArgs->size = 2;
    newArgs->arguments[0].piid = 1;
    newArgs->arguments[0].value = property_value_new_ulong(2);
    newArgs->arguments[1].piid = 2;
    newArgs->arguments[1].value = property_value_new_longlong(-99999);
    miio_gatt_event_occurred(6, 2, newArgs);
}
static void process_external_signal(struct gecko_cmd_packet *evt)
{
    if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_SHORT_PRESS) {
        if(get_mi_reg_stat()) {
            //enqueue_new_objs();
        	//miio_ble_user_adv_start(500);
        	//gatt_prop();
        } else {
            MI_LOG_DEBUG("Set bind confirm bit in mibeacon.\n");
            miio_ble_user_adv_init(BIND_CONFIRMED);
            miio_timer_start(mibeacon_bind_confirm_timer, 10*1000, NULL);
        }
    }

    if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_LONG_PRESS) {
        MI_LOG_DEBUG("Factory reset.\n");
        miio_system_restore();
    	//gatt_event();
    }

    //if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB1_SHORT_PRESS) {
        // TODO
    //}
}

static void user_stack_event_handler(struct gecko_cmd_packet* evt)
{
    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
    /* This boot event is generated when the system boots up after reset.
     * Do not call any stack commands before receiving the boot event.
     * Here the system is set to start advertising immediately after boot procedure. */
    case gecko_evt_system_boot_id:
        process_system_boot(evt);
        break;

    case gecko_evt_hardware_soft_timer_id:
        //process_softtimer(evt);
        break;

    case gecko_evt_system_external_signal_id:
        process_external_signal(evt);
        break;

    default:
        break;
    }
}

int main()
{
    // Initialize device
    initMcu();
    // Initialize board
    initBoard();
    // Initialize application
    initApp();

    // Setup SWD for code correlation
    //BSP_TraceSwoSetup();

    MI_LOG_INFO("%s \n", RTT_CTRL_CLEAR);
    MI_LOG_INFO("Compiled %s %s\n", __DATE__, __TIME__);
    MI_LOG_INFO("system clock %d Hz\n", SystemCoreClockGet());

    // Initialize stack
    gecko_stack_init(&config);
    gecko_bgapi_class_system_init();
    gecko_bgapi_class_le_gap_init();
    gecko_bgapi_class_le_connection_init();
    gecko_bgapi_class_gatt_server_init();
    gecko_bgapi_class_hardware_init();
    gecko_bgapi_class_flash_init();

    button_init();
    //time_init(NULL);


    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    while (1) {
        /* Process stack event */
        do {
            evt = gecko_peek_event();
            if (evt != NULL) {
                mible_stack_event_handler(evt);

                /* customized event handler can be added here. */
                user_stack_event_handler(evt);
            }
        } while (evt != NULL);

        /* Process mi scheduler */
        mi_schd_process();

#if BSP_CLK_LFXO_PRESENT
        /* Enter low power mode */
        gecko_sleep_for_ms(gecko_can_sleep_ms());
#endif
    }
}

