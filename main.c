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

#include "bsp_trace.h"

/* Mijia BLE API and Middleware */
#include "efr32_api.h"
#include "mible_api.h"
#include "mible_trace.h"
#include "common/mible_beacon.h"
#include "standard_auth/mible_standard_auth.h"
#include "mijia_profiles/mi_service_server.h"
#include "mijia_profiles/stdio_service_server.h"

#undef  MI_LOG_MODULE_NAME
#define MI_LOG_MODULE_NAME __FILE__
#include "mible_log.h"

#include "mi_config.h"

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS                1
#endif

#ifndef MAX_ADVERTISERS
#define MAX_ADVERTISERS                2
#endif

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))
#define MS_2_TIMERTICK(ms)             ((TIMER_CLK_FREQ * (uint32)(ms)) / 1000)
#define SEC_2_TIMERTICK(sec)           ((TIMER_CLK_FREQ * (sec)))

#define TIMER_ID_OBJ_PERIOD_ADV        11
#define TIMER_ID_CLEAR_BIND_CFM        12

/// Number of ticks after which press is considered to be long (1s)
#define LONG_PRESS_TIME_TICKS           (32768)
#define EXT_SIGNAL_PB0_SHORT_PRESS      (1<<0)
#define EXT_SIGNAL_PB0_LONG_PRESS       (1<<1)
#define EXT_SIGNAL_PB1_SHORT_PRESS      (1<<2)

static uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* Bluetooth stack configuration parameters (see "UG136: Silicon Labs Bluetooth C Application Developer's Guide" for details on each parameter) */
static gecko_configuration_t config = {
  .config_flags = 0,                                   /* Check flag options from UG136 */
#if defined(FEATURE_LFXO)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,        /* Sleep is enabled */
#else
  .sleep.flags = 0,
#endif // LFXO
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
  .max_timers = 8,
};

static const iic_config_t msc_iic_config = {
        .scl_pin = BSP_I2C0_SCL_PIN,
        .scl_port = BSP_I2C0_SCL_PORT,
        .scl_extra_conf = BSP_I2C0_SCL_LOC,
        .sda_pin = BSP_I2C0_SDA_PIN,
        .sda_port = BSP_I2C0_SDA_PORT,
        .sda_extra_conf = BSP_I2C0_SDA_LOC,
        .freq = HAVE_MSC==1 ? IIC_100K : IIC_400K,
};

#define PAIRCODE_NUMS 6
bool need_kbd_input;
uint8_t pair_code_num;
uint8_t pair_code[PAIRCODE_NUMS];
/// button press timestamp for very long/long/short Push Button 0 press detection
static uint32 pb0_press, pb1_press;

extern void time_init(struct tm * time_ptr);
static void advertising_init(uint8_t solicite_bind);
static void advertising_start(void);

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

    if (pin == BSP_BUTTON1_PIN) {
        if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON1_PIN) == 0) {
            // PB0 pressed - record RTCC timestamp
            pb1_press = RTCC_CounterGet();
        } else {
            // PB0 released - check if it was short or long press
            t_diff = RTCC_CounterGet() - pb1_press;
            if (t_diff < LONG_PRESS_TIME_TICKS) {
                gecko_external_signal(EXT_SIGNAL_PB1_SHORT_PRESS);
            } else {

            }
        }
    }
}


void button_init(void)
{
    // configure pushbutton PB0 and PB1 as inputs, with pull-up enabled
    GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
    GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);

    GPIOINT_Init();

    /* configure interrupt for PB0 and PB1, both falling and rising edges */
    GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);
    GPIO_ExtIntConfig(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, BSP_BUTTON1_PIN, true, true, true);

    /* register the callback function that is invoked when interrupt occurs */
    GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, gpio_irq_handler);
    GPIOINT_CallbackRegister(BSP_BUTTON1_PIN, gpio_irq_handler);
}


static void enqueue_new_objs()
{
    // get your battery
    int8_t  battery = 100;
    mibeacon_obj_enque(MI_STA_BATTERY, sizeof(battery), &battery);
}


static void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch(p_event->id) {
    case SCHD_EVT_OOB_REQUEST:
        MI_LOG_INFO(MI_LOG_COLOR_GREEN "Please scan QR code. \n");
        const uint8_t qr_code[16] = {
                0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xa0,0xaa,0xab,0xac,0xad,0xae,0xaf,
        };
        mi_schd_oob_rsp(qr_code, 16);
        break;

    case SCHD_EVT_REG_SUCCESS:
        // start periodic advertise objects.
        gecko_cmd_hardware_set_soft_timer(SEC_2_TIMERTICK(600), TIMER_ID_OBJ_PERIOD_ADV, 0);
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

static void process_system_boot(struct gecko_cmd_packet *evt)
{

    struct gecko_msg_system_boot_evt_t boot_info = evt->data.evt_system_boot;
    MI_LOG_INFO("system stack %d.%0d.%0d-%d, heap %d bytes\n", boot_info.major, boot_info.minor, boot_info.patch, boot_info.build,sizeof(bluetooth_stack_heap));

    gecko_cmd_system_set_tx_power(0);
    /* Start general advertising and enable connections. */
    advertising_init(0);
    advertising_start();

    mi_service_init();
    stdio_service_init(stdio_rx_handler);

    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    mi_scheduler_start(SYS_KEY_RESTORE);
}


static void process_softtimer(struct gecko_cmd_packet *evt)
{
    switch (evt->data.evt_hardware_soft_timer.handle) {
    case TIMER_ID_OBJ_PERIOD_ADV:
        MI_LOG_WARNING("systime %d\n", gecko_cmd_hardware_get_time()->seconds);
        enqueue_new_objs();
        break;

    case TIMER_ID_CLEAR_BIND_CFM:
        advertising_init(0);
        break;
    }
}


static void process_external_signal(struct gecko_cmd_packet *evt)
{
    if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_SHORT_PRESS) {
        MI_LOG_DEBUG("Set bind confirm bit in mibeacon.\n");
        advertising_init(1);
        gecko_cmd_hardware_set_soft_timer(SEC_2_TIMERTICK(10), TIMER_ID_CLEAR_BIND_CFM, 1);
    }

    if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_LONG_PRESS) {
        MI_LOG_DEBUG("Factory reset.\n");
        mi_scheduler_start(SYS_KEY_DELETE);
        gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_OBJ_PERIOD_ADV, 0);
    }

    if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB1_SHORT_PRESS) {
        // TODO
    }
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
        process_softtimer(evt);
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
    BSP_TraceSwoSetup();

    MI_LOG_INFO(RTT_CTRL_CLEAR"\n");
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
    time_init(NULL);

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

        /* Scan keyboard */
        if (need_kbd_input) {
            if (pair_code_num < PAIRCODE_NUMS) {
                pair_code_num += scan_keyboard(pair_code + pair_code_num, PAIRCODE_NUMS - pair_code_num);
            }
            if (pair_code_num == PAIRCODE_NUMS) {
                pair_code_num = 0;
                need_kbd_input = false;
                mi_schd_oob_rsp(pair_code, sizeof(pair_code));
            }
        }

        /* Enter low power mode */
        gecko_sleep_for_ms(gecko_can_sleep_ms());
    }
}


static void advertising_init(uint8_t solicite_bind)
{
    MI_LOG_INFO("advertising init...\n");

    mibeacon_frame_ctrl_t frame_ctrl = {
            .auth_mode    = 2,
            .solicite     = solicite_bind
    };

    mibeacon_capability_t cap = {
            .bondAbility = 1
    };

    mible_addr_t dev_mac;
    mible_gap_address_get(dev_mac);

    mibeacon_cap_sub_io_t io = {
            .out_image   = 1,
    };

    mibeacon_config_t mibeacon_cfg = {
            .frame_ctrl = frame_ctrl,
            .pid = PRODUCT_ID,
            .p_mac = &dev_mac,
            .p_capability = &cap,
//            .p_cap_sub_IO = &io,
    };

    uint8_t service_data[31];
    uint8_t service_data_len = 0;

    if(MI_SUCCESS != mible_service_data_set(&mibeacon_cfg, service_data, &service_data_len)){
        MI_LOG_ERROR("mibeacon_data_set failed. \r\n");
        return;
    }

    uint8_t adv_data[31] = {2, 1, 6};
    uint8_t adv_len = 3;

    memcpy(adv_data+3, service_data, service_data_len);
    adv_len += service_data_len;

    MI_LOG_INFO("adv_data:\n");
    MI_LOG_HEXDUMP(adv_data, adv_len);

    mible_gap_adv_data_set(adv_data, adv_len, adv_data, 0);
    return;
}


static void advertising_start(void)
{
    MI_LOG_INFO("advertising start...\n");
    mible_gap_adv_param_t adv_param =(mible_gap_adv_param_t){
                .adv_type = MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED,
                .adv_interval_min = MSEC_TO_UNITS(200, 625),
                .adv_interval_max = MSEC_TO_UNITS(200, 625),
    };

    if(MI_SUCCESS != mible_gap_adv_start(&adv_param)){
        MI_LOG_ERROR("mible_gap_adv_start failed. \r\n");
        return;
    }
}

