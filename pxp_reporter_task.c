/**
 ****************************************************************************************
 *
 * @file pxp_reporter_task.c
 *
 * @brief PXP profile application implementation
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdbool.h>
#include <string.h>
#include "osal.h"
#include "ble_att.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_l2cap.h"
#include "sdk_list.h"
#include "bas.h"
#include "ias.h"
#include "lls.h"
#include "tps.h"

#include "sys_power_mgr.h"
#include "sys_socf.h"
#include "ad_nvparam.h"
#include "ad_gpadc.h"
#include "hw_gpio.h"


#include "sys_watchdog.h"
#include "app_nvparam.h"
#include "pxp_reporter_config.h"
#include "platform_devices.h"

#if dg_configSUOTA_SUPPORT
#include "dis.h"
#include "dlg_suota.h"
#include "sw_version.h"
#endif

#include "mi_config.h"
#include "mible_port.h"
#include "mesh_auth/mible_mesh_auth.h"


#define MI_LOG_MODULE_NAME "MI APP"
#include "mible_log.h"





extern void mitimer_callback(uint8_t index);
extern void gap_evt_dispatch(ble_evt_hdr_t *hdr);
extern void gatts_evt_dispatch(ble_evt_hdr_t *hdr);
extern uint32_t mi_service_init();
extern void advertising_init(void);

//typedef void (*mi_schd_event_handler_t)(schd_evt_t * p_event);
//extern uint32_t mi_scheduler_init(uint32_t interval, mi_schd_event_handler_t handler,
//    mible_libs_config_t *p_config);

/*
 * Notification bits reservation
 *
 * Bit #0 is always assigned to BLE event queue notification.
 */
#define TIMER0_NOTIF (1 << 1)
#define TIMER1_NOTIF (1 << 2)
#define TIMER2_NOTIF (1 << 3)
#define TIMER3_NOTIF (1 << 4)

PRIVILEGED_DATA OS_TASK app_task;


static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
        gap_evt_dispatch((ble_evt_hdr_t *)evt);
}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
        gap_evt_dispatch((ble_evt_hdr_t *)evt);
}

static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{

        return;
}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
        ble_gap_pair_reply(evt->conn_idx, true, evt->bond);
}

static void handle_evt_gap_pair_completed(ble_evt_gap_pair_completed_t *evt)
{
        return;
}

static void handle_evt_gap_sec_level_changed(ble_evt_gap_sec_level_changed_t *evt)
{
        return;
}

static void handle_evt_gatts_read_request(ble_evt_gatts_read_req_t * evt)
{
        gatts_evt_dispatch((ble_evt_hdr_t *)evt);
}

static void  handle_evt_gatts_write_request(ble_evt_gatts_write_req_t * evt)
{
        gatts_evt_dispatch((ble_evt_hdr_t *)evt);
        ble_gatts_write_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK);

}

uint32_t timer_index  = 0;
void mi_timer_handler(OS_TIMER timer)
{
        timer_index = (uint32_t)OS_TIMER_GET_TIMER_ID(timer);

        if (timer_index >= 4)
                OS_ASSERT(0);

        OS_TASK_NOTIFY(app_task, (1 << (timer_index + 1)), OS_NOTIFY_SET_BITS);
}


void dialog_hex_dump(uint8_t *base_addr,uint8_t bytes )
{
        int i;
        for(i=0;i<bytes;i++){
        if(i == 0)
            printf("0x%02x",base_addr[i]);
        else
            printf("%02x",base_addr[i]);
        }
}


/*

void mi_schd_event_handler(schd_evt_t *p_event)
{
        MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);

        return;
}
*/

/*
uint8_t tmpkey[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
uint8_t plaintext[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
uint8_t outtext[16];
*/

void pxp_reporter_task(void *params)
{
        int8_t wdog_id;

        /* Register pxp_reporter_task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);

        printf("app task started\n");
        fflush(stdout);

        //mible_aes128_encrypt(tmpkey, plaintext, 16, outtext);

        appl_storage_init();

        /* Start BLE device as peripheral */
        ble_peripheral_start();

        /* Register task to BLE framework to receive BLE event notifications */
        ble_register_app();

        /* Set maximum allowed MTU to increase SUOTA throughput */
        ble_gap_mtu_size_set(512);

        ble_gap_device_name_set("MI SERVICE", ATT_PERM_READ);

        //mi_mesh_otp_program_simulation();

        app_task = OS_GET_CURRENT_TASK();

        //mi_service_init();
        //mi_scheduler_init(10, mi_schd_event_handler, NULL);
        //mi_scheduler_start(SYS_KEY_RESTORE);
        //advertising_init();

        mijia_service_init();

        simulation_miserver_test();

        for (;;) {
                OS_BASE_TYPE ret __attribute__((unused));
                uint32_t notif;

                /* Notify watchdog on each loop */
                sys_watchdog_notify(wdog_id);

                /* Suspend watchdog while blocking on OS_TASK_NOTIFY_WAIT() */
                sys_watchdog_suspend(wdog_id);

                /*
                 * Wait on any of the notification bits, then clear them all
                 */
                ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
                /* Blocks forever waiting for the task notification. Therefore, the return value must
                 * always be OS_OK
                 */
                OS_ASSERT(ret == OS_OK);

                /* Resume watchdog */
                sys_watchdog_notify_and_resume(wdog_id);

                /* Notified from BLE Manager? */
                if (notif & BLE_APP_NOTIFY_MASK) {
                        ble_evt_hdr_t *hdr;

                        hdr = ble_get_event(false);
                        if (!hdr) {
                                goto no_event;
                        }

                        /*
                         * The application will first attempt to handle the event using the
                         * BLE service framework.
                         * If no handler is specified in the BLE service framework, the
                         * event may be handled in the switch statement that follows.
                         * If no handler is specified in the switch statement, the event will be
                         * handled by the default event handler.
                         */
                        if (!ble_service_handle_event(hdr)) {
                                switch (hdr->evt_code) {
                                case BLE_EVT_GAP_CONNECTED:
                                        handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_DISCONNECTED:
                                        handle_evt_gap_disconnected((ble_evt_gap_disconnected_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_ADV_COMPLETED:
                                        handle_evt_gap_adv_completed((ble_evt_gap_adv_completed_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_PAIR_REQ:
                                        handle_evt_gap_pair_req((ble_evt_gap_pair_req_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_PAIR_COMPLETED:
                                        handle_evt_gap_pair_completed((ble_evt_gap_pair_completed_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_SEC_LEVEL_CHANGED:
                                        handle_evt_gap_sec_level_changed((ble_evt_gap_sec_level_changed_t *) hdr);
                                        break;
                                case BLE_EVT_GATTS_READ_REQ:
                                        handle_evt_gatts_read_request((ble_evt_gatts_read_req_t *) hdr);
                                        break;
                                case BLE_EVT_GATTS_WRITE_REQ:
                                        handle_evt_gatts_write_request((ble_evt_gatts_write_req_t *) hdr);
                                        break;

#if dg_configSUOTA_SUPPORT && defined (SUOTA_PSM)
                                case BLE_EVT_L2CAP_CONNECTED:
                                case BLE_EVT_L2CAP_DISCONNECTED:
                                case BLE_EVT_L2CAP_DATA_IND:
                                        suota_l2cap_event(suota, hdr);
                                        break;
#endif
                                default:
                                        ble_handle_event_default(hdr);
                                        break;
                                }
                        }

                        /* Free event buffer (it's not needed anymore) */
                        OS_FREE(hdr);

no_event:
                        /*
                         * If there are more events waiting in queue, application should process
                         * them now.
                         */
                        if (ble_has_event()) {
                                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_MASK,
                                                                                OS_NOTIFY_SET_BITS);
                        }
                }

                if (notif & TIMER0_NOTIF) {
                        mitimer_callback(0);
                }

                if (notif & TIMER1_NOTIF) {
                        mitimer_callback(1);
                }

                if (notif & TIMER2_NOTIF) {
                        mitimer_callback(2);
                }

                if (notif & TIMER3_NOTIF) {
                        mitimer_callback(3);
                }
        }
}
