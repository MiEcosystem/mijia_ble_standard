/*
 * mijia_mesh_config.h
 *
 *  Created on: 2020Äê9ÔÂ8ÈÕ
 *      Author: mi
 */

#ifndef APP_MIJIA_MESH_CONFIG_H_
#define APP_MIJIA_MESH_CONFIG_H_

#define MI_MANU_TEST_ENABLE         0
#define USE_DUMMY_CERT              0

#define QUICK_BOOT_TIMES    3

#define DEFAULT_TTL         5
#define RELAY_EN            0
#define RELAY_STEP          10
#define RELAY_RETRANS_CNT   7
#define NETTX_STEP          10
#define NETTX_CNT           7

#define PEND_ACK_BASE       (150+NETTX_STEP*NETTX_CNT)
#define PEND_ACK_STEP       (NETTX_STEP)
#define WAIT_ACK_BASE       (200+NETTX_STEP*NETTX_CNT)
#define WAIT_ACK_STEP       (NETTX_STEP)
#define SEGMENT_DELAY       (NETTX_STEP*NETTX_CNT + 20)
#define MAX_SAR_RETRY       3

#define XIAOMI_GATEWAY_GROUP                 0xFEFF
#define XIAOMI_COMPANY_ID                    0x038F

#define MI_VENDOR_SYNC_TIMEOUT                  3600 //3600s
#define MI_VENDOR_INDICATION_TIMEOUT            3000 //3000ms
#define MI_VENDOR_INDICATION_INTERVAL           300  //300ms

#define MI_GATT_CONN_TIMEOUT                    20   //20s
#define MI_GATT_REGSUCC_TIMEOUT                 5    //5s

/* device beacon time interval unprov */
#define MI_BEACON_INTERVAL_UNPROV               100 //100ms
/* device beacon time interval proved */
#define MI_BEACON_INTERVAL_PROVED               500 //500ms

/* max scan parameter 10.24s*/
#define MI_GAP_SCAN_PARAM_MAX                   16384
/* min scan parameter 2.5ms*/
#define MI_GAP_SCAN_PARAM_MIN                   4
/* scan window */
#define MI_GAP_SCAN_WINDOW                      40
/* scan interval */
#define MI_GAP_SCAN_INTERVAL                    160

#define MI_RAIL_TX_POWER                        80


#endif /* APP_MIJIA_MESH_CONFIG_H_ */
