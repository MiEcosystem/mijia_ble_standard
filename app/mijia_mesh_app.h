/*
 * mijia_mesh_app.h
 *
 *  Created on: 2020Äê9ÔÂ8ÈÕ
 *      Author: mi
 */

#ifndef APP_MIJIA_MESH_APP_H_
#define APP_MIJIA_MESH_APP_H_

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

#include "native_gecko.h"
#include "mijia_mesh_config.h"

#define MS_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000UL) / (RESOLUTION))

// xiaomi: define ps record id macro
#define PS_MESH_STAT        0x401F

#define PRIMARY_ELEM        0
#define SECONDARY_ELEM      1

#define TIMER_ID_RESTART                  40
#define TIMER_ID_FACTORY_RESET            41

#define TIMER_ID_SELF_CONFIG              42
#define TIMER_ID_POLL_SECOND              43
#define TIMER_ID_SIMPLE_SUB               44

#define TIMER_ID_CONN_TIMEOUT             45

typedef enum {
    mi_mesh_unreg       = 0x00,
    mi_mesh_unprov      = 0x01,
    mi_mesh_unconfig    = 0x02,
    mi_mesh_avail       = 0x03
} mi_mesh_stat_t;

uint32_t rand_in(uint16_t min, uint16_t max);

uint8_t get_new_tid(void);
uint8_t get_vendor_tid(uint8_t siid, uint8_t piid);
uint8_t get_vendor_last_tid(uint8_t siid, uint8_t piid);
void set_gw_tid(uint8_t tid);
uint8_t get_gw_tid(void);
void mesh_app_event_handler(struct gecko_cmd_packet *evt);

#endif /* APP_MIJIA_MESH_APP_H_ */
