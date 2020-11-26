/*
 * miot_model.h
 *
 *  Created on: 2020Äê9ÔÂ8ÈÕ
 *      Author: mi
 */

#ifndef APP_MIOT_MODEL_H_
#define APP_MIOT_MODEL_H_

#include "em_device.h"

#define VENDOR_SERVER_MODEL_MIOT_SPEC        0x0000
#define VENDOR_SERVER_MODEL_MIJIA            0x0002

#define MIOT_SPEC_OPCODE_GET_PROP            0x01
#define MIOT_SPEC_OPCODE_SET_PROP            0x03
#define MIOT_SPEC_OPCODE_SET_PROP_UNACK      0x04
#define MIOT_SPEC_OPCODE_PUB_PROP            0x05
#define MIOT_SPEC_OPCODE_SYNC_PROP           0x06
#define MIOT_SPEC_OPCODE_SYNC_PROP_ACK       0x07
#define MIOT_SPEC_OPCODE_EVENT               0x08
#define MIOT_SPEC_OPCODE_EVENT_TLV           0x09
#define MIOT_SPEC_OPCODE_ACTION              0x0A
#define MIOT_SPEC_OPCODE_ACTION_ACK          0x0B
#define MIOT_SPEC_OPCODE_INDICATION          0x0E
#define MIOT_SPEC_OPCODE_INDICATION_ACK      0x0F
#define MIJIA_MESH_CONFIG_TO_NODE            0x3E
#define MIJIA_MESH_CONFIG_TO_GW              0x3F

#define G_RSP_SIMPLE_SUB         1
#define G_REQ_DISCOVER_NODE      3
#define G_RSP_DISCOVER_NODE      3
#define N_REQ_NET_PARAM          0x80
#define N_RSP_NET_PARAM          0x80

#define MIOT_PARAM_MAX_LEN       8

#define MIOT_VENDOR_EVENT        0x80
#define MIOT_VENDOR_ACTION       0xc0

enum {
    MIOT_NUMBER = 0,
    MIOT_BOOL,
    MIOT_FLOAT,
    MIOT_STRING,
};

typedef struct {
    uint8_t tid;
} discover_node_req_t;

typedef struct {
    uint8_t reserved[1];
} discover_node_rsp_t;

typedef struct {
    uint16_t primary_grp_addr;
} simple_subscribe_rsp_t;

typedef struct {
    uint8_t tid;
} net_param_req_t;

__PACKED_STRUCT net_param_rsp {
    uint8_t tid;
    uint8_t pub_interval;
};
typedef struct net_param_rsp net_param_rsp_t;

__PACKED_STRUCT vendor_mesh_config {
    uint8_t type;
    union {
        simple_subscribe_rsp_t   sub_rsp;
        discover_node_req_t discover_req;
        discover_node_rsp_t discover_rsp;
        net_param_req_t     net_para_req;
        net_param_rsp_t     net_para_rsp;
    } value;
};
typedef struct vendor_mesh_config vendor_mesh_config_t;

__PACKED_STRUCT vendor_mesh_get {
    uint8_t siid;
    uint8_t piid;
};
typedef struct vendor_mesh_get vendor_mesh_get_t;

__PACKED_STRUCT vendor_mesh_state {
    uint8_t siid;
    uint8_t piid;
    uint8_t payload[4];
    uint8_t tid;
    uint8_t type;
};
typedef struct vendor_mesh_state vendor_mesh_state_t;

__PACKED_STRUCT vendor_mesh_sync {
    uint8_t siid;
    uint8_t piid;
    uint8_t tid;
    uint8_t retry_times;
};
typedef struct vendor_mesh_sync vendor_mesh_sync_t;

__PACKED_STRUCT vendor_mesh_sync_ack {
    uint8_t siid;
    uint8_t piid;
    uint8_t tid;
    uint8_t type;
    uint8_t payload[4];
};
typedef struct vendor_mesh_sync_ack vendor_mesh_sync_ack_t;

__PACKED_STRUCT vendor_mesh_event {
    uint8_t siid;
    uint8_t eiid;
    uint8_t tid;
    uint8_t piid;
    uint8_t payload[4];
};
typedef struct vendor_mesh_event vendor_mesh_event_t;

__PACKED_STRUCT vendor_mesh_action {
    uint8_t siid;
    uint8_t aiid;
    uint8_t tid;
    uint8_t piid;
    uint8_t payload[4];
};
typedef struct vendor_mesh_action vendor_mesh_action_t;

typedef struct{
    uint16_t dst_addr;
    uint8_t nonrelayed;
    uint8_t opcode;
    uint8_t len;
    union{
    vendor_mesh_state_t     state;
    vendor_mesh_sync_t      sync;
    vendor_mesh_event_t     event;
    vendor_mesh_action_t    action;
    }value;
}vendor_pub_param_t;


#endif /* APP_MIOT_MODEL_H_ */
