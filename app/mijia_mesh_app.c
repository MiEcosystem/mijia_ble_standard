/*
 * mijia_mesh_app.c
 *
 *  Created on: 2020Äê9ÔÂ8ÈÕ
 *      Author: mi
 */

#include "mijia_mesh_app.h"
#include "lightbulb.h"
#include "init_mcu.h"

#undef  MI_LOG_MODULE_NAME
#define MI_LOG_MODULE_NAME __FILE__
#include "mible_log.h"
/* Mijia BLE API and Middleware */
#include "efr32_api.h"
#include "mible_api.h"
#include "em_rtcc.h"

#include "common/mible_beacon.h"
#include "mijia_profiles/mi_service_server.h"
#include "mesh_auth/mible_mesh_auth.h"
#include "miot_model.h"

typedef struct {
    uint16_t elem_idx;
    uint16_t vendor;
    uint16_t model;
} bind_model_t;

/** global variables */
struct __PACKED {
    mi_mesh_stat_t stat;
    uint8_t quick_reboot;
    union {
        uint8_t value;
        struct {
            uint8_t reserved   :4;
            uint8_t version    :4;
        } bitmap;
    };
    uint8_t model_num;
    bind_model_t models[5];
} mesh_stat;

const uint8_t miot_spec_opcode_set[] = {
        [0] = MIOT_SPEC_OPCODE_GET_PROP,
        [1] = MIOT_SPEC_OPCODE_SET_PROP,
        [2] = MIOT_SPEC_OPCODE_SET_PROP_UNACK,
        [3] = MIOT_SPEC_OPCODE_PUB_PROP,
        [4] = MIOT_SPEC_OPCODE_SYNC_PROP,
        [5] = MIOT_SPEC_OPCODE_SYNC_PROP_ACK,
        [6] = MIOT_SPEC_OPCODE_EVENT,
        [7] = MIOT_SPEC_OPCODE_EVENT_TLV,
        [8] = MIOT_SPEC_OPCODE_ACTION,
        [9] = MIOT_SPEC_OPCODE_ACTION_ACK,
        [10]= MIOT_SPEC_OPCODE_INDICATION,
        [11]= MIOT_SPEC_OPCODE_INDICATION_ACK,
        [12] = MIJIA_MESH_CONFIG_TO_NODE,
        [13] = MIJIA_MESH_CONFIG_TO_GW
};

static uint8_t conn_handle = 0xFF;        /* handle of the last opened LE connection */
static uint16_t m_pri_elem_addr;
static uint16_t m_pri_group;
static uint8_t m_need_overwrite;
static uint8_t m_need_restart = 0;
static uint32_t m_req_pub_interval_moment;
uint8_t is_provisioned = 0;

#define CONV_RANGE(in, in_bits, out_max) ((in) * (out_max) >> (in_bits))

uint32_t rand_in(uint16_t min, uint16_t max)
{
    uint32_t delay = 0;
    memcpy(&delay, gecko_cmd_system_get_random_data(2)->data.data, 2);
    delay = CONV_RANGE(delay, 16, max);
    delay = MAX(delay, min);
    return delay;
}

static uint8_t find_model(uint16_t vendor, uint16_t model )
{
    int i;
    for ( i = 0; i < mesh_stat.model_num; i++ )
        if (vendor == mesh_stat.models[i].vendor && model == mesh_stat.models[i].model)
            break;

    return i < mesh_stat.model_num;
}

void mesh_stat_store(void *in, uint8_t len)
{
    uint16_t result = gecko_cmd_flash_ps_save(PS_MESH_STAT, len, in)->result;
    MI_ERR_CHECK(result);
}

int mesh_stat_restore(void *out, uint8_t len)
{
    struct gecko_msg_flash_ps_load_rsp_t *rsp = gecko_cmd_flash_ps_load(PS_MESH_STAT);

    if (rsp->result != 0)
        return -1;
    else
        memcpy(out, rsp->value.data, MIN(len, rsp->value.len));

    return 0;
}

/*-----------------Vendor tid--------------------*/
#define MEMBER_OFFSET(struct_type, member)      ((uint32_t)&((struct_type *)0)->member)

#define CONTAINER_OF(member_ptr, struct_type, member)                       \
    ({                                                                      \
        (struct_type *)((char *)member_ptr - MEMBER_OFFSET(struct_type, member)); \
    })

typedef struct _mi_tid_list
{
    struct _mi_tid_list *pprev;
    struct _mi_tid_list *pnext;
} mi_tid_list_t;

typedef struct{
    uint8_t siid;
    uint8_t piid;
    uint8_t last_tid;
    mi_tid_list_t node;
} mi_vendor_tid_t;

static mi_tid_list_t mi_tid_vendor_list_head;
static uint8_t m_gw_tid, m_node_tid, request_tid;
static bool m_node_tid_init = false;

uint8_t get_new_tid(void)
{
    if(false == m_node_tid_init){
        m_node_tid = rand_in(0,255);
        if ((NULL == mi_tid_vendor_list_head.pprev) &&
        (NULL == mi_tid_vendor_list_head.pnext))
        {
            mi_tid_vendor_list_head.pprev = &mi_tid_vendor_list_head;
            mi_tid_vendor_list_head.pnext = &mi_tid_vendor_list_head;
        }
        m_node_tid_init = true;
    }else{
        m_node_tid++;
    }
    return m_node_tid;
}

static void mi_tid_add_to_vendor(mi_vendor_tid_t *ppub)
{
    mi_tid_vendor_list_head.pprev->pnext = &ppub->node;
    ppub->node.pprev = mi_tid_vendor_list_head.pprev;
    mi_tid_vendor_list_head.pprev = &ppub->node;
    ppub->node.pnext = &mi_tid_vendor_list_head;
}

static mi_vendor_tid_t *mi_tid_vendor_get(mi_tid_list_t *phead, uint8_t siid, uint8_t piid)
{
    mi_tid_list_t *pnode = phead->pnext;
    mi_vendor_tid_t *ppub;
    for (; pnode != phead; pnode = pnode->pnext)
    {
        ppub = CONTAINER_OF(pnode, mi_vendor_tid_t, node);
        if (ppub->siid == siid && ppub->piid == piid)
        {
            return ppub;
        }
    }

    return NULL;
}

uint8_t get_vendor_tid(uint8_t siid, uint8_t piid)
{
    /* get new vendor tid */
    uint8_t res = get_new_tid();

    mi_vendor_tid_t *ppub = mi_tid_vendor_get(&mi_tid_vendor_list_head,siid,piid);
    if(NULL == ppub)
    {
        /* add to current vendor tid list */
        ppub = malloc(sizeof(mi_vendor_tid_t));
        if (NULL != ppub)
        {
            ppub->siid = siid;
            ppub->piid = piid;
            ppub->last_tid = res;
            mi_tid_add_to_vendor(ppub);
        }
        else
        {
            MI_LOG_INFO("vendor model add tid fail: id 0x%02x%02x!", siid, piid);
            return res;
        }
    }
    else
    {
        /* same tid, get new */
        if(res == ppub->last_tid){
            res = get_new_tid();
        }
        /* save tid for every piid */
        ppub->last_tid = res;
    }
    return res;
}

uint8_t get_vendor_last_tid(uint8_t siid, uint8_t piid)
{
    /* get new vendor tid */
    uint8_t res = 0;

    if ((NULL == mi_tid_vendor_list_head.pprev) &&
        (NULL == mi_tid_vendor_list_head.pnext))
    {
            mi_tid_vendor_list_head.pprev = &mi_tid_vendor_list_head;
            mi_tid_vendor_list_head.pnext = &mi_tid_vendor_list_head;

    }

    mi_vendor_tid_t *ppub = mi_tid_vendor_get(&mi_tid_vendor_list_head,siid,piid);
    if(NULL == ppub)
    {
        /* add to current vendor tid list */
        ppub = malloc(sizeof(mi_vendor_tid_t));
        if (NULL != ppub)
        {
            ppub->siid = siid;
            ppub->piid = piid;
            ppub->last_tid = get_new_tid();
            mi_tid_add_to_vendor(ppub);

        }
        else
        {
            MI_LOG_INFO("get_vendor_last_tid fail: id 0x%02x%02x!", siid, piid);
            return res;
        }
    }
    res = ppub->last_tid;
    return res;
}

void set_gw_tid(uint8_t tid)
{
    m_gw_tid = tid;
}

uint8_t get_gw_tid(void)
{
    return m_gw_tid;
}

/**
 *  this function is called to initiate factory reset. Factory reset may be initiated
 *  by keeping one of the WSTK pushbuttons pressed during reboot. Factory reset is also
 *  performed if it is requested by the provisioner (event gecko_evt_mesh_node_reset_id)
 */
void initiate_factory_reset(void)
{
    MI_LOG_INFO("factory reset\r\n");

    /* if connection is open then close it before rebooting */
    if (conn_handle != 0xFF) {
        gecko_cmd_le_connection_close(conn_handle);
    }

    // erase mesh data
    gecko_cmd_mesh_node_reset();

    // application layer's records
    gecko_cmd_flash_ps_erase(PS_MESH_STAT);
    gecko_cmd_flash_ps_erase(PS_LIGHTBULB_STATE);

    // reboot after a small delay
    gecko_cmd_hardware_set_soft_timer(32768 / 2, TIMER_ID_FACTORY_RESET, 1);
}

// xiaomi: add ble connectable advertising - mibeacon
static void advertising_config(mi_mesh_stat_t stat)
{
    MI_LOG_INFO("advertising config.\n");

    if(MI_SUCCESS != mibeacon_adv_data_set(0, stat, NULL, 0)){
        MI_LOG_ERROR("mibeacon_data_set failed. \r\n");
    }
    return;
}

static void advertising_start(uint16_t interval)
{
    mible_gap_adv_stop();
    if (MI_SUCCESS != mibeacon_adv_start(interval)) {
        MI_LOG_ERROR("adv failed. \r\n");
    }
/*
    mible_gap_adv_param_t adv_param = (mible_gap_adv_param_t ) {
        .adv_type = MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED,
        .adv_interval_min = MS_TO_UNITS(interval, 625),
        .adv_interval_max = MS_TO_UNITS(interval+10, 625),
    };

    mible_gap_adv_stop();
    if (MI_SUCCESS != mible_gap_adv_start(&adv_param)) {
        MI_LOG_ERROR("adv failed. \r\n");
    }
*/
}

static int add_appkey_list(appkey_list_t *p_list)
{
    uint16_t result;
    aes_key_128 appkey;
    for (int i = 0; i < p_list->size; i++) {
        memcpy(appkey.data, p_list->head[i].appkey, 16);
        result = gecko_cmd_mesh_test_add_local_key(1, appkey, i, 0)->result;
        MI_ERR_CHECK(result);
    }

    return 0;
}

static int bind_model_list(model_bind_list_t *p_list)
{
    uint16_t result = 0;
    int i;
    for(i = 0; i < p_list->size; i++) {
        result = gecko_cmd_mesh_test_bind_local_model_app(
                p_list->head[i].elem_idx,
                p_list->head[i].appkey_idx,
                p_list->head[i].vendor == 0 ? 0xFFFF : p_list->head[i].vendor,
                p_list->head[i].model)->result;
        if (result != 0)
            MI_LOG_ERROR("bind model %04X, company id %04X\n", p_list->head[i].model, p_list->head[i].vendor);
        else
            MI_LOG_INFO("bind model %04X, company id %04X\n", p_list->head[i].model, p_list->head[i].vendor);

        mesh_stat.models[i].elem_idx = p_list->head[i].elem_idx;
        mesh_stat.models[i].vendor = p_list->head[i].vendor == 0 ? 0xFFFF : p_list->head[i].vendor;
        mesh_stat.models[i].model  = p_list->head[i].model;

    }

    mesh_stat.model_num = i;
    mesh_stat.bitmap.version = 2;
    return result;
}

static void vendor_model_init()
{
    uint16_t result = 0;
    // Initialize Vendor model functionality
    result = gecko_cmd_mesh_vendor_model_init(  PRIMARY_ELEM,
                                                XIAOMI_COMPANY_ID,
                                                VENDOR_SERVER_MODEL_MIOT_SPEC,
                                                1,
                                                sizeof(miot_spec_opcode_set),
                                                miot_spec_opcode_set)->result;
    MI_ERR_CHECK(result);
    // Enable Vendor model publish
    result = gecko_cmd_mesh_test_set_local_model_pub(PRIMARY_ELEM, 0, 0x038F, 0x0000, XIAOMI_GATEWAY_GROUP, DEFAULT_TTL, 0, 0, 0)->result;
    MI_ERR_CHECK(result);
}

static void request_pub_interval()
{
    uint16_t result = 0;
    vendor_mesh_config_t req = {0};
    uint8_t req_len = 2;
    req.type = N_REQ_NET_PARAM;
    req.value.net_para_req.tid = get_new_tid();
    request_tid = req.value.net_para_req.tid;
    result = gecko_cmd_mesh_vendor_model_send(PRIMARY_ELEM,
            XIAOMI_COMPANY_ID,
            VENDOR_SERVER_MODEL_MIOT_SPEC,
            XIAOMI_GATEWAY_GROUP,
            0,
            0,
            0,
            MIJIA_MESH_CONFIG_TO_GW,
            1,
            req_len,
     (void*)&req)->result;
    MI_ERR_CHECK(result);

    MI_LOG_WARNING("request pub interval\n");
}

void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    uint16_t result = 0;
    if (p_event->id == SCHD_EVT_MESH_REG_SUCCESS ) {
        aes_key_128 devkey;
        memcpy(devkey.data, p_event->data.mesh_config.p_devkey, 16);
        aes_key_128 netkey;
        memcpy(netkey.data, p_event->data.mesh_config.p_prov_data->netkey, 16);

        MI_LOG_INFO(MI_LOG_COLOR_MAGENTA"mesh config info:\n");
        MI_LOG_INFO("devkey: ");
        MI_HEXDUMP(devkey.data, 16);
        MI_LOG_INFO("netkey: ");
        MI_HEXDUMP(netkey.data, 16);
        MI_LOG_INFO("appkey list size %d, value:\n", p_event->data.mesh_config.p_appkey_list->size);
        MI_HEXDUMP(p_event->data.mesh_config.p_appkey_list->head, p_event->data.mesh_config.p_appkey_list->size * sizeof(p_event->data.mesh_config.p_appkey_list->head[0]));
        MI_LOG_INFO("bind list size %d, value:\n", p_event->data.mesh_config.p_bind_list->size);
        MI_HEXDUMP(p_event->data.mesh_config.p_bind_list->head, p_event->data.mesh_config.p_bind_list->size * sizeof(p_event->data.mesh_config.p_bind_list->head[0]));

        result = gecko_cmd_mesh_node_set_provisioning_data(
                devkey,
                netkey,
                p_event->data.mesh_config.p_prov_data->net_idx,
                p_event->data.mesh_config.p_prov_data->iv,
                p_event->data.mesh_config.p_prov_data->address,
                p_event->data.mesh_config.p_prov_data->flags & 0x01)->result;
        MI_ERR_CHECK(result);

        add_appkey_list(p_event->data.mesh_config.p_appkey_list);

        bind_model_list(p_event->data.mesh_config.p_bind_list);

        result = gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(200), TIMER_ID_SELF_CONFIG, 1)->result;
        MI_ERR_CHECK(result);

        MI_LOG_INFO("REG SUCCESS, start TIMER_ID_CONN_TIMEOUT %ds\n", MI_GATT_REGSUCC_TIMEOUT);
        result = gecko_cmd_hardware_set_soft_timer(SEC_2_TIMERTICK(MI_GATT_REGSUCC_TIMEOUT), TIMER_ID_CONN_TIMEOUT, 1)->result;
        MI_ERR_CHECK(result);

        m_need_restart = 1;

    } else if(p_event->id == SCHD_EVT_ADMIN_LOGIN_SUCCESS || p_event->id == SCHD_EVT_SHARE_LOGIN_SUCCESS) {
        MI_LOG_INFO("LOGIN SUCCESS, stop TIMER_ID_CONN_TIMEOUT\n");
        gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_CONN_TIMEOUT, 1);
    }
}

static void process_system_boot(struct gecko_cmd_packet *evt)
{
    uint16_t result;

    result = gecko_cmd_system_set_tx_power(MI_RAIL_TX_POWER)->set_power;
    MI_LOG_DEBUG("set tx power %d.\n", result);
    result = gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(1000), TIMER_ID_POLL_SECOND, false)->result;
    MI_ERR_CHECK(result);

    mesh_stat_restore(&mesh_stat, sizeof(mesh_stat));
    if(mesh_stat.stat == mi_mesh_avail)
        result = gecko_cmd_mesh_test_set_adv_scan_params(1, 32, 1, 0, 7, MI_GAP_SCAN_INTERVAL, MI_GAP_SCAN_WINDOW)->result;
    else
        result = gecko_cmd_mesh_test_set_adv_scan_params(1, 32, 1, 0, 7, MI_GAP_SCAN_INTERVAL, MI_GAP_SCAN_PARAM_MIN)->result;
    MI_ERR_CHECK(result);

#ifdef MI_LOG_ENABLED
    struct gecko_msg_system_boot_evt_t boot_info = evt->data.evt_system_boot;
    MI_LOG_INFO("system stack %d.%0d.%0d-%d\n", boot_info.major, boot_info.minor, boot_info.patch, boot_info.build);
#endif

#if USE_DUMMY_CERT==1
    {
        extern unsigned char dev_mac[6];

        bd_addr new_mac = {0};
        memcpy(new_mac.addr, dev_mac, 6);

        bd_addr curr_mac = gecko_cmd_system_get_bt_address()->address;
        MI_LOG_HEXDUMP(&curr_mac, 6);
        if (memcmp(curr_mac.addr, new_mac.addr, 6) != 0) {
            gecko_cmd_system_set_bt_address(new_mac);
            gecko_cmd_system_reset(0);
        }
    }
    result = mi_mesh_otp_program_simulation();
    MI_ERR_CHECK(result);
#endif

    mi_service_init();
    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    MI_LOG_WARNING("quick boot times %d.\n", mesh_stat.quick_reboot);

    // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
    if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 ||
        mesh_stat.quick_reboot >= QUICK_BOOT_TIMES) {
        gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_POLL_SECOND, false);
        mi_scheduler_start(SYS_KEY_DELETE);
        initiate_factory_reset();
    } else {
        mi_scheduler_start(SYS_KEY_RESTORE);

        result = gecko_cmd_mesh_test_set_sar_config(10000,
                                                    PEND_ACK_BASE,
                                                    PEND_ACK_STEP,
                                                    WAIT_ACK_BASE,
                                                    WAIT_ACK_STEP,
                                                    MAX_SAR_RETRY)->result;
        MI_ERR_CHECK(result);

        result = gecko_cmd_mesh_test_set_segment_send_delay(SEGMENT_DELAY)->result;
        MI_ERR_CHECK(result);

        result = gecko_cmd_mesh_node_init()->result;
        if (result) {
            MI_LOG_ERROR("mesh stack init failed 0x%04X\n", result);
        }
    }
}

static void process_mesh_node_init_event(struct gecko_cmd_packet *evt)
{
    uint16_t result;

    MI_LOG_INFO("node initialized\r\n");

    advertising_config(mesh_stat.stat);
    advertising_start(MI_BEACON_INTERVAL_UNPROV);

    result = gecko_cmd_mesh_generic_server_init_common()->result;
    MI_ERR_CHECK(result);
    result = gecko_cmd_mesh_generic_server_init_on_off()->result;
    MI_ERR_CHECK(result);

    struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

    if (pData->provisioned) {
        m_pri_elem_addr = pData->address;
        MI_LOG_INFO("node is provisioned. address:%X, ivi:%X, SEQ:%X\n",m_pri_elem_addr, pData->ivi, \
                    gecko_cmd_mesh_test_get_element_seqnum(0)->seqnum);

        advertising_start(MI_BEACON_INTERVAL_PROVED);
        is_provisioned = 1;

        vendor_model_init();
    } else {
        setMcu_PLL();
        MI_LOG_INFO("node is unregistered\r\n");
    }

    result = gecko_cmd_mesh_test_set_nettx(NETTX_CNT, NETTX_STEP/10-1)->result;
    MI_ERR_CHECK(result);
    result = gecko_cmd_mesh_test_set_relay(RELAY_EN, RELAY_RETRANS_CNT, RELAY_STEP/10-1)->result;
    MI_ERR_CHECK(result);

#ifdef MI_LOG_ENABLED
    struct gecko_msg_mesh_test_get_nettx_rsp_t *p_nettx = gecko_cmd_mesh_test_get_nettx();
    MI_LOG_WARNING("nettx\t cnt %d\t interval %d\n", p_nettx->count, (p_nettx->interval+1)*10);

    struct gecko_msg_mesh_test_get_relay_rsp_t *p_relay = gecko_cmd_mesh_test_get_relay();
    MI_LOG_WARNING("relay\t stat %d\t cnt %d\t interval %d\n", p_relay->enabled, p_relay->count, (p_relay->interval+1)*10);

    uint8_t friend = gecko_cmd_mesh_test_get_local_config(mesh_node_friendship, 0)->data.data[0];
    uint8_t beacon = gecko_cmd_mesh_test_get_local_config(mesh_node_beacon, 0)->data.data[0];
    uint8_t ttl = gecko_cmd_mesh_test_get_local_config(mesh_node_default_ttl, 0)->data.data[0];
    uint8_t proxy = gecko_cmd_mesh_test_get_local_config(mesh_node_gatt_proxy, 0)->data.data[0];
    uint8_t identity = gecko_cmd_mesh_test_get_local_config(mesh_node_identity, 0)->data.data[0];
    MI_LOG_INFO("\n friend: %d\n beacon: %d\n ttl: %d\n proxy: %d\n identity: %d\n", friend, beacon, ttl, proxy, identity);
#endif

}

static void process_mesh_node_provisioned(struct gecko_cmd_packet *evt)
{
    uint16_t result;

    result = gecko_cmd_mesh_test_set_nettx(NETTX_CNT, NETTX_STEP/10-1)->result;
    MI_ERR_CHECK(result);
    result = gecko_cmd_mesh_test_set_relay(RELAY_EN, RELAY_RETRANS_CNT, RELAY_STEP/10-1)->result;
    MI_ERR_CHECK(result);

    is_provisioned = 1;
    MI_LOG_INFO("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);

    // xiaomi: add mi mesh stat
    mesh_stat.stat = mi_mesh_unconfig;
    mesh_stat_store(&mesh_stat, sizeof(mesh_stat));
}

static void process_mesh_node_model_config(struct gecko_cmd_packet *evt)
{
    struct gecko_msg_mesh_node_model_config_changed_evt_t changed = evt->data.evt_mesh_node_model_config_changed;
    MI_LOG_INFO("model config changed elem address %X, %s model %X, action %d \n",
                changed.element_address,
                changed.vendor_id == 0xFFFF ? "SIG" : "Vendor",
                changed.vendor_id == 0xFFFF ? changed.model_id : changed.vendor_id << 16 | changed.model_id,
                changed.mesh_node_config_state);

    if (!find_model(changed.vendor_id, changed.model_id)) {
        MI_LOG_ERROR("sub model %04X not found in model list.\n", changed.model_id);
        return;
    }

    struct gecko_msg_mesh_test_get_local_model_sub_rsp_t* p = gecko_cmd_mesh_test_get_local_model_sub(
            changed.element_address - m_pri_elem_addr, changed.vendor_id, changed.model_id);

    if (p->result == bg_err_success) {
        MI_LOG_INFO("%s group \n", p->addresses.len ? "sub" : "del");
        if (p->addresses.len) {
            MI_LOG_HEXDUMP(p->addresses.data, p->addresses.len);
            memcpy(&m_pri_group, p->addresses.data, 2);
            m_pri_group -= changed.element_address - m_pri_elem_addr;
            m_need_overwrite = 1;
        } else {
            m_need_overwrite = 0;
        }
        uint8_t payload[3] = {G_RSP_SIMPLE_SUB, m_pri_group, m_pri_group>>8};
        uint8_t result = gecko_cmd_mesh_vendor_model_send(PRIMARY_ELEM,
                                                          XIAOMI_COMPANY_ID,
                                                          VENDOR_SERVER_MODEL_MIOT_SPEC,
                                                          XIAOMI_GATEWAY_GROUP,
                                                          0,
                                                          0,
                                                          1,
                                                          MIJIA_MESH_CONFIG_TO_GW,
                                                          1,
                                                          sizeof(payload),
                                                          payload)->result;
        MI_ERR_CHECK(result);
        gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(300), TIMER_ID_SIMPLE_SUB, 1);
    }
}

static void process_vendor_config(struct gecko_cmd_packet *evt)
{
    vendor_mesh_config_t rsp = {0};
    uint8_t rsp_len = 0;
    vendor_mesh_config_t *p = (void*)evt->data.evt_mesh_vendor_model_receive.payload.data;

    switch(p->type) {
    case G_REQ_DISCOVER_NODE:
        if (get_gw_tid() != p->value.discover_req.tid) {
            set_gw_tid(p->value.discover_req.tid);
            rsp.type = G_RSP_DISCOVER_NODE;
            rsp_len = 1;
        }
        break;

    case N_RSP_NET_PARAM:
        if (request_tid == p->value.net_para_rsp.tid) {
            MI_LOG_WARNING("Periodic pub in %d min.\n", p->value.net_para_rsp.pub_interval);
            set_max_pub_interval(p->value.net_para_rsp.pub_interval);
            periodic_pub_start(rand_in(1000, 30 * 1000));
            m_req_pub_interval_moment = RTCC_CounterGet()/32768 + 4 * 3600;
        }
        break;

    default:
        MI_LOG_WARNING("recv unknown vendor mesh config message.\n");
    }

    if (rsp_len != 0) {
        uint16_t result;
        result = gecko_cmd_mesh_vendor_model_send(PRIMARY_ELEM,
                XIAOMI_COMPANY_ID,
                VENDOR_SERVER_MODEL_MIOT_SPEC,
                evt->data.evt_mesh_vendor_model_receive.source_address,
                0,
                0,
                evt->data.evt_mesh_vendor_model_receive.nonrelayed,
                MIJIA_MESH_CONFIG_TO_GW,
                1,
                rsp_len,
         (void*)&rsp)->result;
        MI_ERR_CHECK(result);
    }
}

static void process_soft_timer_event(struct gecko_cmd_packet *evt)
{
    uint16_t result;
    uint32_t systime;

    switch (evt->data.evt_hardware_soft_timer.handle) {
    case TIMER_ID_FACTORY_RESET:
        gecko_cmd_system_reset(0);
        break;

    case TIMER_ID_RESTART:
        MI_LOG_INFO("system reboot.\n");
        gecko_cmd_system_reset(0);
        break;

    case TIMER_ID_SELF_CONFIG:
        vendor_model_init();

        MI_LOG_INFO("local configuration server config.\n");
        uint8_t var = DEFAULT_TTL;
        result = gecko_cmd_mesh_test_set_local_config(mesh_node_default_ttl, 0, sizeof(var), &var)->result;
        MI_ERR_CHECK(result);

        var = 1;
        result = gecko_cmd_mesh_test_set_local_config(mesh_node_beacon, 0, sizeof(var), &var)->result;
        MI_ERR_CHECK(result);

        var = 0;
        result = gecko_cmd_mesh_test_set_local_config(mesh_node_gatt_proxy, 0, sizeof(var), &var)->result;
        MI_ERR_CHECK(result);

        var = 0;
        result = gecko_cmd_mesh_test_set_local_config(mesh_node_friendship, 0, sizeof(var), &var)->result;
        MI_ERR_CHECK(result);

        mesh_stat.stat = mi_mesh_avail;
        mesh_stat_store(&mesh_stat, sizeof(mesh_stat));

        result = gecko_cmd_mesh_test_set_adv_scan_params(1, 32, 1, 0, 7, MI_GAP_SCAN_INTERVAL, MI_GAP_SCAN_WINDOW)->result;
        MI_ERR_CHECK(result);

        advertising_config(mesh_stat.stat);
        break;

    case TIMER_ID_SIMPLE_SUB:
        for(uint8_t i = 0; i < mesh_stat.model_num; i++) {
            uint16_t result;
            struct gecko_msg_mesh_test_get_local_model_sub_rsp_t* p = gecko_cmd_mesh_test_get_local_model_sub(
                    mesh_stat.models[i].elem_idx,
                    mesh_stat.models[i].vendor,
                    mesh_stat.models[i].model);

            // fetch model sub group address.
            if (p->result != bg_err_success){
                MI_LOG_WARNING("model %04X has no sub ?\n", mesh_stat.models[i].model);
                continue;
            }

            uint16_t model_group = m_pri_group + mesh_stat.models[i].elem_idx;

            // self-config : overwrite or delete sub address.
            if (m_need_overwrite) {
                // compare current group address if it exist.
                if (p->addresses.len && memcmp(&model_group, p->addresses.data, p->addresses.len) == 0) {
                    MI_LOG_WARNING("model %04X has sub %04X !!! \n", mesh_stat.models[i].model, model_group);
                    MI_LOG_HEXDUMP(p->addresses.data, p->addresses.len);
                    continue;
                }

                MI_LOG_INFO("model %04X sub group %04X\n", mesh_stat.models[i].model, model_group);

                for (uint8_t j = 0; j < p->addresses.len / 2; j++) {
                    uint16_t addr = p->addresses.data[j] + (p->addresses.data[j+1]<<8);
                    result = gecko_cmd_mesh_test_del_local_model_sub(mesh_stat.models[i].elem_idx,
                                                                    mesh_stat.models[i].vendor,
                                                                    mesh_stat.models[i].model,
                                                                    addr)->result;
                    MI_ERR_CHECK(result);
                }

                result = gecko_cmd_mesh_test_add_local_model_sub(mesh_stat.models[i].elem_idx,
                                                                mesh_stat.models[i].vendor,
                                                                mesh_stat.models[i].model,
                                                                model_group)->result;
                MI_ERR_CHECK(result);
            } else {
                MI_LOG_INFO("model %04X delete group %04X\n", mesh_stat.models[i].model, model_group);
                result = gecko_cmd_mesh_test_del_local_model_sub(mesh_stat.models[i].elem_idx,
                                                                mesh_stat.models[i].vendor,
                                                                mesh_stat.models[i].model,
                                                                model_group)->result;
                MI_ERR_CHECK(result);
            }
        }
        uint8_t payload[3] = {G_RSP_SIMPLE_SUB, m_pri_group, m_pri_group>>8};
        uint8_t result = gecko_cmd_mesh_vendor_model_send(PRIMARY_ELEM,
                                                          XIAOMI_COMPANY_ID,
                                                          VENDOR_SERVER_MODEL_MIOT_SPEC,
                                                          XIAOMI_GATEWAY_GROUP,
                                                          0,
                                                          0,
                                                          1,
                                                          MIJIA_MESH_CONFIG_TO_GW,
                                                          1,
                                                          sizeof(payload),
                                                          payload)->result;
        MI_ERR_CHECK(result);
        break;

    case TIMER_ID_POLL_SECOND:
        systime = RTCC_CounterGet()/32768;

        // procedures run only once
        if (systime <= 1) {
            mesh_stat.quick_reboot += 1;
            mesh_stat_store(&mesh_stat, sizeof(mesh_stat));
        } else if (systime == 5) {
            mesh_stat.quick_reboot = 0;
            mesh_stat_store(&mesh_stat, sizeof(mesh_stat));
            m_req_pub_interval_moment = rand_in(10, 90);
            MI_LOG_INFO("req pub interval after %d sec\n", m_req_pub_interval_moment - systime);
        }

        // procedures run periodic
        if (is_provisioned && systime % 1800 == 0) {
            uint32_t seq_remain = gecko_cmd_mesh_node_get_seq_remaining(0)->count;
            if (seq_remain < 0x100000 && gecko_cmd_mesh_node_get_ivupdate_state()->state == 0)
                gecko_cmd_mesh_node_request_ivupdate();
        }

        if (is_provisioned && systime == m_req_pub_interval_moment) {
            m_req_pub_interval_moment += 3600;
            request_pub_interval();
        }

        break;
    case TIMER_ID_CONN_TIMEOUT:
        // disconnect from remote
        MI_LOG_INFO("Gatt timeout %d sec, disconnect handle %04x\n", MI_GATT_CONN_TIMEOUT, conn_handle);
        if (conn_handle != 0xFF) {
            result = gecko_cmd_le_connection_close(conn_handle)->result;
            MI_ERR_CHECK(result);
        }
        break;
    default:
        break;
    }
}

/**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events are handled here.
 */
void mesh_app_event_handler(struct gecko_cmd_packet *evt)
{
    uint16_t result;
    if (NULL == evt) {
        return;
    }

    switch (BGLIB_MSG_ID(evt->header)) {
    case gecko_evt_system_boot_id:
        process_system_boot(evt);
        break;

    case gecko_evt_hardware_soft_timer_id:
        process_soft_timer_event(evt);
        break;

    case gecko_evt_mesh_node_initialized_id:
        process_mesh_node_init_event(evt);
        MI_LOG_INFO("system change clock %d Hz\n", SystemCoreClockGet());
        break;

    case gecko_evt_mesh_node_provisioned_id:
        process_mesh_node_provisioned(evt);
        break;

    case gecko_evt_mesh_node_provisioning_failed_id:
        MI_LOG_INFO("provisioning failed, code 0x%04X\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
        /* start a one-shot timer that will trigger soft reset after small delay */
        gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(2000), TIMER_ID_RESTART, 1);
        break;

    case gecko_evt_mesh_node_key_added_id:
        MI_LOG_INFO("got new %s key with index %x, in network %d\n",
                    evt->data.evt_mesh_node_key_added.type == 0 ? "net" : "app",
                    evt->data.evt_mesh_node_key_added.index,
                    evt->data.evt_mesh_node_key_added.netkey_index);
        break;

    case gecko_evt_mesh_node_model_config_changed_id:
        process_mesh_node_model_config(evt);
        break;

    case gecko_evt_mesh_vendor_model_receive_id:
        if(MIJIA_MESH_CONFIG_TO_NODE == evt->data.evt_mesh_vendor_model_receive.opcode)
            process_vendor_config(evt);
        break;

    case gecko_evt_mesh_node_reset_id:
        MI_LOG_INFO("evt gecko_evt_mesh_node_reset_id\r\n");
        mi_scheduler_start(SYS_KEY_DELETE);
        initiate_factory_reset();
        break;

    case gecko_evt_mesh_node_ivrecovery_needed_id:
        gecko_cmd_mesh_node_set_ivrecovery_mode(1);
        break;

    case gecko_evt_mesh_friend_friendship_established_id:
        MI_LOG_INFO("evt gecko_evt_mesh_friend_friendship_established, lpn_address=%x\r\n", evt->data.evt_mesh_friend_friendship_established.lpn_address);
        break;

    case gecko_evt_mesh_friend_friendship_terminated_id:
        MI_LOG_INFO("evt gecko_evt_mesh_friend_friendship_terminated, reason=%x\r\n", evt->data.evt_mesh_friend_friendship_terminated.reason);
        break;

    case gecko_evt_le_connection_opened_id:
        conn_handle = evt->data.evt_le_connection_opened.connection;
        MI_LOG_INFO("evt:gecko_evt_le_connection_opened_id: %d\n", evt->data.evt_le_connection_opened.advertiser);
        result = gecko_cmd_le_gap_end_procedure()->result;
        MI_ERR_CHECK(result);
        result = gecko_cmd_le_gap_set_discovery_timing(1, MI_GAP_SCAN_INTERVAL, MI_GAP_SCAN_PARAM_MIN)->result;
        MI_ERR_CHECK(result);
        result = gecko_cmd_le_gap_start_discovery(1, 2)->result;
        MI_ERR_CHECK(result);
        result = gecko_cmd_hardware_set_soft_timer(SEC_2_TIMERTICK(MI_GATT_CONN_TIMEOUT), TIMER_ID_CONN_TIMEOUT, 1)->result;
        MI_ERR_CHECK(result);
        break;

    case gecko_evt_le_connection_closed_id:
        conn_handle = 0xFF;
        MI_LOG_INFO("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
        result = gecko_cmd_le_gap_end_procedure()->result;
        MI_ERR_CHECK(result);
        result = gecko_cmd_le_gap_set_discovery_timing(1, MI_GAP_SCAN_INTERVAL, MI_GAP_SCAN_WINDOW)->result;
        MI_ERR_CHECK(result);
        result = gecko_cmd_le_gap_start_discovery(1, 2)->result;
        MI_ERR_CHECK(result);
        result = gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_CONN_TIMEOUT, 1)->result;
        MI_ERR_CHECK(result);
        if(m_need_restart) {
            MI_LOG_INFO("evt:prov done, restart after %d ms\r\n", 200);
            result = gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(200), TIMER_ID_RESTART, 1)->result;
            MI_ERR_CHECK(result);
        }
        break;
    case gecko_evt_le_connection_parameters_id:
        MI_LOG_INFO("evt:gecko_evt_le_connection_parameters_id: conn %d, int %d, latency %d, timeout %d\n",
                evt->data.evt_le_connection_parameters.connection, evt->data.evt_le_connection_parameters.interval,
                evt->data.evt_le_connection_parameters.latency, evt->data.evt_le_connection_parameters.timeout);
        break;
    default:
        //      MI_LOG_INFO("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
        break;
    }
}

