/***************************************************************************//**
 * @file
 * @brief Lightbulb module
 * This file implements a lightbulb with associated mesh models
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "lightbulb.h"
#include "mijia_mesh_app.h"

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Bluetooth stack headers */
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"

/* LED driver with support for PWM dimming */
#include "leds.h"

/* Mijia BLE API and Middleware */
#undef  MI_LOG_MODULE_NAME
#define MI_LOG_MODULE_NAME __FILE__
#include "mible_log.h"
#include "mible_api.h"

/***********************************************************************************************//**
 * @addtogroup Lightbulb
 * @{
 **************************************************************************************************/
/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * (ms)) / 1000)
#define SEC_2_TIMERTICK(s) (TIMER_CLK_FREQ * (s))

#define IS_UNICAST_ADDR(x)   (0x0001 <= (x) && (x) <= 0x7FFF)

/*******************************************************************************
 * Timer handles defines.
 ******************************************************************************/
#define TIMER_ID_SAVE_STATE               50
#define TIMER_ID_PERIOD_PUB               51

#define TIMER_ID_DELAYED_ONOFF            52
#define TIMER_ID_ONOFF_TRANSITION         53
#define TIMER_ID_ONOFF_RSP                54
#define TIMER_ID_ONOFF_PUB                55

#define TIMER_ID_DELAYED_LIGHTNESS        56
#define TIMER_ID_LIGHTNESS_TRANSITION     57
#define TIMER_ID_LIGHTNESS_RSP            58
#define TIMER_ID_LIGHTNESS_PUB            59

#define TIMER_ID_DELAYED_CTL_TEMPERATURE  60
#define TIMER_ID_CTL_TEMP_TRANSITION      61
#define TIMER_ID_CTL_TEMP_RSP             62
#define TIMER_ID_CTL_TEMP_PUB             63

#define TIMER_ID_DELAYED_CTL              64
#define TIMER_ID_CTL_TRANSITION           65
#define TIMER_ID_CTL_RSP                  66
#define TIMER_ID_CTL_PUB                  67

typedef struct {
    uint16_t element_index;
    uint16_t client_addr;
    uint16_t appkey_index;
    uint32_t trans_time;
} rsp_param_t;

static uint16 _primary_elem_index = 0xffff; /* For indexing elements of the node */
static uint16 _secondary_elem_index = 0xffff; /* For indexing elements of the node */

/***********************************************************************************************//**
 * \defgroup lightbulb_state Lightbulb State
 * \brief Manage lightbulb state.
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup lightbulb_state
 * @{
 **************************************************************************************************/

/// Lightbulb state
static PACKSTRUCT(struct lightbulb_state {
    // On/Off Server state
    uint8_t onoff_current;          /**< Current generic on/off value */
    uint8_t onoff_target;           /**< Target generic on/off value */

    // Transition Time Server state
    uint8_t transtime;              /**< Transition time */

    // On Power Up Server state
    uint8_t onpowerup;              /**< On Power Up value */

    // Lightness server
    uint16_t lightness_current;     /**< Current lightness value */
    uint16_t lightness_target;      /**< Target lightness value */
    uint16_t lightness_last;        /**< Last lightness value */
    uint16_t lightness_default;     /**< Default lightness value */

    // Primary Generic Level
    int16_t pri_level_current;      /**< Current primary generic level value */
    int16_t pri_level_target;       /**< Target primary generic level value */

    // Temperature server
    uint16_t temperature_current;   /**< Current temperature value */
    uint16_t temperature_target;    /**< Target temperature value */
    uint16_t temperature_default;   /**< Default temperature value */
    uint16_t temperature_min;       /**< Minimum temperature value */
    uint16_t temperature_max;       /**< Maximum temperature value */

    // Delta UV
    int16_t deltauv_current;        /**< Current delta UV value */
    int16_t deltauv_target;         /**< Target delta UV value */
    int16_t deltauv_default;        /**< Default delta UV value */

    // Secondary Generic Level
    int16_t sec_level_current;      /**< Current secondary generic level value */
    int16_t sec_level_target;       /**< Target secondary generic level value */
}) lightbulb_state;

/** @} (end addtogroup lightbulb_state) */

/// copy of transition delay parameter, needed for delayed on/off request
static uint32_t delayed_onoff_trans = 0;
static rsp_param_t m_rsp_onoff_params;

static uint16_t max_pub_interval = 180;
static uint16_t onoff_pub_interval;

static int lightbulb_state_load(void);
static int lightbulb_state_store(void);
static void lightbulb_state_changed(void);

/***********************************************************************************************//**
 * \defgroup mesh_models Mesh Models
 * \brief Mesh models associated with lightbulb.
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup mesh_models
 * @{
 **************************************************************************************************/

#define PUB_MODEL_NUM      1
typedef errorcode_t (*publish_fn)(uint16_t);
typedef struct {
    publish_fn pub;
    uint16_t   elem;
} pub_t;

static pub_t pub_list[PUB_MODEL_NUM];

void set_max_pub_interval(uint8_t max_pub_invl_minute)
{
    max_pub_interval = max_pub_invl_minute * 60;
}

int periodic_pub_reg(publish_fn p, uint16_t elem)
{
    for (uint8_t i = 0; i < PUB_MODEL_NUM; i++) {
        if (pub_list[i].pub == p || pub_list[i].pub == NULL) {
            pub_list[i].pub  = p;
            pub_list[i].elem = elem;
            return 0;
        }
    }
    return 1;
}

int periodic_pub_unreg(publish_fn p)
{
    for (uint8_t i = 0; i < PUB_MODEL_NUM; i++) {
        if (pub_list[i].pub == p) {
            pub_list[i].pub  = NULL;
            return 0;
        }
    }
    return 1;
}

void periodic_pub_start(uint32_t delay_ms)
{
    delay_ms += 10;
    uint32_t ticks;

    if (delay_ms > 32867)
        ticks = (delay_ms / 1000) << 16;
    else
        ticks = (delay_ms << 16) / 1000;

    MI_LOG_INFO("start period pub after %d ms.\n", delay_ms);
    gecko_cmd_hardware_set_soft_timer(ticks, TIMER_ID_PERIOD_PUB, 1);
}

void periodic_pub_stop()
{
    gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PERIOD_PUB, 1);
}

/***************************************************************************//**
 * This function convert mesh format of default transition time to milliseconds.
 *
 * @return Default transition time in milliseconds.
 ******************************************************************************/
uint32_t default_transition_time(void)
{
    return mesh_lib_transition_time_to_ms(lightbulb_state.transtime);
}

/***********************************************************************************************//**
 * \defgroup GenericOnOff
 * \brief Generic OnOff Server model.
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup GenericOnOff
 * @{
 **************************************************************************************************/

/***************************************************************************//**
 * Response to generic on/off request.
 *
 * @param[in] element_index  Server model element index.
 * @param[in] client_addr    Address of the client model which sent the message.
 * @param[in] appkey_index   The application key index used in encrypting.
 *
 * @return Status of the response operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_response(rsp_param_t *p)
{
    struct mesh_generic_state current, target;

    current.kind = mesh_generic_state_on_off;
    current.on_off.on = lightbulb_state.onoff_current;

    target.kind = mesh_generic_state_on_off;
    target.on_off.on = lightbulb_state.onoff_target;

    return mesh_lib_generic_server_response(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                            p->element_index,
                                            p->client_addr,
                                            p->appkey_index,
                                            &current,
                                            p->trans_time ? &target : NULL,
                                            p->trans_time,
                                            0x00);
}

/***************************************************************************//**
 * Update generic on/off state.
 *
 * @param[in] element_index  Server model element index.
 *
 * @return Status of the update operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_update(uint16_t element_index, uint32_t remaining_ms)
{
    struct mesh_generic_state current, target;

    current.kind = mesh_generic_state_on_off;
    current.on_off.on = lightbulb_state.onoff_current;

    target.kind = mesh_generic_state_on_off;
    target.on_off.on = lightbulb_state.onoff_target;

    return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                          element_index,
                                          &current,
                                          &target,
                                          remaining_ms);
}

static errorcode_t onoff_publish(uint16_t element_index)
{
    errorcode_t e;
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
    MI_LOG_INFO("pub onoff state: %d\n", e);
    return e;
}

/***************************************************************************//**
 * This function process the requests for the generic on/off model.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] client_addr    Address of the client model which sent the message.
 * @param[in] server_addr    Address the message was sent to.
 * @param[in] appkey_index   The application key index used in encrypting the request.
 * @param[in] request        Pointer to the request structure.
 * @param[in] transition_ms  Requested transition time (in milliseconds).
 * @param[in] delay_ms       Delay time (in milliseconds).
 * @param[in] request_flags  Message flags. Bitmask of the following:
 *                           - Bit 0: Nonrelayed. If nonzero indicates
 *                                    a response to a nonrelayed request.
 *                           - Bit 1: Response required. If nonzero client
 *                                    expects a response from the server.
 ******************************************************************************/

static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
    uint16_t rsp_delay;
    uint16_t pub_delay;

    MI_LOG_INFO("ON/OFF request: requested state=<%s>, transition=%lu, delay=%u\r\n",
                request->on_off ? "ON" : "OFF", transition_ms, delay_ms);

    /* this device NOT SUPPORT transition time */
    transition_ms = 0;

    /* Update onoff state */
    if (lightbulb_state.onoff_current == request->on_off) {
        MI_LOG_INFO("Request for current state received; no op\n");
        return;
    } else {
        MI_LOG_INFO("Turning lightbulb <%s>\r\n", request->on_off ? "ON" : "OFF");
        lightbulb_state.onoff_target = request->on_off;
        if (delay_ms > 0) {
            // a delay has been specified for the light change. Start a soft timer
            // that will trigger the change after the given delay
            // Current state remains as is for now
            // store transition parameter for later use
            delayed_onoff_trans = transition_ms;

            // delayed_onoff_request()
            gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_ONOFF, 1);
        } else if (transition_ms > 0) {
            // no delay but transition time has been set.
            if (request->on_off == MESH_GENERIC_ON_OFF_STATE_OFF) {
                led_set_state(LED_STATE_OFF);
            } else {
                led_set_state(LED_STATE_ON);
            }

            onoff_update(element_index, transition_ms);
            gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(transition_ms), TIMER_ID_ONOFF_TRANSITION, 1);
        } else {
            /* Change state immediately */
            lightbulb_state.onoff_current = lightbulb_state.onoff_target;
			
            if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
                led_set_state(LED_STATE_OFF);
            } else {
                led_set_state(LED_STATE_ON);
            }
			
            onoff_update(element_index, 0);
        }
        lightbulb_state_changed();
    }

    /* Response status to source address */
    rsp_delay = IS_UNICAST_ADDR(server_addr) ? rand_in(1, 50) : rand_in(50, 2000);
    if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
        m_rsp_onoff_params.element_index = element_index;
        m_rsp_onoff_params.client_addr   = client_addr;
        m_rsp_onoff_params.appkey_index  = appkey_index;
        m_rsp_onoff_params.trans_time    = transition_ms;
        gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(rsp_delay), TIMER_ID_ONOFF_RSP, 1);
        pub_delay = rsp_delay + MAX(delay_ms+transition_ms, 200);
    } else {
    	if (rsp_delay > delay_ms + transition_ms)
    		pub_delay = rsp_delay;
    	else
    		pub_delay = rsp_delay + delay_ms + transition_ms;
    }

    /* Publish status to publish address */
    gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(pub_delay), TIMER_ID_ONOFF_PUB, 1);

    /* Turn off periodic publish */
    periodic_pub_stop();
    onoff_pub_interval = 0;
    MI_LOG_WARNING(" onoff rsp delay %d ms\n", rsp_delay);
}
/***************************************************************************//**
 * This function is a handler for generic on/off change event.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] current        Pointer to current state structure.
 * @param[in] target         Pointer to target state structure.
 * @param[in] remaining_ms   Time (in milliseconds) remaining before transition
 *                           from current state to target state is complete.
 ******************************************************************************/
static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
    if (current->on_off.on != lightbulb_state.onoff_current) {
        MI_LOG_INFO("on-off state changed %u to %u\r\n", lightbulb_state.onoff_current, current->on_off.on);

        lightbulb_state.onoff_current = current->on_off.on;
        lightbulb_state_changed();
    } else {
        MI_LOG_INFO("dummy onoff change - same state as before\r\n");
    }
}

/***************************************************************************//**
 * This function is called when a light on/off request
 * with non-zero transition time has completed.
 ******************************************************************************/
static void onoff_transition_complete(void)
{
    // transition done -> set state, update and publish
    lightbulb_state.onoff_current = lightbulb_state.onoff_target;

    MI_LOG_INFO("transition complete. New state is %s\r\n", lightbulb_state.onoff_current ? "ON" : "OFF");

    lightbulb_state_changed();
    onoff_update(_primary_elem_index, 0);
}

/***************************************************************************//**
 * Initialization of the models supported by this node.
 * This function registers callbacks for each of the supported models.
 ******************************************************************************/
static void init_models(void)
{
    uint16_t result;
    result = mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                             0,
                                             onoff_request,
                                             onoff_change,
                                             NULL);
    MI_ERR_CHECK(result);
    result = gecko_cmd_mesh_test_set_local_model_pub(PRIMARY_ELEM, 0, 0xFFFF, 0x1000, XIAOMI_GATEWAY_GROUP, DEFAULT_TTL, 0, 0, 0)->result;
    MI_ERR_CHECK(result);
}

/** @} (end addtogroup mesh_models) */

/***********************************************************************************************//**
 * @addtogroup lightbulb_state
 * @{
 **************************************************************************************************/

/***************************************************************************//**
 * This function loads the saved light state from Persistent Storage and
 * copies the data in the global variable lightbulb_state.
 * If PS key with ID 0x4004 does not exist or loading failed,
 * lightbulb_state is set to zero and some default values are written to it.
 *
 * @return 0 if loading succeeds. -1 if loading fails.
 ******************************************************************************/
static int lightbulb_state_load(void)
{
    struct gecko_msg_flash_ps_load_rsp_t* pLoad;

    pLoad = gecko_cmd_flash_ps_load(PS_LIGHTBULB_STATE);

    if (pLoad->result != bg_err_success) {
        memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
        lightbulb_state.lightness_last = 0xFFFF;
        lightbulb_state.onpowerup = MESH_GENERIC_ON_POWER_UP_STATE_RESTORE;
        return -1;
    }

    memcpy(&lightbulb_state, pLoad->value.data, pLoad->value.len);

    return 0;
}

/***************************************************************************//**
 * This function saves the current light state in Persistent Storage so that
 * the data is preserved over reboots and power cycles.
 * The light state is hold in a global variable lightbulb_state.
 * A PS key with ID 0x4004 is used to store the whole struct.
 *
 * @return 0 if saving succeed, -1 if saving fails.
 ******************************************************************************/
static int lightbulb_state_store(void)
{
    struct gecko_msg_flash_ps_save_rsp_t* pSave;

    pSave = gecko_cmd_flash_ps_save(PS_LIGHTBULB_STATE, sizeof(struct lightbulb_state), (const uint8*)&lightbulb_state);

    if (pSave->result) {
        MI_LOG_INFO("lightbulb_state_store(): PS save failed, code %x\r\n", pSave->result);
        return(-1);
    }

    return 0;
}

/***************************************************************************//**
 * This function is called each time the lightbulb state in RAM is changed.
 * It sets up a soft timer that will save the state in flash after small delay.
 * The purpose is to reduce amount of unnecessary flash writes.
 ******************************************************************************/
static void lightbulb_state_changed(void)
{
    gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(5000), TIMER_ID_SAVE_STATE, 1);
}

void lightbulb_onoff_toogle(void)
{
	if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
		led_set_state(LED_STATE_ON);
		lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
	} else {
		led_set_state(LED_STATE_OFF);
		lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
	}

	onoff_update(_primary_elem_index, 0);
	lightbulb_state_changed();

	uint16_t pub_delay = rand_in(1, 50);
	gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(pub_delay), TIMER_ID_ONOFF_PUB, 1);

	/* Turn off periodic publish */
	periodic_pub_stop();
	onoff_pub_interval = 0;
	MI_LOG_WARNING(" onoff toogle %d, pub delay %d ms\n", lightbulb_state.onoff_current, pub_delay);
}

/*******************************************************************************
 * Lightbulb state initialization.
 * This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 ******************************************************************************/
void lightbulb_state_init(void)
{
    /* Initialize mesh lib */
    mesh_lib_init(malloc, free, 2);

    _primary_elem_index = 0;   // index of primary element is zero.
    _secondary_elem_index = 1; // index of secondary element is one.

    memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
    if (lightbulb_state_load() != 0) {
        MI_LOG_INFO("lightbulb_state_load() failed, using defaults\r\n");
    }

    switch (lightbulb_state.onpowerup) {
    case MESH_GENERIC_ON_POWER_UP_STATE_OFF:
        MI_LOG_INFO("On power up state is OFF\r\n");
        lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
        lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;
        led_set_state(LED_STATE_OFF);
        break;

    case MESH_GENERIC_ON_POWER_UP_STATE_ON:
        MI_LOG_INFO("On power up state is ON\r\n");
        lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
        lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_ON;
        break;

    case MESH_GENERIC_ON_POWER_UP_STATE_RESTORE:
        MI_LOG_INFO("On power up state is RESTORE\r\n");
        if (lightbulb_state.lightness_current) {
            lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
        } else {
            lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
        }
        if (lightbulb_state.lightness_target) {
            lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_ON;
        } else {
            lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;
        }
        break;
    }

    lightbulb_state_changed();
    init_models();
    onoff_update(_primary_elem_index, 0);

    // register app scheduled periodic pub
    periodic_pub_reg(onoff_publish, 0);
}

/** @} (end addtogroup lightbulb_state) */

uint32_t rand_enlarge(uint32_t interval, uint32_t max_interval)
{
    interval = interval * 2 + rand_in(1, 10);
    return MIN(interval, max_interval);
}

/*******************************************************************************
 *  Handling of lightbulb timer events.
 *
 *  @param[in] evt  Pointer to incoming event.
 ******************************************************************************/
void handle_lightbulb_timer_evt(struct gecko_cmd_packet *evt)
{
    static uint16_t init_pub_interval_sec = 1;
    static uint8_t pub_index = 0;
    switch (evt->data.evt_hardware_soft_timer.handle) {
    case TIMER_ID_SAVE_STATE:
        /* save the lightbulb state */
        lightbulb_state_store();
        break;

    case TIMER_ID_PERIOD_PUB:
        if (init_pub_interval_sec == 1) {
            for(uint8_t i = 0; i < PUB_MODEL_NUM; i++)
                if (pub_list[i].pub) pub_list[i].pub(pub_list[i].elem);
        } else {
            if (pub_list[pub_index].pub) pub_list[pub_index].pub(pub_list[pub_index].elem);
            pub_index = (pub_index + 1) % PUB_MODEL_NUM;
        }

        init_pub_interval_sec = rand_enlarge(init_pub_interval_sec, max_pub_interval);
        MI_LOG_INFO("next period pub after %d sec\n", init_pub_interval_sec);
        gecko_cmd_hardware_set_soft_timer(SEC_2_TIMERTICK(init_pub_interval_sec), TIMER_ID_PERIOD_PUB, 1);
        break;

    case TIMER_ID_ONOFF_TRANSITION:
        /* transition for an on/off request has completed, update the lightbulb state */
        onoff_transition_complete();
        break;

    case TIMER_ID_ONOFF_PUB:
        onoff_publish(_primary_elem_index);
        if(onoff_pub_interval == 0){
        	onoff_pub_interval = 5;
        	gecko_cmd_hardware_set_soft_timer(MS_2_TIMERTICK(200), TIMER_ID_ONOFF_PUB, 1);
        }else {
			onoff_pub_interval = rand_enlarge(onoff_pub_interval, max_pub_interval);
			if (onoff_pub_interval < max_pub_interval) {
				gecko_cmd_hardware_set_soft_timer(SEC_2_TIMERTICK(onoff_pub_interval), TIMER_ID_ONOFF_PUB, 1);
				MI_LOG_INFO("next onoff pub after %d sec\n", onoff_pub_interval);
			}else{
				periodic_pub_start(max_pub_interval * 1000);
			}
        }
        break;

    case TIMER_ID_ONOFF_RSP:
        onoff_response(&m_rsp_onoff_params);
        break;

    default:
        break;
    }
}

/** @} (end addtogroup Lightbulb) */
