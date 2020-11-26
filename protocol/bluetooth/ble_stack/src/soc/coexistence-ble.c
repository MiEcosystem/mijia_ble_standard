/***************************************************************************//**
 * @brief Co-existence support for Bluetooth
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
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

#include <stdlib.h>
#include <string.h>
#include <em_assert.h>
#include <em_core.h>
#include <em_cmu.h>
#include <rail.h>

#include "coexistence_ll-ble.h"

#include "coexistence-ble.h"
#include "coexistence-hal.h"

struct {
  uint16_t requestWindow;
  uint8_t scheduledPriority;
  bool txAbort : 1;
  bool scheduledRequest : 1;
  bool scheduled : 1;
  bool pwmEnable : 1; // PWM is used if requested by stack
  bool pwmActive : 1; // PWM is currently active
  bool pwmToggling : 1; //PWM timer is scheduled
  bool pwmOn : 1; //PWM signal phase
  bool pwmPriority : 1;
  bool enablePriority : 1;
  bool pullResistor : 1;
  struct ll_coexConfig config;
  RAIL_MultiTimer_t timer;
  RAIL_Handle_t handle;
  CBcoexAbortTx abortTx;
  CBcoexFastRandom fastRandom;
  uint16_t requestWindowCalibration;
  uint8_t requestBackoffMax;
  COEX_ReqState_t reqState;
  COEX_ReqState_t pwmState;
} ll_coex;

static inline bool isCoexEnabled(void)
{
  return COEX_GetOptions() & COEX_OPTION_COEX_ENABLED;
}

static void setCoexOptions(COEX_Options_t mask, COEX_Options_t values)
{
  //No bits outside mask must be set
  EFM_ASSERT((~mask & values) == 0);
  //Get existing options, mask out enable bit
  COEX_Options_t options = COEX_GetOptions() & ~mask;
  COEX_SetOptions(options | values);
}

static void pwmRequest(bool request);
// Set/clear request and priority signals.
static void setRequest(bool request, uint8_t priority);
// Set/clear request and priority signals
static void coexRequest(bool request, uint8_t priority);
// Update grant signal state
static void coexUpdateGrant(bool abortTx);
// Timer event handler to set delayed COEX request
static void coexHandleTimerEvent(struct RAIL_MultiTimer *tmr,
                                 RAIL_Time_t expectedTimeOfEvent,
                                 void *cbArg);
// Timer event handler for PWM
static void coexHandlePwmTimerEvent(struct RAIL_MultiTimer *tmr,
                                    RAIL_Time_t expectedTimeOfEvent,
                                    void *cbArg);

/**
 * @brief Initialize coex from Link Layer side
 *
 */
void ll_coexSetContext(RAIL_Handle_t handle, CBcoexAbortTx abortCB, CBcoexFastRandom fastRandom)
{
  memset(&ll_coex, 0, sizeof(ll_coex));

  ll_coex.handle = handle;
  ll_coex.abortTx = abortCB;
  ll_coex.fastRandom = fastRandom;
}

uint16_t ll_coexFastRandom(void)
{
  return ll_coex.fastRandom();
}

void gecko_coexSetConfig(struct ll_coexConfig * config)
{
  memcpy(&ll_coex.config, config, sizeof(struct ll_coexConfig));
}

//set request and start pwm if in use
static void startRequest(bool request, uint8_t priority)
{
  //if either period and dutycycle are 0, do not use pwm
  if (ll_coex.pwmEnable == false
      || ll_coex.pwmActive == false
      || request == false
      || ll_coex.config.coex_pwm_period == 0
      || ll_coex.config.coex_pwm_dutycycle >= 100) {
    coexRequest(request, priority);
    return;
  } else if (ll_coex.config.coex_pwm_dutycycle == 0) {
    //If dutycycle is 0, then always disable
    coexRequest(false, priority);
    return;
  }

  //Get phase
  uint32_t period = ll_coex.config.coex_pwm_period * 1000UL;
  uint32_t phase = RAIL_GetTime() % period;
  uint32_t ontime;
  if (phase * 100 < period * ll_coex.config.coex_pwm_dutycycle) {
    ll_coex.pwmOn = true;
    ontime = period * ll_coex.config.coex_pwm_dutycycle / 100 - phase;
  } else {
    ll_coex.pwmOn = false;
    ontime = period - phase;
  }

  //Make sure the request line is off
  coexRequest(false, 0);

  ll_coex.pwmToggling = true;

  //Toggle pwm line
  pwmRequest(ll_coex.pwmOn);

  RAIL_SetMultiTimer(&ll_coex.timer,
                     ontime,
                     RAIL_TIME_DELAY,
                     &coexHandlePwmTimerEvent,
                     NULL);
}

static void coexHandlePwmTimerEvent(struct RAIL_MultiTimer *tmr,
                                    RAIL_Time_t expectedTimeOfEvent,
                                    void *cbArg)
{
  if (!ll_coex.pwmToggling) {
    return;
  }

  startRequest(ll_coex.scheduledRequest, ll_coex.scheduledPriority);
}

void ll_coexSetRequestWindow(uint16_t requestWindow)
{
  ll_coex.requestWindow = requestWindow;
}

void ll_coexUpdateGrant(bool abort)
{
  coexUpdateGrant(abort && ll_coex.txAbort);
}

void ll_coexRequestDelayed(uint32_t time, bool request, bool pwmActive, uint8_t priority)
{
  if (!isCoexEnabled()) {
    return;
  }

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();
  if (ll_coex.scheduled || ll_coex.pwmToggling) {
    ll_coex.scheduled = false;
    ll_coex.pwmToggling = false;
    RAIL_CancelMultiTimer(&ll_coex.timer);
    pwmRequest(false);
  }
  ll_coex.pwmActive = pwmActive;

  int ret = RAIL_SetMultiTimer(&ll_coex.timer,
                               time - ll_coex.requestWindow,
                               RAIL_TIME_ABSOLUTE,
                               &coexHandleTimerEvent,
                               NULL);
  if (ret) {
    // timer setting failed, request immediately
    coexRequest(request, priority);
  } else {
    // clear request now and wait for timer event to request later
    ll_coex.scheduled = true;
    ll_coex.scheduledRequest = request;
    ll_coex.scheduledPriority = priority;
    coexRequest(false, 0xff);
  }

  CORE_EXIT_ATOMIC();
}

void ll_coexRequest(bool request, bool pwmActive, uint8_t priority)
{
  if (!isCoexEnabled()) {
    return;
  }

  if (request == true
      && pwmActive == false
      && (ll_coex.reqState.coexReq & COEX_REQ_ON)
      && (priority > ll_coex.config.threshold_coex_pri)) {
    //If requesting immediately and already requested, inherit previous request without updating
    //If priority is high then allow increasing
    return;
  }

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();

  if ((ll_coex.scheduled  || ll_coex.pwmToggling) && request == false && pwmActive == false) {
    ll_coex.scheduled = false;
    ll_coex.pwmToggling = false;
    RAIL_CancelMultiTimer(&ll_coex.timer);
  }

  //cache priority for pwm
  ll_coex.scheduledPriority = priority;
  ll_coex.pwmActive = pwmActive;

  startRequest(request, priority);

  //Make sure pwm is disabled
  if (!pwmActive) {
    pwmRequest(false);
  }

  CORE_EXIT_ATOMIC();
}

static void coexHandleTimerEvent(struct RAIL_MultiTimer *tmr,
                                 RAIL_Time_t expectedTimeOfEvent,
                                 void *cbArg)
{
  if (!ll_coex.scheduled) {
    return;
  }

  ll_coex.scheduled = false;
  startRequest(ll_coex.scheduledRequest, ll_coex.scheduledPriority);
}

static void coexUpdateGrant(bool abortTx)
{
  bool grant = coex_txAllowed();
  coex_counterGrantUpdate(grant);
  RAIL_EnableTxHoldOff(ll_coex.handle, !grant);

  if (abortTx && !grant) {
    EFM_ASSERT(ll_coex.abortTx);
    ll_coex.abortTx();
  }
}

static void coexRequest(bool request, uint8_t priority)
{
  //Priority too low for pta request, make sure existing request is disabled
  if (priority > ll_coex.config.threshold_coex_req && request == true) {
    setRequest(false, priority);
    return;
  }

  setRequest(request, priority);
  if (request) {
    coexUpdateGrant(false);
  }
}

void gecko_enableCoexPullResistor(bool enable)
{
  ll_coex.pullResistor = enable;
}

static void pwmRequest(bool request)
{
  COEX_SetRequest(&ll_coex.pwmState, (request ? COEX_REQ_PWM : COEX_REQ_OFF) | (ll_coex.pwmPriority ? COEX_REQ_HIPRI : COEX_REQ_OFF), NULL);
}

static void coex_RequestCallback(COEX_Req_t coexStatus)
{
  if (coexStatus & COEX_REQCB_GRANTED) {
    coex_counterGrantUpdate(true);
  }
  if (coexStatus & COEX_REQCB_NEGATED) {
    coex_counterGrantUpdate(false);
  }
}

/**
 * Set/clear request and priority signals.
 */
static void setRequest(bool request, uint8_t priority)
{
  bool priorityState = ll_coex.enablePriority && (priority <= ll_coex.config.threshold_coex_pri);
  coex_counterRequest(request, priorityState);
  if (request) {
    COEX_SetRequest(&ll_coex.reqState, COEX_REQ_ON | (priorityState ? COEX_REQ_HIPRI : 0) | (COEX_REQCB_GRANTED | COEX_REQCB_NEGATED), &coex_RequestCallback);
  } else {
    COEX_SetRequest(&ll_coex.reqState, COEX_REQ_OFF, NULL);
  }
}

/**
 * Get the state of grant signal.
 */
bool coex_txAllowed(void)
{
  return ((COEX_GetOptions() & COEX_OPTION_HOLDOFF_ACTIVE) == 0U);
}

/**
 * Random backoff delay to avoid request collisions in shared use case.
 */
static void randomBackoffDelay(uint16_t randomDelayMaskUs)
{
  uint32_t delay = ll_coexFastRandom() & randomDelayMaskUs;
  delay += RAIL_GetTime();
  while ((int)(delay - RAIL_GetTime()) > 0) {
  }
}

static void coex_RadioCallback(COEX_Events_t events)
{
//  if (events & COEX_EVENT_GRANT_RELEASED){
//	  coex_counterGrantUpdate();
//  }

  if (events & COEX_EVENT_HOLDOFF_CHANGED) {
    coexUpdateGrant(true);
  }
  if (events & COEX_EVENT_REQUEST_DENIED) {
    coex_counterIncrementDenied();
  }
}
/**
 * Initialise the coex from application side.
 */
void gecko_initCoex(const struct gecko_coexInit *coexInit)
{
  ll_coex.requestWindowCalibration = coexInit->requestWindowCalibration;

  COEX_HAL_Init();
  COEX_SetRandomDelayCallback(&randomBackoffDelay);
  COEX_SetRadioCallback(&coex_RadioCallback);
  ll_coex.requestWindow = coexInit->requestWindow + coexInit->requestWindowCalibration;

  //Set default coex parameters
  struct ll_coexConfig cfg = GECKO_COEX_DEFAULT_CONFIG;
  gecko_coexSetConfig(&cfg);
  #ifdef HAL_COEX_DP_PULSE_WIDTH_US
  COEX_HAL_ConfigDp(HAL_COEX_DP_PULSE_WIDTH_US);
  #endif
  gecko_setCoexOptions(GECKO_COEX_OPTION_MASK
                       | GECKO_COEX_OPTION_REQUEST_BACKOFF_MASK,
                       coexInit->options);
  //Enable signal for early packet reception
  RAIL_ConfigEvents(ll_coex.handle, RAIL_EVENT_RX_SYNC1_DETECT, RAIL_EVENT_RX_SYNC1_DETECT);

#ifdef BSP_COEX_RHO_PORT
  setCoexPowerState(true);
#endif
}

RAIL_Events_t ll_radioFilterEvents(RAIL_Handle_t ll_radioHandle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_RX_SYNC1_DETECT) {
    //Request low priority if not already requested
    if (!(ll_coex.reqState.coexReq & COEX_REQ_ON)) {
      setRequest(true, ll_coex.scheduledPriority);
    }
  }
  if (events & (RAIL_EVENT_RX_PACKET_RECEIVED
                | RAIL_EVENT_RX_TIMEOUT
                | RAIL_EVENT_RX_SCHEDULED_RX_END
                | RAIL_EVENT_RSSI_AVERAGE_DONE
                | RAIL_EVENT_RX_PACKET_ABORTED)) {
    RAIL_StateTransitions_t transitions;
    RAIL_GetRxTransitions(ll_coex.handle, &transitions);
    if (transitions.success != RAIL_RF_STATE_TX) {
      setRequest(false, ll_coex.scheduledPriority);
    }
  }
  return events;
}

bool gecko_setCoexOptions(uint32_t mask, uint32_t options)
{
  if (mask & GECKO_COEX_OPTION_ENABLE) {
    bool enable = options & GECKO_COEX_OPTION_ENABLE;
    if (!enable) {
      // disable request and cancel timer
      ll_coexRequest(false, false, 0xff);
    }

    setCoexOptions(COEX_OPTION_COEX_ENABLED, enable ? COEX_OPTION_COEX_ENABLED : 0);
  }
  if (mask & GECKO_COEX_OPTION_TX_ABORT) {
    ll_coex.txAbort = options & GECKO_COEX_OPTION_TX_ABORT;
  }
  if (mask & GECKO_COEX_OPTION_PRIORITY_ENABLE) {
    ll_coex.enablePriority = options & GECKO_COEX_OPTION_PRIORITY_ENABLE;
  }
  if (mask & GECKO_COEX_OPTION_PWM_PRIORITY) {
    ll_coex.pwmPriority = options & GECKO_COEX_OPTION_PWM_PRIORITY;
  }
  if (mask & GECKO_COEX_OPTION_PWM_ENABLE) {
    ll_coex.pwmEnable = options & GECKO_COEX_OPTION_PWM_ENABLE;
  }

  if (mask & GECKO_COEX_OPTION_REQUEST_BACKOFF_MASK) {
    ll_coex.requestBackoffMax = (options & GECKO_COEX_OPTION_REQUEST_BACKOFF_MASK)
                                >> GECKO_COEX_OPTION_REQUEST_BACKOFF_SHIFT;
  }

  if (mask & GECKO_COEX_OPTION_REQUEST_WINDOW_MASK) {
    uint16_t requestWindow = ((options & GECKO_COEX_OPTION_REQUEST_WINDOW_MASK)
                              >> GECKO_COEX_OPTION_REQUEST_WINDOW_SHIFT)
                             + ll_coex.requestWindowCalibration;
    ll_coexSetRequestWindow(requestWindow);
  }

  return true;
}

SL_WEAK void coex_counterIncrementDenied(void)
{
}

SL_WEAK void coex_counterRequest(bool request, bool priority)
{
}

SL_WEAK void coex_counterGrantUpdate(bool state)
{
}
