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

#include <rail.h>
#include <string.h>
#include "coexistence-ble.h"
#include "coexistence_ll-ble.h"

// enum value is used as index in counter array. order of counter values has
// been documented in public documentation. please update API reference manual
// if you need to update the enum values or order
enum coexCounter {
  coexCounterLowPriorityRequested = 0,
  coexCounterHighPriorityRequested,
  coexCounterLowPriorityDenied,
  coexCounterHighPriorityDenied,
  coexCounterLowPriorityTxAborted,
  coexCounterHighPriorityTxAborted,
  coexCounterSize,
};

static struct {
  uint32_t counters[coexCounterSize];
  bool priority; // current priority
  bool granted; // is granted during request peried
  bool requested;
} coex_counter;

static inline void incrementCounter(enum coexCounter counter)
{
  if (coex_counter.priority) {
    counter++;
  }
  coex_counter.counters[counter]++;
}

void coex_counterRequest(bool request, bool priority)
{
  if (request) {
    coex_counter.priority = priority;
    coex_counter.granted = coex_txAllowed();
    if (!coex_counter.requested) {
      incrementCounter(coexCounterLowPriorityRequested);
    }
  }

  coex_counter.requested = request;
}

void coex_counterIncrementDenied(void)
{
  incrementCounter(coexCounterLowPriorityDenied);
}

void coex_counterGrantUpdate(bool grant)
{
  if (!coex_counter.requested) {
    return;
  }

  if (grant) {
    coex_counter.granted = true;
  } else {
    //clear requested state when not granted
    coex_counter.requested = false;

    if (RAIL_GetRadioState(RAIL_EFR32_HANDLE) == RAIL_RF_STATE_TX_ACTIVE) {
      incrementCounter(coexCounterLowPriorityTxAborted);
    }
  }
}

bool gecko_getCoexCounters(const void **ptr, uint8_t *size)
{
  if (!ptr || !size) {
    // reset counters
    memset(coex_counter.counters, 0, sizeof(coex_counter.counters));
  } else {
    *ptr = coex_counter.counters;
    *size = sizeof(coex_counter.counters);
  }

  return true;
}
