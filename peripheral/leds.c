/***************************************************************************//**
 * @file  leds.c
 * @brief Leds implementation file
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

#include "hal-config.h"
#include "leds.h"

/***************************************************************************//**
 * @addtogroup Leds
 * @{
 ******************************************************************************/

/*******************************************************************************
 *  These defines are needed to support radio boards with active-low and
 *  active-high LED configuration.
 ******************************************************************************/
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
#define LED_DEFAULT_STATE  1
#else
/* LED GPIO is active-high */
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_DEFAULT_STATE  0
#endif

/*******************************************************************************
 * LED initialization. Configure LED pins as outputs.
 ******************************************************************************/
void led_init(void)
{
  // configure LED0 and LED1 as outputs
  GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, LED_DEFAULT_STATE);
#ifndef FEATURE_ONE_LED
  GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, LED_DEFAULT_STATE);
#endif
}

/*******************************************************************************
 * Update the state of LEDs.
 *
 * @param[in] state  New state defined as LED_STATE_xxx.
 ******************************************************************************/
void led_set_state(uint8_t state)
{
  switch (state) {
    case LED_STATE_OFF:
      TURN_LED_OFF(BSP_LED0_PORT, BSP_LED0_PIN);
#ifndef FEATURE_ONE_LED
      TURN_LED_OFF(BSP_LED1_PORT, BSP_LED1_PIN);
#endif
      break;

    case LED_STATE_ON:
      TURN_LED_ON(BSP_LED0_PORT, BSP_LED0_PIN);
#ifndef FEATURE_ONE_LED
      TURN_LED_ON(BSP_LED1_PORT, BSP_LED1_PIN);
#endif
      break;

    case LED_STATE_TOGGLE:
      GPIO_PinOutToggle(BSP_LED0_PORT, BSP_LED0_PIN);
#ifndef FEATURE_ONE_LED
      GPIO_PinOutToggle(BSP_LED1_PORT, BSP_LED1_PIN);
#endif
      break;

    default:
      break;
  }
}

/** @} (end addtogroup Leds) */
