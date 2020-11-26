/***************************************************************************//**
 * @file  leds.h
 * @brief Leds header file
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

#ifndef LEDS_H
#define LEDS_H

/***************************************************************************//**
 * @defgroup Leds Leds Module
 * @brief Leds Module Implementation
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Leds
 * @{
 ******************************************************************************/

/*******************************************************************************
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 ******************************************************************************/
#define LED_STATE_OFF    0   ///< light off (both LEDs turned off)
#define LED_STATE_ON     1   ///< light on (both LEDs turned on)
#define LED_STATE_TOGGLE 2   ///< provisioning (LEDs blinking)

/***************************************************************************//**
 * LED initialization. Configure LED pins as outputs.
 ******************************************************************************/
void led_init(void);

/***************************************************************************//**
 * Update the state of LEDs.
 *
 * @param[in] state  New state defined as LED_STATE_xxx.
 ******************************************************************************/
void led_set_state(uint8_t state);

/** @} (end addtogroup Leds) */

#endif /* LEDS_H */
