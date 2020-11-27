/***************************************************************************//**
 * @file
 * @brief hal-config-board.h
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

#ifndef HAL_CONFIG_BOARD_H
#define HAL_CONFIG_BOARD_H

#include "em_device.h"
#include "hal-config-types.h"

/***************************************************************************//**
 * @file
 * @brief hal-config-standalone-default.h
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


// $[BUTTON]
#define BSP_BUTTON_PRESENT                   (1)

#define BSP_BUTTON0_PIN                      (0U)
#define BSP_BUTTON0_PORT                     (gpioPortB)

#define BSP_BUTTON_COUNT                     (1U)
#define BSP_BUTTON_INIT                      { { BSP_BUTTON0_PORT, BSP_BUTTON0_PIN } }
#define BSP_BUTTON_GPIO_DOUT                 (HAL_GPIO_DOUT_LOW)
#define BSP_BUTTON_GPIO_MODE                 (HAL_GPIO_MODE_INPUT)
// [BUTTON]$

// $[CMU]
#define USE_HFXO_AS_SYSCLK					 (1)
#define BSP_CLK_HFXO_PRESENT                 (1)
#define BSP_CLK_HFXO_FREQ                    (38400000UL)
#define BSP_CLK_HFXO_INIT                     CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE                   (120)
#define BSP_CLK_LFXO_PRESENT                 (0)
#define BSP_CLK_LFXO_INIT                     CMU_LFXOINIT_DEFAULT
#define BSP_CLK_LFXO_FREQ                    (32768U)
#define BSP_CLK_LFXO_CTUNE                   (37U)
// [CMU]$

#warning "Following pin mappings need to be set for your custom board when using printf..>"
// $[DCDC]
#define BSP_DCDC_PRESENT                     (1)

#define BSP_DCDC_INIT                         EMU_DCDCINIT_DEFAULT
// [DCDC]$
// $[GPIO]
#define PORTIO_GPIO_SWV_PIN                  (3U)
#define PORTIO_GPIO_SWV_PORT                 (gpioPortA)

#define BSP_TRACE_SWO_PIN                    (3U)
#define BSP_TRACE_SWO_PORT                   (gpioPortA)

// [GPIO]$
// $[LED]
#define BSP_LED_PRESENT                      (1)

#define BSP_LED0_PIN                         (1U)
#define BSP_LED0_PORT                        (gpioPortB)

#define BSP_LED_COUNT                        (1U)
#define BSP_LED_INIT                         { { BSP_LED0_PORT, BSP_LED0_PIN } }
#define BSP_LED_POLARITY                     (1)
// [LED]$
// $[PA]
//#define BSP_PA_VOLTAGE                       (1800U)
// [PA]$
// $[SERIAL]
#define BSP_SERIAL_APP_PORT                           (HAL_SERIAL_PORT_USART1)
#define BSP_SERIAL_APP_CTS_PIN                        (4U)
#define BSP_SERIAL_APP_CTS_PORT                       (gpioPortA)

#define BSP_SERIAL_APP_RX_PIN                         (6U)
#define BSP_SERIAL_APP_RX_PORT                        (gpioPortA)

#define BSP_SERIAL_APP_TX_PIN                         (5U)
#define BSP_SERIAL_APP_TX_PORT                        (gpioPortA)

#define BSP_SERIAL_APP_RTS_PIN                        (1U)
#define BSP_SERIAL_APP_RTS_PORT                       (gpioPortC)

#define HAL_SERIAL_APP_BAUD_RATE                      (115200UL)
#define HAL_SERIAL_APP_FLOW_CONTROL                   (HAL_USART_FLOW_CONTROL_NONE)

// [SERIAL]$

// $[UARTNCP]
#define BSP_UARTNCP_USART_PORT                        (HAL_SERIAL_PORT_USART0)
#define BSP_UARTNCP_CTS_PIN                           (4U)
#define BSP_UARTNCP_CTS_PORT                          (gpioPortA)

#define BSP_UARTNCP_RX_PIN                            (6U)
#define BSP_UARTNCP_RX_PORT                           (gpioPortA)

#define BSP_UARTNCP_TX_PIN                            (5U)
#define BSP_UARTNCP_TX_PORT                           (gpioPortA)

#define BSP_UARTNCP_RTS_PIN                           (1U)
#define BSP_UARTNCP_RTS_PORT                          (gpioPortC)

// [UARTNCP]$

// $[USART0]
#define PORTIO_USART0_CTS_PIN                         (4U)
#define PORTIO_USART0_CTS_PORT                        (gpioPortA)

#define PORTIO_USART0_RTS_PIN                         (1U)
#define PORTIO_USART0_RTS_PORT                        (gpioPortC)

#define PORTIO_USART0_RX_PIN                          (6U)
#define PORTIO_USART0_RX_PORT                         (gpioPortA)

#define PORTIO_USART0_TX_PIN                          (5U)
#define PORTIO_USART0_TX_PORT                         (gpioPortA)

#define BSP_USART0_CTS_PIN                            (4U)
#define BSP_USART0_CTS_PORT                           (gpioPortA)

#define BSP_USART0_RX_PIN                             (6U)
#define BSP_USART0_RX_PORT                            (gpioPortA)

#define BSP_USART0_TX_PIN                             (5U)
#define BSP_USART0_TX_PORT                            (gpioPortA)

#define BSP_USART0_RTS_PIN                            (1U)
#define BSP_USART0_RTS_PORT                           (gpioPortC)

// [USART0]$

// $[USART1]
#define PORTIO_USART1_CTS_PIN                (2U)
#define PORTIO_USART1_CTS_PORT               (gpioPortB)

#define PORTIO_USART1_RTS_PIN                (0U)
#define PORTIO_USART1_RTS_PORT               (gpioPortA)

#define PORTIO_USART1_RX_PIN                 (6U)
#define PORTIO_USART1_RX_PORT                (gpioPortA)

#define PORTIO_USART1_TX_PIN                 (5U)
#define PORTIO_USART1_TX_PORT                (gpioPortA)

#define BSP_USART1_TX_PIN                    (5U)
#define BSP_USART1_TX_PORT                   (gpioPortA)

#define BSP_USART1_RX_PIN                    (6U)
#define BSP_USART1_RX_PORT                   (gpioPortA)

#define BSP_USART1_CTS_PIN                   (2U)
#define BSP_USART1_CTS_PORT                  (gpioPortB)

#define BSP_USART1_RTS_PIN                   (0U)
#define BSP_USART1_RTS_PORT                  (gpioPortA)

// [USART1]$

// $[VCOM]

#define BSP_VCOM_ENABLE_PIN                  (4U)
#define BSP_VCOM_ENABLE_PORT                 (gpioPortA)

#endif /* HAL_CONFIG_BOARD_H */
