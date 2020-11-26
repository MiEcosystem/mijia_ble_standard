/*
 * usart_api.h
 *
 *  Created on: 2020Äê10ÔÂ28ÈÕ
 *      Author: mi
 */

#ifndef APP_USART_API_H_
#define APP_USART_API_H_

#include "mi_config.h"
#include "mible_type.h"
#include "bg_types.h"

#if (defined(MCU_OTA_DEMO) && MCU_OTA_DEMO)
mible_status_t mible_usart_init(void);
mible_status_t mible_usart_send_buffer(const char* buffer, size_t length, bool blocking);
mible_status_t mible_usart_send_byte(uint8_t byte);
mible_status_t mible_usart_receive_buffer(uint8_t* buffer, size_t requestedLength,
                           size_t *receivedLength, bool blocking, uint32_t timeout);
mible_status_t mible_usart_receive_byte(uint8_t* byte);
mible_status_t mible_usart_receive_byte_timeout(uint8_t* byte, uint32_t timeout);
mible_status_t mible_usart_flush(bool flushTx, bool flushRx);

mible_status_t mible_usart_receive_delay_init(uint16_t length, uint32_t timeout);
mible_status_t mible_usart_receive_delay_expired(void);

#endif

#endif /* APP_USART_API_H_ */
