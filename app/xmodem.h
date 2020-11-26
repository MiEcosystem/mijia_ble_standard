/**
 * @author  songyu
 * @date    2019
 * @par     Copyright (c):
 *
 *    Copyright 2019 MIoT,MI
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */


#ifndef __XMODEM_H__
#define __XMODEM_H__

#include "mi_config.h"

#if (defined(MCU_OTA_DEMO) && MCU_OTA_DEMO)
typedef enum _miio_xmodem_type {
	XMODEM = 0,
	XMODEM_1K,
} miio_xmodem_type;

typedef struct _miio_xmodem_t{
	miio_xmodem_type type;

	/*    xmodem -  134 bytes : 1[head] + 1[num] + 1[~num] +  128[data] + 2[crc] + 1[null] */
	/* xmodem-1k - 1030 bytes : 1[head] + 1[num] + 1[~num] + 1024[data] + 2[crc] + 1[null] */
	unsigned char xbuff[134];
} miio_xmodem_t;

/**
 * @brief  create xmodem instance
 *
 * @param[in]  xmodem: miio_xmodem_t struct pointer
 *             uart: miio_uart_t struct pointer created before
 * @return
 *             - XMODEM_OK: create success
 *             - XMODEM_PARAM_ERR: create failed
 */
int miio_xmodem_create_instance(miio_xmodem_t *xmodem);
/**
 * @brief  destroy xmodem instance
 *
 * @param[in]  xmodem: miio_xmodem_t struct pointer
 *
 * @return     none
 */
void miio_xmodem_destroy(miio_xmodem_t *x);

/**
 * @brief  transfer xmodem data to mcu module
 *
 * @param[in]  handle: miio_handle_t struct pointer
 * @return
 *             - XMODEM_OK: receive xmodem data success
 *             - XMODEM_ERR: receive xmodem data failed
 */
int xmodem_transfer_data(miio_xmodem_t *x, unsigned char *src, int srcsz);

#endif /* USER_OTA_ENABLE */
#endif /* __XMODEM_H__ */
