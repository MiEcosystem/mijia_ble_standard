/**
 ****************************************************************************************
 *
 * @file app_nvparam.h
 *
 * @brief Configuration of non-volatile parameters for the application
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_NVPARAM_H_
#define APP_NVPARAM_H_

#include "platform_nvparam.h"

/**
 * Tags definition for 'ble_app' area.
 */
#define TAG_BLE_APP_NAME                        0x01 // up to 29 bytes value

/**
 * 'ble_app' area definition
 *
 * Parameters length need to include 2 extra bytes for parameter header.
 * Variable-length parameters length need to include 4 extra bytes for parameter header and length.
 *
 * \note It does not follow 'ble_platform' area directly on flash to allow adding new parameters
 *       to 'ble_platform' without need to move 'ble_app'.
 */

NVPARAM_AREA(ble_app, NVMS_PARAM_PART, 0x0100)
        NVPARAM_VARPARAM(TAG_BLE_APP_NAME,                      0x0000, 33) // uint8[29]
NVPARAM_AREA_END()

#endif /* APP_NVPARAM_H_ */
