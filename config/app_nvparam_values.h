/**
 ****************************************************************************************
 *
 * @file nvparam_values_app.h
 *
 * @brief Non-volatile parameters description for create_nvparam script
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#include "platform_nvparam_values.h"

/**
 * \note
 * This file is not used in regular build. It is only used by create_nvparam script to create flash
 * image to populate parameters partition with default parameter values.
 * See utilities/nvparam for more information.
 *
 */
NVPARAM_PARAM_VALUE( TAG_BLE_APP_NAME,  char,   'D', 'i', 'a', 'l', 'o', 'g', ' ', 'P', 'X', ' ',
                                                'R', 'e', 'p', 'o', 'r', 't', 'e', 'r')
