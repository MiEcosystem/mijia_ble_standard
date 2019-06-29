/***************************************************************************//**
 * @file
 * @brief Silicon Labs SE device management interface.
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: APACHE-2.0
 *
 * This software is subject to an open source license and is distributed by
 * Silicon Laboratories Inc. pursuant to the terms of the Apache License,
 * Version 2.0 available at https://www.apache.org/licenses/LICENSE-2.0.
 * Such terms and conditions may be further supplemented by the Silicon Labs
 * Master Software License Agreement (MSLA) available at www.silabs.com and its
 * sections applicable to open source software.
 *
 ******************************************************************************/

#ifndef SE_MANAGEMENT_H
#define SE_MANAGEMENT_H

/***************************************************************************//**
 * \addtogroup sl_se
 * \{
 ******************************************************************************/

/***************************************************************************//**
 * \addtogroup sl_se_management SE device instance management
 * \brief Management functions for the SE. These functions take care
 *        of not having two 'owners' simultaneously for the same mailbox,
 *        which could potentially be causing conflicts and system lock-up.
 * \{
 ******************************************************************************/

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <stdlib.h>

#include "em_device.h"

#if defined( SEMAILBOX_PRESENT )
#include "em_se.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief		   Get ownership of the SE mailbox
 *
 * \return         0 if successful, negative on error
 */
int se_management_acquire( void );

/**
 * \brief          Release ownership of the SE mailbox
 *
 * \return         0 if successful, negative on error
 */
int se_management_release( void );

#ifdef __cplusplus
}
#endif

#endif /* SEMAILBOX_PRESENT */

/** \} (end addtogroup sl_se_management) */
/** \} (end addtogroup sl_se) */

#endif /* SE_MANAGEMENT_H */
