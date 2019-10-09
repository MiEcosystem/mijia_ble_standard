/**
 ****************************************************************************************
 *
 * @file custom_config_qspi_suota.h
 *
 * @brief Board Support Package. User Configuration file for cached QSPI mode.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#ifndef CUSTOM_CONFIG_QSPI_SUOTA_H_
#define CUSTOM_CONFIG_QSPI_SUOTA_H_

#include "bsp_definitions.h"

#define CONFIG_USE_BLE

/*************************************************************************************************\
 * System configuration
 */
#define dg_configUSE_LP_CLK                     ( LP_CLK_32768 )
#define dg_configEXEC_MODE                      ( MODE_IS_CACHED )
#define dg_configCODE_LOCATION                  ( NON_VOLATILE_IS_FLASH )
#define dg_configEMULATE_OTP_COPY               ( 0 )
#define dg_configUSER_CAN_USE_TIMER1            ( 0 )

#define dg_configIMAGE_FLASH_OFFSET             ( 0x20000 )


#define dg_configUSE_WDOG                       ( 1 )

#define dg_configFLASH_CONNECTED_TO             ( FLASH_CONNECTED_TO_1V8 )
#define dg_configFLASH_POWER_DOWN               ( 0 )
#define dg_configPOWER_1V8_ACTIVE               ( 1 )
#define dg_configPOWER_1V8_SLEEP                ( 1 )

#define dg_configUSE_SOC                        ( 0 )
/* Uncomment the following line to enable State-of-Charge debugging or performance test */
//#define DEBUG_SOC
#if defined(DEBUG_SOC)
        #define CONFIG_RETARGET
#endif

#define dg_configUSE_SYS_CHARGER                ( 1 )

#define dg_configUSE_SW_CURSOR                  ( 1 )

#define dg_configCACHEABLE_QSPI_AREA_LEN        ( NVMS_PARAM_PART_start - MEMORY_QSPIF_BASE )

#if (dg_configENABLE_DA1469x_AA_SUPPORT == 1)
#define USE_PARTITION_TABLE_1MB_WITH_SUOTA
#else
#define USE_PARTITION_TABLE_4MB_WITH_SUOTA
#endif




/*************************************************************************************************\
 * FreeRTOS configuration
 */
#define OS_FREERTOS                             /* Define this to use FreeRTOS */

#define SUOTA_HEAP_OVERHEAD                     ( 4096 )  /* Heap overhead while SUOTA is ongoing */

#if (dg_configUSE_SOC)
        #if defined(DEBUG_SOC)
                #define configTOTAL_HEAP_SIZE   ( 18788 + SUOTA_HEAP_OVERHEAD )  /* FreeRTOS Total Heap Size */
        #else
                #define configTOTAL_HEAP_SIZE   ( 20188 + SUOTA_HEAP_OVERHEAD )  /* FreeRTOS Total Heap Size */
        #endif
#else
        #define configTOTAL_HEAP_SIZE           ( 17488 + SUOTA_HEAP_OVERHEAD )  /* FreeRTOS Total Heap Size */
#endif

/*************************************************************************************************\
 * Peripherals configuration
 */
#define dg_configFLASH_ADAPTER                  ( 1 )
#define dg_configNVMS_ADAPTER                   ( 1 )
#define dg_configNVMS_VES                       ( 1 )
#define dg_configNVPARAM_ADAPTER                ( 1 )
#define dg_configNVPARAM_APP_AREA               ( 1 )
#define dg_configGPADC_ADAPTER                  ( 1 )
#define dg_configUSE_HW_SENSOR_NODE             ( 1 )
#define dg_configUSE_SNC_HW_GPADC               ( 1 )
#define dg_configSNC_ADAPTER                    ( 1 )


/*************************************************************************************************\
 * BLE configuration
 */
#define CONFIG_USE_BLE_SERVICES

#define dg_configBLE_CENTRAL                    ( 0 )
#define dg_configBLE_GATT_CLIENT                ( 0 )
#define dg_configBLE_OBSERVER                   ( 0 )
#define dg_configBLE_BROADCASTER                ( 0 )

#define defaultBLE_ATT_DB_CONFIGURATION         ( 0x10 )  // Peripheral Pref. Conn. Param. attribute
#define defaultBLE_PPCP_INTERVAL_MIN            ( BLE_CONN_INTERVAL_FROM_MS( 500 ) )    // 500 ms
#define defaultBLE_PPCP_INTERVAL_MAX            ( BLE_CONN_INTERVAL_FROM_MS( 750 ) )    // 750 ms
#define defaultBLE_PPCP_SLAVE_LATENCY           ( 0 )                                   // 0 events
#define defaultBLE_PPCP_SUP_TIMEOUT             ( BLE_SUPERVISION_TMO_FROM_MS( 6000 ) ) // 6000 ms

#define BLE_MAX_MISSES_ALLOWED                  ( 3 )
#define BLE_MAX_DELAYS_ALLOWED                  ( 3 )

/* Use dynamic list for BLE events instead of FreeRTOS queues */
#define BLE_MGR_USE_EVT_LIST                    ( 1 )

#define dg_configSUOTA_SUPPORT                  ( 1 )

/*
 * SUOTA loader configuration:
 * - To enable SUOTA over GATT only, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_1
 *      and leave SUOTA_PSM undefined.
 * - To enable SUOTA over GATT and L2CAP CoC, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_2
 *      and also define SUOTA_PSM to match the desired PSM. In this case the central device
 *      can use either of both according to its preference.
 */
#define SUOTA_VERSION                           ( SUOTA_VERSION_1_3 )
#define SUOTA_PSM                               ( 0x81 )

#ifndef SUOTA_PSM
        #define dg_configBLE_L2CAP_COC          ( 0 )
#endif

/* Include bsp default values */
#include "bsp_defaults.h"
/* Include middleware default values */
#include "middleware_defaults.h"
/* Include memory layout */
#include "bsp_memory_layout.h"
#endif /* CUSTOM_CONFIG_QSPI_SUOTA_H_ */
