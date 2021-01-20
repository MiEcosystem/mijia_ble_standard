/***************************************************************************//**
 * @file
 * @brief custom_mi_config.h
 *
 ******************************************************************************/
#ifndef CUSTOM_MI_CONFIG_H
#define CUSTOM_MI_CONFIG_H

/**
 * @note Device firmware version. It'll be filled in Version character appended to
 * mijia ble libs version.
 */
#define DEVELOPER_VERSION      0001

/**
 * @note Product identification got from xiaomi IoT developer platform.
 */
#define PRODUCT_ID             156
#define MODEL_NAME             "xiaomi.dev.ble"

/**
 * @note Device access method : BLE or Mesh.
 */
//#define MI_MESH_ENABLED        0

#define USE_GATT_SPEC          1
#define USE_MIBLE_OTA          1

#define USE_MCU_OTA            0
#define MCU_OTA_DEMO           0
/**
 * @note Device side has RESET button or not.
 */
#define HAVE_RESET_BUTTON      1

/**
 * @note Device side has bind confirm button.
 */
#define HAVE_CONFIRM_BUTTON    1

/**
 * @note Process mi scheduler in main loop (non-interrupt context).
 */
#define MI_SCHD_PROCESS_IN_MAIN_LOOP    1

#define DFU_NVM_START           (288 * 1024UL)                         // 16KB loader + 272KB app
#define DFU_NVM_SIZE            (192 * 1024UL)                         // 192KB OTA download cache

/* DEBUG */
#define DEBUG_MIBLE            0
#define TIME_PROFILE           0
#define RXFER_VERBOSE          0

#endif
