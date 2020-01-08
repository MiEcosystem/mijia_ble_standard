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
#define DEVELOPER_VERSION       0002


/**
 * @note Product identification got from xiaomi IoT developer platform.
 */
#define PRODUCT_ID              156   // xiaomi BLE devboard


/**
 * @note Device access method : BLE or Mesh.
 */
#define MI_BLE_ENABLED


/**
 * @note To support Secure Auth procedure MUST have mijia secure chip (MSC).
 * If device use Standard auth or Mesh Auth, it should be 0.
 *      NONE   : 0
 *      MJSC   : 1
 *      MJA1   : 2
 */
#define HAVE_MSC               0


/**
 * @note Device side has RESET button or not.
 */
#define HAVE_RESET_BUTTON      1


/**
 * @note Process mi scheduler in main loop (non-interrupt context).
 */
#define MI_SCHD_PROCESS_IN_MAIN_LOOP    1

/* DEBUG */
#define DEBUG_MIBLE            0
#define TIME_PROFILE           0
#define RXFER_VERBOSE          0

#define MAX_ATT_MTU            247

#endif
